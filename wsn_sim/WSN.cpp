#include "WSN.h"
#include "Logger.h"
#include "Radio.h"
#include <fstream>

#define GRAPHVIZ_SCALING (0.06)


WSN::WSN() : mPacketCount(0), mPacketsDeletedCount(0), mDropRate(0.0)
{

}


WSN::~WSN()
{
}

WSN::PacketReceiver::PacketReceiver(Radio* radio) : mRadio(radio), mStartTime(radio->getEnvironment()->getTimestamp()), mChannel(radio->getChannel()){}

void WSN::PacketReceiver::putPacket(RadioPacket* pPacket) { mPackets.push_back(pPacket); }


packetHandle_t WSN::startTransmit(RadioPacket& packet)
{
  RadioPacket* pPacket = new RadioPacket(packet);
  mPackets.push_back(pPacket);

  auto receivers = getPacketReceiversInRange(pPacket);

  for (PacketReceiver* receiver : receivers)
  {
    if (receiver->mChannel == pPacket->mChannel)
    { 
      receiver->putPacket(pPacket);
    }
  }

  return mPacketCount++;
}

void WSN::endTransmit(packetHandle_t packetHandle)
{
  RadioPacket* pPacket = getRadioPacket(packetHandle);
  if (pPacket == NULL)
  {
    _ERROR("Someone ended an unregistered packet transmit.");
    return;
  }

  pPacket->mEndTime = getEnvironment()->getTimestamp();

  // make copy of receivers, as mReceivers may change during iteration
  auto receivers = getReceiversListening(pPacket);

  // store it inbetween, since we delete elements as we go
  std::vector<Radio*> radios;
  std::vector<bool> corrupted;
  for (PacketReceiver* pRecv : receivers)
  {
    radios.push_back(pRecv->mRadio);
    corrupted.push_back(pRecv->packetIsCorrupted(pPacket) && (mRand.Float() > mDropRate));
  }

  for (uint8_t i = 0; i < radios.size(); ++i)
  {
    Radio* pRadio = radios[i];
    uint8_t sigStrength;
    if (pPacket->getMaxDistance() == 0.0)
    {
      sigStrength = 0;
    }
    else
    {
      sigStrength = uint8_t(pPacket->getSender()->getSignalStrength() *
        (1 -
        pRadio->getDevice()->getDistanceTo(*pPacket->getSender()->getDevice()) /
        pPacket->getMaxDistance()));
    }

   pRadio->receivePacket(pPacket, sigStrength, corrupted[i]); // will cause radio to stop RX
  }
}

void WSN::abortTransmit(packetHandle_t packetHandle)
{
  RadioPacket* pPacket = getRadioPacket(packetHandle);
  if (pPacket == NULL)
  {
    _ERROR("Someone aborted an unregistered packet transmit.");
    return;
  }

  pPacket->mEndTime = getEnvironment()->getTimestamp();

  // Packet should not be cleared from receiver lists, as it may have caused collisions that are yet to be calculated
}

void WSN::addDevice(Device* device)
{
  mDevices.push_back(device);
  device->getRadio()->setWSN(this);
  
  getEnvironment()->attachRunnable(device);
  getEnvironment()->attachRunnable(device->mRadio);
  getEnvironment()->attachRunnable(device->mTimer);
}

RadioPacket* WSN::getRadioPacket(packetHandle_t handle) const 
{
  if (mPackets.size() > handle - mPacketsDeletedCount)
    return mPackets.at(handle - mPacketsDeletedCount);
  else
    return NULL;
}

std::vector<RadioPacket*> WSN::getPacketsInFlight(void) const
{
  std::vector<RadioPacket*> resultVector;
  uint32_t now = getEnvironment()->getTimestamp();

  for (RadioPacket* p : mPackets)
  { 
    if (p->mEndTime > now)
      resultVector.push_back(p);
  }

  return resultVector;
}

void WSN::addReceiver(Radio* radio)
{
  if (!hasReceiver(radio))
  {
    mReceivers.push_back(PacketReceiver(radio)); 
  }
  else
  {
    _ERROR("Attempted to add a receiver twice");
  }
  mReceiverListChanged = true;
}

bool WSN::removeReceiver(Radio* radio)
{
  for (auto it = mReceivers.begin(); it != mReceivers.end(); it++)
  {
    if (it->mRadio == radio)
    {
      mReceivers.erase(it);
      mReceiverListChanged = true;
      return true;
    }
  }
  return false;
}

bool WSN::hasReceiver(Radio* radio)
{
  for (auto it = mReceivers.begin(); it != mReceivers.end(); it++)
  {
    if (it->mRadio == radio)
    {
      return true;
    }
  }
  return false;
}


void WSN::addConnection(Device* first, Device* second, bool strong)
{
  // replace strong value if conn already exists
  for (connection_t& conn : mConnections)
  {
    if (conn.is(first, second))
    {
      conn.strong = strong;
      if (conn.pFirst == second) // registered the other way
      {
        conn.symmetric = true;
      }
      return;
    }
  }

  connection_t conn;
  conn.pFirst = first;
  conn.pSecond = second;
  conn.strong = strong;
  conn.symmetric = false;
  mConnections.push_back(conn);
}

bool WSN::connectionExists(Device* first, Device* second)
{
  for (connection_t& conn : mConnections)
  {
    if (conn.is(first, second))
      return true;
  }
  return false;
}

void WSN::removeConnection(Device* first, Device* second)
{
  for (auto it = mConnections.begin(); it != mConnections.end(); it++)
  {
    if (it->is(first, second))
    {
      if (it->symmetric)
      {
        if (it->pFirst == first) // must reverse direction
        {
          Device* pTemp = it->pFirst;
          it->pFirst = it->pSecond;
          it->pSecond = pTemp;
        }
        it->symmetric = false;
      }
      else
      {
        mConnections.erase(it);
      }
      break;
    }
  }
}

std::vector<const Device*> WSN::getConnections(const Device* dev)
{
  std::vector<const Device*> resultVector;
  for (connection_t& conn : mConnections)
  {
    if (conn.pFirst == dev)
      resultVector.push_back(conn.pSecond); // only outgoing
  }

  return resultVector;
}

void WSN::exportGraphViz(std::string filename, uint32_t area_diameter, double spacing)
{
  std::ofstream file; 
  file.open(filename);
  double areaSqrt = sqrt(area_diameter);
  double areaSqrt2 = sqrt(areaSqrt);

  file << "strict digraph {\n";
  file << "\tnode [width = \"" << GRAPHVIZ_SCALING * areaSqrt2 << 
    "\" height =\"" << GRAPHVIZ_SCALING * areaSqrt2 << "\" label=\"\", fixedsize=false,";
  file << "fontsize = " << areaSqrt / 2 << ", fontname = \"Consolas\"]\n";
  file << "\tgraph [dpi=" << min(uint32_t(100 * (50.0 / areaSqrt)), 1000) << "]\n\n";
  

  for (uint32_t i = 0; i < mDevices.size(); ++i)
  {
    file << "\t" << i
      << " [pos=\"" << mDevices[i]->pos.x * 100 * spacing / areaSqrt << ","
      << mDevices[i]->pos.y * 100 * spacing / areaSqrt << "\", xlabel=\"" << mDevices[i]->mName << "\" ";
    switch (mDevices[i]->mDevTag)
    {
    case DEVICE_TAG_NONE:
      break;
    case DEVICE_TAG_WEAK:
      file << "style=\"filled\", color=\"gray\"";
      break;
    case DEVICE_TAG_MEDIUM:
      file << "style=\"filled\", color=\"red\"";
      break;
    case DEVICE_TAG_STRONG:
      file << "style=\"filled\", color=\"black\"";
      break;
    }

    file << "]\n";
  }

  std::vector<connection_t*> symmetric, asymmetric;
  for (connection_t& conn : mConnections)
  {
    if (conn.symmetric)
      symmetric.push_back(&conn);
    else
      asymmetric.push_back(&conn);
  }

  if (symmetric.size() > 0)
  {
    file << "\n\tsubgraph sym {\n";
    file << "\t\tedge [color=red, dir=none]\n";

    for (connection_t* pConn : symmetric)
    {
      file << "\t\t" << getDevIndex(pConn->pFirst) << " -> " << getDevIndex(pConn->pSecond);
      if (!pConn->strong)
        file << " [style=dotted]"; 
      file << "\n";
    }

    file << "\t}\n";
  }
  if (asymmetric.size() > 0)
  {
    file << "\n\tsubgraph asym {\n";
    file << "\t\tedge [arrowsize = " << 2 * GRAPHVIZ_SCALING * areaSqrt2 << "]\n";
    for (connection_t* pConn : asymmetric)
    {
      // !! reversing the direction for visual representation (the subscriber is at the end of the arrow)
      file << "\t\t" << getDevIndex(pConn->pSecond) << " -> " << getDevIndex(pConn->pFirst);
      if (!pConn->strong)
        file << " [style=dotted]"; 
      file << "\n";
    }

    file << "\t}\n";
  }
#if 0 // symmetric connections only
  for (connection_t& conn : mConnections)
  {
    file << "\t" << getDevIndex(conn.pFirst) << " -- " << getDevIndex(conn.pSecond);
    if (!conn.strong)
      file << " [style=dotted]";
    file << "\n";
  }
#endif
  file << "}";
  while(file.is_open())
    file.close();
  std::string callstr = "neato -n2 -Tpng " + filename + " -O";
  system(callstr.c_str());
}

void WSN::step(uint32_t timestamp)
{

}


bool WSN::PacketReceiver::hasPacket(RadioPacket* pPacket)
{
  for (RadioPacket* pPacketInList : mPackets)
  {
    if (pPacketInList == pPacket)
    {
      return true;
    }
  }
  return false;
}

bool WSN::PacketReceiver::packetIsCorrupted(RadioPacket* pPacket) const
{
  if (mPackets.size() == 0)
    return false;

  for (auto it = mPackets.begin(); it != mPackets.end(); it++)
  { 
    if ((*it)->collidesWith(pPacket))
    {
      return true;
    }
  }
  return false;
}

std::vector<WSN::PacketReceiver*> WSN::getReceiversListening(RadioPacket* pPacket)
{
  std::vector<PacketReceiver*> resultVector;
  for (PacketReceiver& receiver : mReceivers)
  {
    if (receiver.hasPacket(pPacket))
      resultVector.push_back(&receiver);
  }
  return resultVector;
}

std::vector<WSN::PacketReceiver*> WSN::getPacketReceiversInRange(RadioPacket* pPacket)
{
  std::vector<PacketReceiver*> resultVector;
  for (PacketReceiver& receiver : mReceivers)
  {
    if (pPacket->getSender() != receiver.mRadio &&
      pPacket->getSender()->getDevice()->getDistanceTo(*receiver.mRadio->getDevice()) < pPacket->getMaxDistance())
    {
      resultVector.push_back(&receiver);
    }
  }
  return resultVector;
}

uint32_t WSN::getDevIndex(const Device* dev)
{
  for (uint32_t i = 0; i < mDevices.size(); ++i)
  {
    if (mDevices[i] == dev)
      return i;
  }
  return 0xFFFFFFFF;
}