#include "WSN.h"
#include "Logger.h"
#include "Radio.h"
#include <fstream>

#define GRAPHVIZ_SCALING (10.0)


WSN::WSN() : mPacketCount(0), mPacketsDeletedCount(0)
{

}


WSN::~WSN()
{
}

WSN::PacketReceiver::PacketReceiver(Radio* radio) : mRadio(radio), mStartTime(radio->getEnvironment()->getTimestamp()){}

void WSN::PacketReceiver::putPacket(RadioPacket* pPacket) { mPackets.push_back(pPacket); }


packetHandle_t WSN::startTransmit(RadioPacket& packet)
{
  RadioPacket* pPacket = new RadioPacket(packet);
  mPackets.push_back(pPacket);

  auto receivers = getPacketReceiversInRange(pPacket);

  for (PacketReceiver* receiver : receivers)
  {
    receiver->putPacket(pPacket);
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
    corrupted.push_back(pRecv->packetIsCorrupted(pPacket));
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
    LOG_ERROR << "Someone aborted an unregistered packet transmit.";
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
    mReceiverListMut.lock();
    mReceivers.push_back(PacketReceiver(radio));
    mReceiverListMut.unlock();
  }
  mReceiverListChanged = true;
}

bool WSN::removeReceiver(Radio* radio)
{
  mReceiverListMut.lock();
  for (auto it = mReceivers.begin(); it != mReceivers.end(); it++)
  {
    if (it->mRadio == radio)
    {
      mReceivers.erase(it);
      mReceiverListChanged = true;
      mReceiverListMut.unlock();
      return true;
    }
  }
  mReceiverListMut.unlock();
  return false;
}

bool WSN::hasReceiver(Radio* radio)
{
  mReceiverListMut.lock();
  for (auto it = mReceivers.begin(); it != mReceivers.end(); it++)
  {
    if (it->mRadio == radio)
    {
      mReceiverListMut.unlock();
      return true;
    }
  }
  mReceiverListMut.unlock();
  return false;
}


void WSN::addConnection(Device* first, Device* second, bool strong)
{
  if (connectionExists(first, second))
    return;

  connection_t conn;
  conn.pFirst = first;
  conn.pSecond = second;
  conn.strong = strong;
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
      mConnections.erase(it);
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
      resultVector.push_back(conn.pSecond);
    else if (conn.pSecond == dev)
      resultVector.push_back(conn.pFirst);
  }

  return resultVector;
}

void WSN::exportGraphViz(std::string filename)
{
  std::ofstream file; 
  file.open(filename);

  file << "strict graph {\n";
  file << "\tnode [width = \"" << 0.025 * GRAPHVIZ_SCALING << 
    "\" height =\"" << 0.025 * GRAPHVIZ_SCALING << "\" label=\"\", fixedsize=true]\n";

  for (uint32_t i = 0; i < mDevices.size(); ++i)
  {
    file << "\t" << i
      << " [pos=\"" << GRAPHVIZ_SCALING * mDevices[i]->pos.x << "," 
      << GRAPHVIZ_SCALING * mDevices[i]->pos.y << "\"]\n";
  }

  for (connection_t& conn : mConnections)
  {
    file << "\t" << getDevIndex(conn.pFirst) << " --- " << getDevIndex(conn.pSecond);
    if (!conn.strong)
      file << " [style=dotted]";
    file << "\n";
  }
  
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
  mReceiverListMut.lock();
  std::vector<PacketReceiver*> resultVector;
  for (PacketReceiver& receiver : mReceivers)
  {
    if (receiver.hasPacket(pPacket))
      resultVector.push_back(&receiver);
  }
  mReceiverListMut.unlock();
  return resultVector;
}

std::vector<WSN::PacketReceiver*> WSN::getPacketReceiversInRange(RadioPacket* pPacket)
{
  mReceiverListMut.lock();
  std::vector<PacketReceiver*> resultVector;
  for (PacketReceiver& receiver : mReceivers)
  {
    if (pPacket->getSender() != receiver.mRadio &&
      pPacket->getSender()->getDevice()->getDistanceTo(*receiver.mRadio->getDevice()) < pPacket->getMaxDistance())
    {
      resultVector.push_back(&receiver);
    }
  }
  mReceiverListMut.unlock();
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