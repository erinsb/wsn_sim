#include "WSN.h"
#include "Logger.h"
#include "Radio.h"

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
    LOG_ERROR << "Someone ended an unregistered packet transmit.";
    return;
  }

  pPacket->mEndTime = getEnvironment()->getTimestamp();

  // make copy of receivers, as mReceives may change during iteration
  auto receivers = getReceiversListening(pPacket);

  for (PacketReceiver* pRecv : receivers)
  {
    uint8_t sigStrength;
    if (pPacket->getMaxDistance() == 0.0)
    {
      sigStrength = 0;
    }
    else
    {
      sigStrength = uint8_t(pPacket->getSender()->getSignalStrength() *
        (1 -
        pRecv->mRadio->getDevice()->getDistanceTo(*pPacket->getSender()->getDevice()) /
        pPacket->getMaxDistance()));
    }

    pRecv->mRadio->receivePacket(pPacket, sigStrength, pRecv->packetIsCorrupted(pPacket)); // will cause radio to stop RX
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

void WSN::addDevice(Device& device)
{
  mDevices.push_back(&device);
  device.getRadio()->setWSN(this);
  
  getEnvironment()->attachRunnable(&device);
  getEnvironment()->attachRunnable(device.mRadio);
  getEnvironment()->attachRunnable(device.mTimer);
}

RadioPacket* WSN::getRadioPacket(packetHandle_t handle) const 
{
  return mPackets.at(handle - mPacketsDeletedCount);
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
    mReceivers.push_back(PacketReceiver(radio));
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

bool WSN::PacketReceiver::packetIsCorrupted(RadioPacket* pPacket)
{
  for (RadioPacket* pIncomingPacket : mPackets)
  { 
    if (pIncomingPacket->collidesWith(pPacket))
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