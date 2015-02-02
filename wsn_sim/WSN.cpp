#include "WSN.h"
#include "Logger.h"


WSN::WSN() : mPacketCount(0), mPacketsDeletedCount(0)
{

}


WSN::~WSN()
{
}



packetHandle_t WSN::startTransmit(RadioPacket& packet)
{
  RadioPacket* pPacket = new RadioPacket(packet);
  mPackets.push_back(pPacket);

  for (PacketReceiver& receiver : mReceivers)
  {
    receiver.putPacket(pPacket);
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
  for (uint32_t i = 0; i < mReceivers.size(); ++i)
  {
    mReceiverListChanged = false;
    PacketReceiver& recv = mReceivers.at(i);

    if (recv.hasPacket(pPacket) && !recv.packetWasCorrupted(pPacket))
    {
      //TODO
    }
  }
}

void WSN::abortTransmit(packetHandle_t packetHandle)
{

}

RadioPacket* WSN::getRadioPacket(packetHandle_t handle) const 
{
  return mPackets.at(handle - mPacketsDeletedCount);
}

std::vector<Radio*> WSN::getReceivingRadios(RadioPacket* packet)
{
  std::vector<Radio*> resultVector;
  for (PacketReceiver& receiver : mReceivers)
  {
    if (packet->getSender() != receiver.mRadio && 
       packet->getSender()->getDevice()->getDistanceTo(*receiver.mRadio->getDevice()) < packet->getMaxDistance())
    {
      resultVector.push_back(receiver.mRadio);  
    }
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
