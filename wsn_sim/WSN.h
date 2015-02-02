#pragma once
#include "Device.h"
#include "RadioPacket.h"
#include <vector>

class RadioPacket;
typedef uint32_t packetHandle_t;

class WSN : public Runnable
{
public:
  WSN();
  ~WSN();

  void addDevice(Device* device) { mDevices.push_back(device); }


  packetHandle_t startTransmit(RadioPacket& packet);
  void endTransmit(packetHandle_t packetHandle);
  void abortTransmit(packetHandle_t packetHandle);

  void addReceiver(Radio* radio);
  bool removeReceiver(Radio* radio);
  bool hasReceiver(Radio* radio);

  std::vector<Radio*> getReceivingRadios(RadioPacket* packet);

  RadioPacket* getRadioPacket(packetHandle_t handle) const;

  virtual void step(uint32_t timestamp);

private:
  class PacketReceiver
  {
  public:
    PacketReceiver(Radio* radio) : mRadio(radio), mStartTime(radio->getEnvironment()->getTimestamp()){}
    void putPacket(RadioPacket* pPacket) { mPackets.push_back(pPacket); }
    bool hasPacket(RadioPacket* pPacket);

    bool packetIsCorrupted(RadioPacket* pPacket);

    Radio* mRadio;
    uint32_t mStartTime;
    std::vector<RadioPacket*> mPackets;
  };

  uint32_t mPacketCount, mPacketsDeletedCount;
  std::vector<Device*> mDevices;
  std::vector<RadioPacket*> mPackets;
  std::vector<PacketReceiver> mReceivers;
  bool mReceiverListChanged;
  
};

