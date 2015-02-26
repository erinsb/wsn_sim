#pragma once
#include "Runnable.h"
#include <stdint.h>
#include <vector>

class Device;
class RadioPacket;
class Radio;
typedef uint32_t packetHandle_t;

class WSN : public Runnable
{
public:
  WSN();
  ~WSN();

  void addDevice(Device* device);

  packetHandle_t startTransmit(RadioPacket& packet);
  void endTransmit(packetHandle_t packetHandle);
  void abortTransmit(packetHandle_t packetHandle);

  void addReceiver(Radio* radio);
  bool removeReceiver(Radio* radio);
  bool hasReceiver(Radio* radio);

  void addDevice(Device& device);

  RadioPacket* getRadioPacket(packetHandle_t handle) const;
  std::vector<RadioPacket*> getPacketsInFlight(void) const;

  void addConnection(Device* first, Device* second, bool strong = true);
  bool connectionExists(Device* first, Device* second);
  void removeConnection(Device* first, Device* second);
  std::vector<const Device*> getConnections(const Device* dev);
  void exportGraphViz(std::string filename);

  virtual void step(uint32_t timestamp);

private:
  class PacketReceiver
  {
  public:
    PacketReceiver(Radio* radio);
    void putPacket(RadioPacket* pPacket);
    bool hasPacket(RadioPacket* pPacket);

    bool packetIsCorrupted(RadioPacket* pPacket);

    Radio* mRadio;
    uint32_t mStartTime;
    std::vector<RadioPacket*> mPackets;
  };

  typedef struct
  {
    const Device *pFirst, *pSecond;
    bool strong;
    bool is(const Device* one, const Device* two) const
    {
      return (pFirst == one && pSecond == two) || (pFirst == two && pSecond == one);
    }
  } connection_t;

  uint32_t mPacketCount, mPacketsDeletedCount;
  std::vector<Device*> mDevices;
  std::vector<RadioPacket*> mPackets;
  std::vector<PacketReceiver> mReceivers;
  std::vector<connection_t> mConnections;
  bool mReceiverListChanged;
  std::mutex mReceiverListMut;

  std::vector<PacketReceiver*> getReceiversListening(RadioPacket* pPacket);
  std::vector<PacketReceiver*> getPacketReceiversInRange(RadioPacket* pPacket);
  uint32_t getDevIndex(const Device* dev);
};

