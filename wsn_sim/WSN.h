#pragma once
#include "Runnable.h"
#include "RandomLib\Random.hpp"
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


  packetHandle_t startTransmit(RadioPacket& packet);
  void endTransmit(packetHandle_t packetHandle);
  void abortTransmit(packetHandle_t packetHandle);

  void addReceiver(Radio* radio);
  bool removeReceiver(Radio* radio);
  bool hasReceiver(Radio* radio);

  void setDropRate(double dropRate) { mDropRate = dropRate; }

  void addDevice(Device* device);

  RadioPacket* getRadioPacket(packetHandle_t handle) const;
  std::vector<RadioPacket*> getPacketsInFlight(void) const;

  void addConnection(Device* first, Device* second, bool strong = true);
  bool connectionExists(Device* first, Device* second);
  void removeConnection(Device* first, Device* second);
  std::vector<const Device*> getConnections(const Device* dev);
  void exportGraphViz(std::string filename, uint32_t area_diameter = 100, double spacing = 1.0);

  virtual void step(uint32_t timestamp);

private:
  typedef struct
  {
    Device *pFirst, *pSecond;
    bool symmetric;
    bool strong;
    bool is(const Device* one, const Device* two) const
    {
      return (pFirst == one && pSecond == two) || (pFirst == two && pSecond == one);
    }
  } connection_t;

  class PacketReceiver
  {
  public:
    PacketReceiver(Radio* radio);
    void putPacket(RadioPacket* pPacket);
    bool hasPacket(RadioPacket* pPacket);

    bool packetIsCorrupted(RadioPacket* pPacket) const;

    Radio* mRadio;
    uint32_t mStartTime;
    uint32_t mChannel;
    std::vector<RadioPacket*> mPackets;
  };

  uint32_t mPacketCount, mPacketsDeletedCount;
  std::vector<Device*> mDevices;
  std::vector<RadioPacket*> mPackets;
  std::vector<PacketReceiver> mReceivers;
  RandomLib::Random mRand;
  bool mReceiverListChanged;
  std::vector<connection_t> mConnections;
  double mDropRate;

  uint32_t getDevIndex(const Device* dev);
  std::vector<PacketReceiver*> getReceiversListening(RadioPacket* pPacket);
  std::vector<PacketReceiver*> getPacketReceiversInRange(RadioPacket* pPacket);
};

