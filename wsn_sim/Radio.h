#pragma once
#include <stdint.h>
#include "Device.h"
#include "Runnable.h"
#include "RadioPacket.h"

#define RADIO_LOG_ENABLED (true)

/* Nordic Semiconductor nRF51822 numbers */
#define RADIO_DEFAULT_SIGSTRENGTH (100)
#define RADIO_DEFAULT_TURNAROUND  (138)
#define RADIO_DEFAULT_TIFS        (150)
#define RADIO_DEFAULT_BITRATE     (1) /* bit per us */

#define RADIO_POWER_RX            (13.0)
#define RADIO_POWER_TX            (10.5)
#define RADIO_POWER_RAMPUP_RX     (8.7)
#define RADIO_POWER_RAMPUP_TX     (7.0)
#define RADIO_POWER_IDLE          (0.0)

class Device;
class RadioPacket;


class Radio : public Runnable
{
  friend WSN;

public:
  typedef enum
  {
    RADIO_STATE_RX,
    RADIO_STATE_TX,
    RADIO_STATE_RAMPUP_RX,
    RADIO_STATE_RAMPUP_TX,
    RADIO_STATE_IDLE
  } state_t;

  Radio(
    Device* device, 
    uint8_t sigStrength, 
    uint32_t turnaroundTime_us, 
    uint32_t tifs_us, 
    uint32_t bitrate);

  ~Radio();

  void transmit(void);
  void receive(void);
  void disable(void);
  void setPacket(uint8_t* packet, uint8_t length);

  void shortToRx(void);
  void shortToTx(void);
  void shortDisable(void);

  void setWSN(WSN* wsn) { this->mWSN = wsn; }

  state_t getState(void) const { return mState; }
  uint8_t getSignalStrength(void) const { return mSigStrength; }
  uint32_t getTxTime(uint32_t packet_length) 
  { 
    return packet_length * 8 / mBitrate; 
  }

  double getX(void) { return mDevice->pos.x; }
  double getY(void) { return mDevice->pos.y; }

  Device* const getDevice(void) const { return mDevice; }

  virtual void step(uint32_t time);

private:
  typedef enum
  {
    SHORT_TO_RX,
    SHORT_TO_TX,
    SHORT_DISABLED
  }short_t;

  short_t mShort;
  state_t mState;
  Device* mDevice;
  RadioPacket mCurrentPacket;
  packetHandle_t mTxPacketHandle;
  WSN* mWSN;
  uint32_t mTurnaroundTime_us;
  uint32_t mBitrate;
  uint8_t mTifs_us;
  uint8_t mSigStrength;
  uint32_t mNextStateTime;

  static const double powerProfile[7];
  
  void receivePacket(
    RadioPacket* pPacket, 
    uint8_t rx_strength, 
    bool corrupted);
  void shortToNextState(void);

  void setState(state_t newState);
};

