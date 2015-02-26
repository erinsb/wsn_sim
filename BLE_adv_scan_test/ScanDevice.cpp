#include "ScanDevice.h"
#include "Timer.h"
#include "Radio.h"
#include "Logger.h"

ScanDevice::ScanDevice() 
{
  mScanRsp = new uint8_t[]{ 1, 2, 3, 4, 5 };
}


ScanDevice::~ScanDevice()
{
}

void ScanDevice::start(uint32_t scanInterval, uint32_t scanTime) 
{
  mScanInterval = scanInterval;
  mScanTime = scanTime;
  if (mScanInterval < mScanTime)
    mScanInterval = mScanTime;

  
  startScanning();
}

void ScanDevice::step(uint32_t timestamp)
{
  
}

void ScanDevice::startScanning(void)
{
  if (mScanInterval != mScanTime)
    mTimer->orderRelative(mScanTime, std::bind(&ScanDevice::stopScanning, this));
  mRadio->shortToTx();
  mRadio->receive();
  mWaitingForAdv = true;

  mRadio->setTifs(150);
  mRadio->setPacket(mScanRsp, 5);
  _LOG("Start scanning");
}

void ScanDevice::stopScanning(void)
{
  mTimer->orderRelative(mScanInterval - mScanTime, std::bind(&ScanDevice::startScanning, this));
  mRadio->shortDisable();
  mRadio->disable();
}

void ScanDevice::abortRspRx()
{
  if (!mWaitingForAdv)
  {
    //just report scan rsp as failed, continue scanning
    mWaitingForAdv = true;
    _LOG("No scan rsp");
    mRadio->shortToTx();
    mRadio->setTifs(150);
  }
}

void ScanDevice::radioCallbackTx(RadioPacket* packet)
{
  //_LOG("Done TX");
  mTimer->orderRelative(600, std::bind(&ScanDevice::abortRspRx, this));
  mRadio->shortToRx();
  mRadio->setTifs(148);
}

void ScanDevice::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{
  if (packet == NULL)
  {
    _LOG("NULL PACKET");
    return;
  }
  if (mWaitingForAdv)
  {
    //do scan req
    _LOG("Got adv");
    mRadio->shortToRx();
    mRadio->setTifs(148);
    mWaitingForAdv = false;

    mWSN->addConnection(this, packet->getSender()->getDevice(), false);
  }
  else
  {
    //got scan rsp, do general scan
    _LOG("Got scan rsp");
    mRadio->shortToTx();
    mRadio->setTifs(150);
    mWaitingForAdv = true;
  }
}
