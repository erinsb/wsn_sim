#include "stdafx.h"
#include "CppUnitTest.h"

#include "Device.h"
#include "Radio.h"
#include "RadioPacket.h"
#include "WSN.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace wsn_sim_tests
{
  class TestDevice : public Device
  {
  public:
    bool mTxInProgress = false;
    bool mRxInProgress = false;

    TestDevice(double x, double y) : Device(x, y){}

    void doTx(uint8_t* packet, uint32_t len)
    {
      stepToIdle();
      mRadio->setPacket(packet, len);
      mTxInProgress = true;
      mRadio->transmit();
    }

    void doRx(uint8_t* expectedPacketContent, uint32_t expectedLength)
    {
      stepToIdle();
      mRxInProgress = true;
      this->mExpectedPacketContent = expectedPacketContent;
      this->mExpectedLength = expectedLength;
      mRadio->receive();
    }

    void stepToIdle(void)
    {
      uint32_t steps = 0;
      while (mRxInProgress || mTxInProgress)
      {
        getEnvironment()->run(1);
        Assert::IsFalse(steps++ > 100000, L"TX never ended");
      }
    }

  protected:
    uint8_t* mExpectedPacketContent, mExpectedLength;

    virtual void radioCallbackTx(RadioPacket* packet)
    {
      mTxInProgress = false;
    }

    virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
    {
      Assert::IsFalse(packet == NULL, L"Got NULL packet");
      Assert::IsFalse(corrupted, L"Got corrupted packet");
      Assert::AreEqual<int>(packet->getLength(), mExpectedLength, L"Wrong packetLength");
      for (uint32_t i = 0; i < packet->getLength(); ++i)
        Assert::AreEqual<int>(packet->getContents()[i], mExpectedPacketContent[i], L"Packet has changed on air");
      mRxInProgress = false;
    }
  };



	TEST_CLASS(WSNTest)
	{
	public:
    SimEnv mSimEnv;
    WSN mWSN;

    TEST_METHOD_INITIALIZE(InitWSN)
    {
      if (mSimEnv.numberOfRunnables() == 0)
      {
        mSimEnv.attachRunnable(&mWSN);
      }
    }
		
		TEST_METHOD(TransmissionTest)
		{
      TestDevice dev1(0.0, 0.0);
      TestDevice dev2(1.0, 0.0);
      mWSN.addDevice(dev1);
      mWSN.addDevice(dev2);

      uint8_t dummyData[] {1, 2, 3, 4, 5};
      dev1.doRx(dummyData, 5);
      dev2.doTx(dummyData, 5);

      //Assert::AreEqual<int>(mWSN.getPacketsInFlight().size(), 1, L"WSN registered wrong amount of packets");

      dev2.stepToIdle(); // finish tx

      Assert::IsFalse(dev1.mRxInProgress);
		}

	};
}