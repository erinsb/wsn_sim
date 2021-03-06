#include "stdafx.h"
#include "CppUnitTest.h"
#include "Radio.h"
#include "WSN.h"
#include "Device.h"
#include "SimEnv.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace wsn_sim_tests
{
	TEST_CLASS(RadioTest)
	{
	public:

    WSN mWSN;
    SimEnv mSimEnv;
    Device mDevice;
    Radio* pRadio;
    uint8_t dummyData[5];
    TEST_METHOD_INITIALIZE(RadioTestInit)
    {
      if (mSimEnv.numberOfRunnables() == 0)
      {
        mSimEnv.attachRunnable(&mWSN);

        mWSN.addDevice(&mDevice);
        pRadio = mDevice.getRadio();
        for (uint8_t i = 0; i < 5; ++i)
          dummyData[i] = i;
      }
      pRadio->disable();
      pRadio->shortDisable();
    }

    TEST_METHOD(InitializerTest)
    {
      Assert::IsFalse(pRadio == (Radio*)NULL, L"Has no radio");
    }

    TEST_METHOD(TxTimeTest)
    {
      Assert::IsTrue(pRadio->getTxTime(100) == 8 * 100, L"TX Time is not right");
    }

    TEST_METHOD(SMTestTx)
    {
      pRadio->setPacket(dummyData, 5);
      pRadio->transmit();
      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_RAMPUP_TX, L"Radio failed entering rampup");
      
      mSimEnv.run(1);
      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_RAMPUP_TX, L"Radio failed to stay in RU");
      
      mSimEnv.run(RADIO_DEFAULT_TURNAROUND - 2);
      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_RAMPUP_TX, L"Radio failed to stay in RU for the entire RU");

      mSimEnv.run(mSimEnv.getTimestamp() + 4);
      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_TX, L"Radio failed to enter TX");
      
      mSimEnv.run(mSimEnv.getTimestamp() + pRadio->getTxTime(5));
      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_IDLE, L"Radio failed to go back to idle");

      mSimEnv.run(mSimEnv.getTimestamp() + 1000);
      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_IDLE, L"Radio failed stay in idle");
		}

    TEST_METHOD(SMTestShort)
    {
      pRadio->setPacket(dummyData, 5);
      pRadio->shortToTx();
      pRadio->transmit();

      mSimEnv.run(RADIO_DEFAULT_TURNAROUND + 1);
      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_TX, L"Failed to transmit first");

      mSimEnv.run(mSimEnv.getTimestamp() + pRadio->getTxTime(5) + 1);
      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_RAMPUP_TX, L"Failed to do short");
      
      pRadio->shortDisable();

      runWhileRadioInState(Radio::RADIO_STATE_RAMPUP_TX);

      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_TX, L"Failed to enter TX");

      runWhileRadioInState(Radio::RADIO_STATE_TX);

      Assert::AreEqual<int>(pRadio->getState(), Radio::RADIO_STATE_IDLE, L"Failed to return to idle");
    }

    TEST_METHOD(SMTestExitTXRUAttempt)
    {
      pRadio->setPacket(dummyData, 5);
      pRadio->shortToRx();
      pRadio->transmit();
      
      runUntilRadioInState(Radio::RADIO_STATE_RAMPUP_RX);

      pRadio->shortToTx();

      runUntilRadioInState(Radio::RADIO_STATE_RAMPUP_RX);

      pRadio->disable();
      pRadio->shortDisable();

      runUntilRadioInState(Radio::RADIO_STATE_IDLE);

      pRadio->transmit();
      runUntilRadioInState(Radio::RADIO_STATE_IDLE);
      pRadio->receive();
      runUntilRadioInState(Radio::RADIO_STATE_RX);
      pRadio->disable();
    }

    TEST_METHOD(NotifyingWSNTest)
    {
      pRadio->setPacket(dummyData, 5);
      pRadio->transmit();

      runWhileRadioInState(Radio::RADIO_STATE_RAMPUP_TX);
      
      auto packetsOnAir = mWSN.getPacketsInFlight();
      Assert::AreEqual<int>(packetsOnAir.size(), 1, L"Failed to post packet to WSN");

      runWhileRadioInState(Radio::RADIO_STATE_TX);

      packetsOnAir = mWSN.getPacketsInFlight();
      Assert::AreEqual<int>(packetsOnAir.size(), 0, L"Failed to notify end of transmit to WSN");
      
    }
  private:
    void runWhileRadioInState(Radio::state_t state)
    {
      uint32_t i = 10000;
      while (pRadio->getState() == state && --i)
      {
        mSimEnv.run(mSimEnv.getTimestamp() + 1);
      }
      std::string str = "Radio failed to leave state " + std::to_string(state);
      Assert::IsFalse(i == 0, std::wstring(str.begin(), str.end()).c_str());
    }

    void runUntilRadioInState(Radio::state_t state)
    {
      uint32_t i = 10000; 
      while (pRadio->getState() != state && --i)
      {
        mSimEnv.run(mSimEnv.getTimestamp() + 1);
      }
      std::string str = "Radio failed to enter state " + std::to_string(state);
      Assert::IsFalse(i == 0, std::wstring(str.begin(), str.end()).c_str());
    }

	};
}