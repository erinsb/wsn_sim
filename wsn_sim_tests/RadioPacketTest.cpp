#include "stdafx.h"
#include "CppUnitTest.h"

#include "RadioPacket.h"
#include "Device.h"
#include "Radio.h"
#include "WSN.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace wsn_sim_tests
{

  uint8_t dummydata[] = { 0, 1, 2, 3, 4 };
  SimEnv env;
  WSN network;
  Device device;

	TEST_CLASS(RadioPacketTest)
	{
	public:

		TEST_METHOD(PacketCollisionTest)
		{
      env.attachRunnable(&network);
      network.addDevice(device);

      RadioPacket packet1(device.getRadio(), dummydata, 5);
      RadioPacket packet2(device.getRadio(), dummydata, 5);

      packet1.mStartTime = 20;
      packet1.mEndTime = 40;
      packet2.mStartTime = 30;
      packet2.mEndTime = 50;
      // s1 s2 e1 e2
      Assert::IsFalse(packet1.collidesWith(&packet1));
      Assert::IsTrue(packet1.collidesWith(&packet2));
      Assert::IsTrue(packet2.collidesWith(&packet1));
      // s1 e1 s2 e2
      packet2.mStartTime = 45;
      Assert::IsFalse(packet1.collidesWith(&packet2));
      Assert::IsFalse(packet2.collidesWith(&packet1));
      // s2 s1 e1 e2
      packet2.mStartTime = 10;
      Assert::IsTrue(packet1.collidesWith(&packet2));
      Assert::IsTrue(packet2.collidesWith(&packet1));
      // same start/end - should collide
      packet2.mStartTime = 40;
      Assert::IsTrue(packet1.collidesWith(&packet2));
      Assert::IsTrue(packet2.collidesWith(&packet1));

		}

    TEST_METHOD(CopyOperatorTest)
    {
      RadioPacket packet1(device.getRadio(), dummydata, 5);
      RadioPacket packet2 = packet1;
      Assert::AreNotSame(packet1.getContents(), packet2.getContents(), L"Deep copy");
    }
	};
}