#include "stdafx.h"
#include "CppUnitTest.h"

#include "Timer.h"
#include "SimEnv.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace wsn_sim_tests
{

	TEST_CLASS(TimerTest)
	{
	public:
		
		TEST_METHOD(TestDrift)
		{
      SimEnv env;
      const uint32_t timestamp = 20;

      Timer correct_timer(1.0);
      Timer fast_timer(2.0);

      env.attachRunnable(&correct_timer);
      env.attachRunnable(&fast_timer);
      env.run(timestamp);
      
      Assert::AreEqual(correct_timer.getTimestamp(), timestamp, L"no-drift timer");
      Assert::AreEqual(fast_timer.getTimestamp(), timestamp * 2, L"Fast timer");
		}

	};
}