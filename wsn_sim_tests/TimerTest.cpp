#include "stdafx.h"
#include "CppUnitTest.h"

#include "Timer.h"
#include "SimEnv.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace wsn_sim_tests
{
  class DummyClient : public TimerClient
  {
    virtual void timerEnded(Timeout* to)
    {
      //skip
    }
  };

	TEST_CLASS(TimerTest)
	{
	public:
		
		TEST_METHOD(TestDrift)
		{
      SimEnv env;
      DummyClient client;
      const uint32_t timestamp = 20;

      Timer correct_timer(1.0, client);
      Timer fast_timer(2.0, client);

      env.attachRunnable(correct_timer);
      env.attachRunnable(fast_timer);
      env.step(timestamp);
      
      Assert::AreEqual(correct_timer.getTimestamp(), timestamp);
      Assert::AreEqual(fast_timer.getTimestamp(), timestamp * 2);
		}

	};
}