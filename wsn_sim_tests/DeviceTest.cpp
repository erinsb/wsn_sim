#include "stdafx.h"
#include "CppUnitTest.h"

#include "Device.h"

#include <math.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace wsn_sim_tests
{
	TEST_CLASS(DeviceTest)
	{
	public:
		
		TEST_METHOD(DistanceTest)
		{
      Device dev1;
      Device dev2(1.0, 0.0);
      Device dev3(2.0, 2.0);
#if 0
      Assert::AreEqual<double>(dev1.getDistanceTo(dev1), 0.0, L"Distance to self not 0");
      Assert::AreEqual<double>(dev1.getDistanceTo(dev2), dev2.getDistanceTo(dev1), L"Distance not commutative");
      Assert::AreEqual<double>(dev1.getDistanceTo(dev2), 1.0, L"Distance wrong");
      Assert::AreEqual<double>(dev1.getDistanceTo(dev3), sqrt(8.0), L"Diagonal distance wrong");
#endif
		}

	};
}