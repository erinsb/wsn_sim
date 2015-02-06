#pragma once
#include <mutex>
#include <condition_variable>
#include <string>

class Barrier
{
private:
	std::mutex _mutex;
	std::condition_variable _cv;
	std::size_t _count;
  std::mutex _writeMutex;
public:
	explicit Barrier(uint32_t count) : _count{ count } { }
  Barrier() : Barrier(1){}
  void reset(uint32_t count) 
  { 
    _writeMutex.lock();
    _count = count; 
    _writeMutex.unlock();
  }

	void wait()
	{
		std::unique_lock<std::mutex> lock{ _mutex };
    _writeMutex.lock();
		if (--_count == 0) {
      _writeMutex.unlock();
			_cv.notify_all();
		}
    else {
      _writeMutex.unlock();
			_cv.wait(lock);
		}
	}

  void post()
  {
    _writeMutex.lock();
    if (--_count == 0)
    {
      _cv.notify_all();
    }
    _writeMutex.unlock();
  }

  std::string ToString(void) { return "Count = " + std::to_string(_count); }
};

