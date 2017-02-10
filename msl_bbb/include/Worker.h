#pragma once

#define TIMEDIFFMS(n, o) (((n).tv_sec - (o).tv_sec) * 1000 + ((n).tv_usec - (o).tv_usec) / 1000)

#include <string>
#include <condition_variable>
#include <chrono>
#include <thread>

#define WORKER_DEBUG

namespace std
{
class thread;
}

namespace supplementary
{
class Timer;
}

namespace msl_bbb
{

class Worker
{
  public:
    Worker(std::string name);
    virtual ~Worker();
    virtual void run() = 0; /** < Meant to be overwritten by derived classes. */
    bool stop();
	bool start();

	std::string name; /** < The name of this worker. */
    std::chrono::milliseconds msInterval;
    std::chrono::milliseconds msDelayedStart;

  protected:
    std::condition_variable runCV;

    bool started; /** < Is always true except when the worker is shutting down. */
    bool running; /** < Tells us whether the worker is currently running (or active). */

    std::thread *runThread;      /** < Executes the runInternal and thereby the abstract run method. */
    supplementary::Timer *timer; /** < Triggers the condition_variable of the runThread. */

  private:
    void runInternal();

    std::mutex runCV_mtx;
};

} /* namespace msl_bbb */
