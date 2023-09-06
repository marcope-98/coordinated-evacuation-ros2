#ifndef RPL_TIMER_HPP_
#define RPL_TIMER_HPP_
#include <chrono>
#include <cstdint>
#include <iostream>

#ifndef RPL_TIME_FUNCTION_VERBOSE
#define RPL_TIME_FUNCTION_VERBOSE(function)           \
  rpl::Timer timer;                                   \
  timer.start();                                      \
  function;                                           \
  std::cerr << __FILE__ << " @ " << __LINE__ << "\n"; \
  std::cerr << #function << ": " << timer.stop() << " us\n";
#endif

#ifndef RPL_TIME_FUNCTION
#define RPL_TIME_FUNCTION(function) \
  rpl::Timer timer;                 \
  timer.start();                    \
  function;                         \
  std::cerr << #function << ": " << timer.stop() << " us\n";
#endif

namespace rpl
{
  struct Timer
  {
  public:
    Timer() : d_start(std::chrono::system_clock::time_point::min()) {}

    void        start() { this->d_start = std::chrono::system_clock::now(); }
    void        clear() { this->d_start = std::chrono::system_clock::time_point::min(); }
    std::size_t stop() const
    {
      if (!this->is_started()) return 0;
      std::chrono::system_clock::duration diff(std::chrono::system_clock::now() - this->d_start);
      return (std::size_t)(std::chrono::duration_cast<std::chrono::microseconds>(diff).count());
    }

  private:
    bool is_started() const { return this->d_start.time_since_epoch() != std::chrono::system_clock::duration(0); }

  private:
    std::chrono::system_clock::time_point d_start;
  };
} // namespace rpl

#endif // RPL_TIMER_HPP_
