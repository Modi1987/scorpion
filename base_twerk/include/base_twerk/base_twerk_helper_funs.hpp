#ifndef BASE_TWERK_BASE_TWERK_HELPER_FUNS_HPP_
#define BASE_TWERK_BASE_TWERK_HELPER_FUNS_HPP_

#include "math.h"

auto get_rounded_dance_time_from_goal_millis(double w, int dance_time_millis)
    -> std::chrono::milliseconds {

  constexpr double to_millis = 1000.0;

  auto dance_period_seconds = 2 * M_PI / w;
  auto dance_time_seconds = static_cast<double>(dance_time_millis) / to_millis;

  auto num_periods = dance_time_seconds / dance_period_seconds;

  auto rounded_periods =
      std::round(num_periods) + 1; // +1 to ensure at least one period

  auto rounded_dance_time_millis =
      static_cast<long>(rounded_periods * dance_period_seconds * to_millis);

  return std::chrono::milliseconds(rounded_dance_time_millis);
}

#endif // BASE_TWERK_BASE_TWERK_HELPER_FUNS_HPP_