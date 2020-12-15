#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace ceres_loam {
class Timer {
 public:
  Timer() { tic(); }

  void tic() { start_ = std::chrono::system_clock::now(); }

  double end(bool restart = false) {
    end_ = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_ - start_;
    if (restart) {
      start_ = std::chrono::system_clock::now();
    }
    return elapsed_seconds.count() * 1000;  // ms
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
};

}  // namespace ceres_loam
