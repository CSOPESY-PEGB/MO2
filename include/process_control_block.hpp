// process_control_block.hpp
#pragma once

#include <chrono>
#include <string>
#include <functional>   // for std::hash

struct PCB {
  PCB(std::string procName, size_t totalLines);

  void step();
  bool isComplete() const;
  std::string status() const;

  std::string processName;
  size_t      currentInstruction{0};
  size_t      totalInstructions;
  std::chrono::system_clock::time_point creationTime;
};

inline bool operator==(PCB const& a, PCB const& b) noexcept {
  return a.processName == b.processName;
}

namespace std {
template<>
struct hash<PCB> {
  size_t operator()(PCB const& pcb) const noexcept {
    return std::hash<std::string>()(pcb.processName);
  }
};
}
