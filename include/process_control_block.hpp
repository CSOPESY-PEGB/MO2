#ifndef OSEMU_PROCESS_CONTROL_BLOCK_H_
#define OSEMU_PROCESS_CONTROL_BLOCK_H_

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace osemu {

class PCB : public std::enable_shared_from_this<PCB> {
 public:
  PCB(std::string procName, size_t totalLines);

  void step();
  bool isComplete() const;
  std::string status() const;

  std::string processName;
  size_t currentInstruction;
  size_t totalInstructions;
  std::chrono::system_clock::time_point creationTime;

  // NEW: State tracking
  std::optional<int> assignedCore;
  std::chrono::system_clock::time_point finishTime;
};

}  // namespace osemu

#endif  // OSEMU_PROCESS_CONTROL_BLOCK_H_
