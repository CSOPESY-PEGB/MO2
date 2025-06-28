#ifndef OSEMU_PROCESS_CONTROL_BLOCK_H_
#define OSEMU_PROCESS_CONTROL_BLOCK_H_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "instruction_parser.hpp"
#include <atomic>

namespace osemu {

class PCB : public std::enable_shared_from_this<PCB> {
 public:
  PCB(std::string procName, size_t totalLines);
  PCB(std::string procName, const std::vector<Expr>& instructions);
  static std::atomic<uint32_t> next_pid;

  void step();
  bool isComplete() const;
  std::string status() const;
  
  
  bool executeCurrentInstruction();
  const std::vector<std::string>& getExecutionLogs() const;
  
  
  void setSleepCycles(uint16_t cycles);
  bool isSleeping() const;
  void decrementSleepCycles();

  uint32_t processID;
  std::string processName;
  size_t currentInstruction;
  size_t totalInstructions;
  std::chrono::system_clock::time_point creationTime;

  
  std::optional<int> assignedCore;
  std::chrono::system_clock::time_point finishTime;
  
  
  std::vector<Expr> instructions;
  InstructionEvaluator evaluator;
  uint16_t sleepCyclesRemaining;
};

}  

#endif  
