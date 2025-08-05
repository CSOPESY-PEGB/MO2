#ifndef OSEMU_PROCESS_CONTROL_BLOCK_H_
#define OSEMU_PROCESS_CONTROL_BLOCK_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>

#include "instruction_evaluator.hpp"

namespace osemu {

class MemoryManager;

class PCB : public std::enable_shared_from_this<PCB> {
private:
  bool terminated_ = false;
  std::string termination_reason_ = "";
  size_t memory_size_;
  int consecutive_faults_ = 0;

public:
  PCB(std::string procName, const std::vector<Expr>& instrs, size_t memory_size);

  static std::atomic<uint32_t> next_pid;

  bool step(MemoryManager& mm);
  bool executeCurrentInstruction(MemoryManager& mm);

  bool isTerminated() const { return terminated_; }
  size_t getMemorySize() const { return memory_size_; }
  void terminate(const std::string& reason);
  const std::string& getTerminationReason() const { return termination_reason_; }

  bool isComplete() const;
  std::string status() const;
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
  uint16_t sleepCyclesRemaining;

  std::vector<Expr> instructions;
  std::unordered_map<std::string, uint16_t> symbol_table;
  std::vector<std::string> output_log;
  std::unique_ptr<InstructionEvaluator> evaluator;
};

}

#endif