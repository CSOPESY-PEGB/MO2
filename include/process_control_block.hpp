#ifndef OSEMU_PROCESS_CONTROL_BLOCK_H_
#define OSEMU_PROCESS_CONTROL_BLOCK_H_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "instruction_parser.hpp"
#include "instruction_evaluator.hpp"
#include <atomic>
#include <map>

namespace osemu {

class PCB : public std::enable_shared_from_this<PCB> {
 public:
  PCB(std::string procName, size_t totalLines);
  PCB(std::string procName, const std::vector<Expr>& instructions);
  PCB(std::string procName, const std::vector<Expr>& instrs,
      size_t memory_size);
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
  
  std::map<std::string, uint16_t> symbol_table;
  size_t symbol_table_limit = 32; // Limit for symbol table size
  size_t symbol_table_size = 0; // Current size of the symbol table

  // Heap memory for the process
  std::vector<uint16_t> heap_memory;  

  std::vector<Expr> instructions;
  InstructionEvaluator evaluator; //CONTAINS VARIABLES
  uint16_t sleepCyclesRemaining;
};

}  

#endif  
