#ifndef OSEMU_PROCESS_CONTROL_BLOCK_H_
#define OSEMU_PROCESS_CONTROL_BLOCK_H_

#include "instruction_evaluator.hpp"


#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <atomic>
#include <map>

namespace osemu {

class InstructionEvaluator;
class MemoryManager;

enum class InstructionResult {
    SUCCESS,        // Instruction executed successfully
    PAGE_FAULT,     // Instruction caused a page fault (needed page not in memory)
    PROCESS_COMPLETE // Instruction caused the process to complete (e.g., last instruction)
};

struct InstructionExecutionInfo{
  InstructionResult result;
  size_t faulting_virtual_address;

  // Default constructor: Initializes to SUCCESS, with no faulting address
  InstructionExecutionInfo() : result(InstructionResult::SUCCESS), faulting_virtual_address(0) {}
  // Parameterized constructor: For specifying result and an optional faulting address
  InstructionExecutionInfo(InstructionResult r, size_t addr = 0)
      : result(r), faulting_virtual_address(addr) {}
};

class PCB : public std::enable_shared_from_this<PCB> {
 public:
  PCB(std::string procName, size_t totalLines, MemoryManager* memory_manager = nullptr);
  PCB(std::string procName, const std::vector<Expr>& instructions, MemoryManager* memory_manager = nullptr);
  PCB(std::string procName, const std::vector<Expr>& instrs,
      size_t memory_size, MemoryManager* memory_manager = nullptr);
  static std::atomic<uint32_t> next_pid;

  InstructionExecutionInfo step();
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
  
  size_t symbol_table_limit = 32; // Limit for symbol table size
  size_t symbol_table_size = 0; // Current size of the symbol table

  
  std::vector<Expr> instructions;
  uint16_t sleepCyclesRemaining;

  //evaluator stuff
  std::vector<uint8_t> heap_memory; //for now, this is the raw memory representation, this will be replaced with a page table.
  std::unordered_map<std::string, uint16_t> symbol_table; //here we store string(variable name):address(logical memory address, starts from the top of heap_memory) 
  std::vector<std::string> output_log;
  std::unique_ptr<InstructionEvaluator> evaluator; //evaluator takes all the top 3 members as its members too.

};

}  

#endif  
