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
#include <queue>

namespace osemu {

class InstructionEvaluator;

enum class InstructionResult {
    SUCCESS,        // Instruction executed successfully
    PAGE_FAULT,     // Instruction caused a page fault (needed page not in memory)
    PROCESS_COMPLETE // Instruction caused the process to complete (e.g., last instruction)
};

struct PageTableEntry{
    uint16_t page_id; //logical address / page_size(mem_per_frame)
    uint16_t frame_id; //set to MAX_uint16_t if invalid.
    bool valid; //if false not loaded
    bool dirty; //updated since last used

    uint8_t* data; //pointer to actual memory representation if valid
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
  PCB(std::string procName, size_t totalLines, size_t mem_per_frame);
  PCB(std::string procName, const std::vector<Expr>& instructions, size_t mem_per_frame);
  PCB(std::string procName, const std::vector<Expr>& instrs,
      size_t memory_size, size_t mem_per_frame);
  static std::atomic<uint32_t> next_pid;

  InstructionExecutionInfo step();
  bool isComplete() const;
  std::string status() const;

  bool load(std::string& storage_file);
  bool store(std::string& storage_file);
  
  
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

  std::vector<PageTableEntry> page_table; //We initialize this during memory_manager_.submit(pcb);
  std::queue<PageTableEntry> victim_pages; 
  
  std::optional<int> assignedCore;
  std::chrono::system_clock::time_point finishTime;
  
  size_t symbol_table_limit = 32; // Limit for symbol table size
  size_t symbol_table_size = 0; // Current size of the symbol table
  
  //page table
  size_t mem_per_frame = 64;
  uint16_t heap_end; // store end of heap
  
  std::vector<Expr> instructions;
  uint16_t sleepCyclesRemaining;

  //evaluator stuff
  // std::vector<uint8_t> heap_memory; //for now, this is the raw memory representation, this will be replaced with a page table.
  std::unordered_map<std::string, uint16_t> symbol_table; //here we store string(variable name):address(logical memory address, starts from the top of heap_memory) 
  std::vector<std::string> output_log;
  std::unique_ptr<InstructionEvaluator> evaluator; //evaluator takes all the top 3 members as its members too.

};

}  

#endif  
