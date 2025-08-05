#ifndef OSEMU_PROCESS_CONTROL_BLOCK_H_
#define OSEMU_PROCESS_CONTROL_BLOCK_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>

#include "instruction_evaluator.hpp" // Needs full definition to hold a unique_ptr

namespace osemu {

// Forward declare to avoid circular includes
class MemoryManager;

class PCB : public std::enable_shared_from_this<PCB> {
private:
    // --- NEW --- Member variables for state tracking
    bool terminated_ = false;
    std::string termination_reason_ = "";
    size_t memory_size_;

public:
    // --- UPDATED --- Constructor
    PCB(std::string procName, const std::vector<Expr>& instrs, size_t memory_size);

    static std::atomic<uint32_t> next_pid;

    // --- UPDATED --- Core execution functions
    void step(MemoryManager& mm);
    bool executeCurrentInstruction(MemoryManager& mm);

    // --- NEW --- Methods for state management
    bool isTerminated() const { return terminated_; }
    size_t getMemorySize() const { return memory_size_; }
    void terminate(const std::string& reason);
    const std::string& getTerminationReason() const { return termination_reason_; }

    // --- EXISTING --- Public methods (no changes needed)
    bool isComplete() const;
    std::string status() const;
    const std::vector<std::string>& getExecutionLogs() const;
    void setSleepCycles(uint16_t cycles);
    bool isSleeping() const;
    void decrementSleepCycles();

    // --- EXISTING --- Public member variables
    uint32_t processID;
    std::string processName;
    size_t currentInstruction;
    size_t totalInstructions;
    std::chrono::system_clock::time_point creationTime;
    std::optional<int> assignedCore;
    std::chrono::system_clock::time_point finishTime;

    std::vector<Expr> instructions;
    uint16_t sleepCyclesRemaining;

    // --- EXISTING --- Members passed to the evaluator
    // Note: heap_memory is now obsolete but kept to avoid breaking the old evaluator constructor.
    // The new system uses the MemoryManager instead.
    std::vector<uint8_t> heap_memory;
    std::unordered_map<std::string, uint16_t> symbol_table;
    std::vector<std::string> output_log;
    std::unique_ptr<InstructionEvaluator> evaluator;
};

} // namespace osemu

#endif