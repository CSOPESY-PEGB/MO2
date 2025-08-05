#include "process_control_block.hpp"

#include <format>
#include <sstream>

// Needs the full definition to call methods
#include "memory_manager.hpp"

namespace osemu {

std::atomic<uint32_t> PCB::next_pid{1};

// --- UPDATED CONSTRUCTOR ---
PCB::PCB(std::string procName, const std::vector<Expr>& instrs, size_t memory_size)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs),
      memory_size_(memory_size), // Initialize the new member variable
      heap_memory(memory_size, 0) // Kept for compatibility, but not used by paging
{
    // Initialize the evaluator with the correct constructor
    evaluator = std::make_unique<InstructionEvaluator>(
        this->symbol_table,
        this->output_log,
        this->processName,
        this->memory_size_ // Pass the size by reference
    );
}

// --- NEW METHOD IMPLEMENTATION ---
void PCB::terminate(const std::string& reason) {
    if (!terminated_) {
        terminated_ = true;
        termination_reason_ = reason;
        finishTime = std::chrono::system_clock::now();
    }
}

// --- UPDATED ---
void PCB::step(MemoryManager& mm) {
    if (isSleeping()) {
        decrementSleepCycles();
        return;
    }
    if (currentInstruction < instructions.size() && !isTerminated()) {
        // executeCurrentInstruction is now responsible for incrementing the instruction pointer
        executeCurrentInstruction(mm);
    }
}

// --- UPDATED ---
bool PCB::isComplete() const {
    return currentInstruction >= totalInstructions;
}

// --- UPDATED ---
std::string PCB::status() const {
    auto truncated_creation_time = std::chrono::time_point_cast<std::chrono::seconds>(creationTime);
    auto creation_time_str = std::format("{:%m/%d/%Y %I:%M:%S %p}", truncated_creation_time);

    std::ostringstream oss;
    oss << "PID:" << processID << " " << processName << " (" << creation_time_str << ")  ";

    if (isTerminated()) {
        oss << "Terminated         " << currentInstruction << " / "
            << totalInstructions << " (" << getTerminationReason() << ")";
    } else if (isComplete()) {
        oss << "Finished           " << totalInstructions << " / "
            << totalInstructions;
    } else if (assignedCore.has_value()) {
        oss << "Core: " << *assignedCore << "            " << currentInstruction
            << " / " << totalInstructions;
    } else {
        oss << "Ready (in queue)   " << currentInstruction << " / "
            << totalInstructions;
    }
    return oss.str();
}

// --- UPDATED --- The core logic for instruction execution with error handling
bool PCB::executeCurrentInstruction(MemoryManager& mm) {
    if (isComplete() || isTerminated()) {
        return false;
    }

    const auto& instr = instructions[currentInstruction];

    try {
        if (instr.type == Expr::DECLARE) {
            uint16_t value = evaluator->resolve_atom_value(*instr.atom_value, mm, this->processID);
            uint16_t var_addr = evaluator->get_or_create_variable_address(instr.var_name);
            mm.write_u16(processID, var_addr, value);
        }
        else if (instr.type == Expr::READ) {
            uint16_t source_addr = evaluator->resolve_atom_value(*instr.atom_value, mm, this->processID);
            uint16_t value = mm.read_u16(processID, source_addr);
            uint16_t dest_addr = evaluator->get_or_create_variable_address(instr.var_name);
            mm.write_u16(processID, dest_addr, value);
        }
        else if (instr.type == Expr::WRITE) {
            uint16_t value = evaluator->resolve_atom_value(*instr.rhs, mm, this->processID);
            uint16_t dest_addr = evaluator->resolve_atom_value(*instr.lhs, mm, this->processID);
            mm.write_u16(processID, dest_addr, value);
        }
        else if (instr.type == Expr::CALL && instr.var_name == "SLEEP") {
            uint16_t cycles = evaluator->resolve_atom_value(*instr.atom_value, mm, this->processID);
            setSleepCycles(cycles);
        }
        else {
            evaluator->evaluate(instr, mm, this->processID);
        }

        // --- SUCCESS ---
        // If no exception was thrown, the instruction completed successfully.
        currentInstruction++;
        return true;

    } catch (const PageFaultException& pfe) {
      // NORMAL, recoverable page fault. The page has been loaded.
      // Do nothing. The instruction pointer is NOT incremented.
      // The process will re-try this same instruction on the next tick.
      return false;
    }catch (const ResourceLimitException& rle) {
      // --- THE NEW LOGIC TO MATCH THE SPEC ---
      // A non-fatal resource limit was hit (e.g., symbol table full).
      // The instruction should be "ignored".

      // We log the event for debugging, which is good practice.
      output_log.push_back("Warning: " + std::string(rle.what()) + " Instruction ignored.");

      // We INCREMENT the instruction pointer to move on to the next one.
      currentInstruction++;

      // We return 'true' because the step is "complete" from the scheduler's PoV.
      // The process is healthy and should continue running.
      return true;

    } catch (const AccessViolationException& ave) {
        // FATAL, unrecoverable memory error. Terminate the process.
        terminate(std::string("Access Violation: ") + ave.what());
        return false;

    } catch (const std::runtime_error& re) {
        // Any other fatal error during execution.
        terminate(std::string("Runtime Error: ") + re.what());
        return false;
    }
}

// --- NO CHANGES NEEDED for the functions below ---

const std::vector<std::string>& PCB::getExecutionLogs() const {
    return evaluator->get_output_log();
}

void PCB::setSleepCycles(uint16_t cycles) {
    sleepCyclesRemaining = cycles;
}

bool PCB::isSleeping() const {
    return sleepCyclesRemaining > 0;
}

void PCB::decrementSleepCycles() {
    if (sleepCyclesRemaining > 0) {
        --sleepCyclesRemaining;
    }
}

} // namespace osemu