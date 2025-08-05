#include "process_control_block.hpp"

#include <format>
#include <sstream>
#include "memory_manager.hpp"

namespace osemu {

std::atomic<uint32_t> PCB::next_pid{1};

PCB::PCB(std::string procName, const std::vector<Expr>& instrs, size_t memory_size)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs),
      memory_size_(memory_size)
{
    evaluator = std::make_unique<InstructionEvaluator>(
        this->symbol_table,
        this->output_log,
        this->processName,
        this->memory_size_
    );
}

void PCB::terminate(const std::string& reason) {
    if (!terminated_) {
        terminated_ = true;
        termination_reason_ = reason;
        finishTime = std::chrono::system_clock::now();
    }
}

bool PCB::isComplete() const {
  return currentInstruction >= totalInstructions;
}
bool PCB::step(MemoryManager& mm) {
    if (isSleeping()) {
        decrementSleepCycles();
        return true;
    }
    if (currentInstruction < instructions.size() && !isTerminated()) {
        return executeCurrentInstruction(mm);
    }
    return true;
}

std::string PCB::status() const {
    auto truncated_creation_time = std::chrono::time_point_cast<std::chrono::seconds>(creationTime);
    auto creation_time_str = std::format("{:%m/%d/%Y %I:%M:%S %p}", truncated_creation_time);
    std::ostringstream oss;
    oss << "PID:" << processID << " " << processName << " (" << creation_time_str << ")  ";
    if (isTerminated()) {
        auto truncated_finish_time = std::chrono::time_point_cast<std::chrono::seconds>(finishTime);
        auto finish_time_str = std::format("{:%H:%M:%S}", truncated_finish_time);
        oss << "Terminated (" << getTerminationReason() << " at " << finish_time_str << ") "
            << currentInstruction << " / " << totalInstructions;
    } else if (isComplete()) {
        oss << "Finished           " << totalInstructions << " / "
            << totalInstructions;
    } else if (isSleeping()) {
        oss << "Sleeping (" << sleepCyclesRemaining << ") " << currentInstruction
            << " / " << totalInstructions;
    } else if (assignedCore.has_value()) {
        oss << "Core: " << *assignedCore << "            " << currentInstruction
            << " / " << totalInstructions;
    } else {
        oss << "Ready (in queue)   " << currentInstruction << " / "
            << totalInstructions;
    }
    return oss.str();
}

// --- THIS IS THE MOST IMPORTANT FIX ---
bool PCB::executeCurrentInstruction(MemoryManager& mm) {
    if (isComplete() || isTerminated()) {
        return true; // Not a fault
    }

    const auto& instr = instructions[currentInstruction];
    const int FAULT_LIMIT = 50; // After 50 faults in a row, the process is terminated.
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

        currentInstruction++;
        return true; // Instruction completed successfully

    } catch (const PageFaultException&) {
      // A page fault occurred. The exception is caught here.
      // We do not increment the instruction pointer.
      // We signal the fault to the CpuWorker by returning false.
      return false;
    }catch (const ResourceLimitException& rle) {
      output_log.push_back("Warning: " + std::string(rle.what()) + ". Instruction ignored.");
      currentInstruction++;
      return true;
    } catch (const AccessViolationException& ave) {
        terminate(std::string("Access Violation: ") + ave.what());
        return true; // The process is "done" (terminated), not faulted.
    } catch (const std::runtime_error& re) {
        terminate(std::string("Runtime Error: ") + re.what());
        return true; // The process is "done" (terminated), not faulted.
    }
}


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

}