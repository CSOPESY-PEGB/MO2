#include "process_control_block.hpp"

#include <format>
#include <sstream>

namespace osemu {

PCB::PCB(std::string procName, size_t totalLines)
    : processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(totalLines),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0)
{
}

PCB::PCB(std::string procName, const std::vector<Expr>& instrs)
    : processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs)  // Use copy constructor
{
}

// Definition for step()
void PCB::step() {
  if (isSleeping()) {
    decrementSleepCycles();
    return;
  }
  
  if (currentInstruction < totalInstructions) {
    if (!instructions.empty()) {
      executeCurrentInstruction();
    }
    ++currentInstruction;
  }
}

// Definition for isComplete()
bool PCB::isComplete() const { return currentInstruction >= totalInstructions; }

// Definition for status() - This is the key one you need to implement as per
// the new requirements
std::string PCB::status() const {
  // Get the creation time as a formatted string
  auto creation_time_str = std::format("{:%m/%d/%Y %I:%M:%S%p}", creationTime);

  std::ostringstream oss;
  oss << processName << " (" << creation_time_str << ")  ";

  // Logic to display different status based on the process state
  if (isComplete()) {
    // If the process is done, show "Finished" and the final instruction count
    oss << "Finished           " << totalInstructions << " / "
        << totalInstructions;

  } else if (assignedCore.has_value()) {
    // If it's running on a core, show the core number and progress
    oss << "Core: " << *assignedCore << "            " << currentInstruction
        << " / " << totalInstructions;
  } else {
    // If it's not complete and not on a core, it's in the "Ready" state
    // You can customize this, but for now, we'll just show progress.
    // The UI screenshot doesn't show a "Ready" state, only "Running" or
    // "Finished". This part of the status string is for the "Running processes"
    // list.
    oss << "Ready (in queue)   " << currentInstruction << " / "
        << totalInstructions;
  }

  return oss.str();
}

bool PCB::executeCurrentInstruction() {
  if (currentInstruction >= instructions.size()) {
    return false;
  }
  
  try {
    const auto& instr = instructions[currentInstruction];
    
    // Handle SLEEP instruction specially
    if (instr.type == Expr::CALL && instr.var_name == "SLEEP" && instr.atom_value) {
      uint16_t cycles = 0;
      if (instr.atom_value->type == Atom::NUMBER) {
        cycles = instr.atom_value->number_value;
      } else if (instr.atom_value->type == Atom::NAME) {
        // Resolve variable - for now assume 0 if undefined
        cycles = 0;
      }
      setSleepCycles(cycles);
      return true;
    }
    
    // Handle PRINT instruction with process name
    if (instr.type == Expr::CALL && instr.var_name == "PRINT" && instr.atom_value) {
      // Create a modified instruction with default message if needed
      Expr print_instr = instr;
      if (instr.atom_value->type == Atom::STRING && instr.atom_value->string_value.empty()) {
        print_instr.atom_value = std::make_unique<Atom>("Hello world from " + processName + "!", Atom::STRING);
      }
      evaluator.handle_print(*print_instr.atom_value, processName);
      return true;
    }
    
    // Execute other instructions normally
    evaluator.evaluate(instr);
    return true;
  } catch (const std::exception& e) {
    // Log error or handle gracefully
    return false;
  }
}

const std::vector<std::string>& PCB::getExecutionLogs() const {
  return evaluator.get_output_log();
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

}  // namespace osemu
