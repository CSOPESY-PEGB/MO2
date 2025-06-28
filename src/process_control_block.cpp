#include "process_control_block.hpp"

#include <format>
#include <sstream>

namespace osemu {
std::atomic<uint32_t> PCB::next_pid{1}; 

PCB::PCB(std::string procName, size_t totalLines)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(totalLines),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0)
{
}

PCB::PCB(std::string procName, const std::vector<Expr>& instrs)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs)  
{
  totalInstructions = 0;
  for (const auto& instr : instrs) {
    totalInstructions++; // for the instruction itself

    // Add sleep cost if it's a SLEEP instruction
    if (instr.type == Expr::CALL && instr.var_name == "SLEEP" && instr.atom_value) {
      if (instr.atom_value->type == Atom::NUMBER) {
        totalInstructions += instr.atom_value->number_value; // count all sleep ticks
      }
    }
  }
}


void PCB::step() {
  if (isSleeping()) {
    decrementSleepCycles();
    currentInstruction++; 
    return;
  }
  
  if (currentInstruction < totalInstructions) {
    if (!instructions.empty()) {
      executeCurrentInstruction();
    }
    ++currentInstruction;
  }
}


bool PCB::isComplete() const { return currentInstruction >= totalInstructions; }



std::string PCB::status() const {
  
  auto truncated_creation_time = std::chrono::time_point_cast<std::chrono::seconds>(creationTime); 
  auto creation_time_str = std::format("{:%m/%d/%Y %I:%M:%S %p}", truncated_creation_time);

  std::ostringstream oss;
  oss << "PID:" << processID << " " << processName << " (" << creation_time_str << ")  ";

  
  if (isComplete()) {
    
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

bool PCB::executeCurrentInstruction() {
  if (currentInstruction >= instructions.size()) {
    return false;
  }
  
  try {
    const auto& instr = instructions[currentInstruction];
    
    
    if (instr.type == Expr::CALL && instr.var_name == "SLEEP" && instr.atom_value) {
      uint16_t cycles = evaluator.resolve_atom_value(*instr.atom_value);
      setSleepCycles(cycles);
      return true;
    }
    
    
    if (instr.type == Expr::CALL && instr.var_name == "PRINT") {
        if (instr.atom_value) { 
            Expr print_instr = instr;
            if (instr.atom_value->type == Atom::STRING && instr.atom_value->string_value.empty()) {
                print_instr.atom_value = std::make_unique<Atom>("Hello world from " + processName + "!", Atom::STRING);
            }
            evaluator.handle_print(*print_instr.atom_value, processName);
        } else if (instr.lhs && instr.rhs) { 
            std::string lhs_str = evaluator.print_atom_to_string(*instr.lhs);
            std::string rhs_str = evaluator.print_atom_to_string(*instr.rhs);
            Atom temp_atom(lhs_str + rhs_str, Atom::STRING);
            evaluator.handle_print(temp_atom, processName);
        } else {
            
            evaluator.evaluate(instr);
        }
        return true;
    }
    
    
    evaluator.evaluate(instr);
    return true;
  } catch (const std::exception& e) {
    
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

}  
