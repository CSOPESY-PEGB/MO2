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
      sleepCyclesRemaining(0),
      evaluator(std::make_unique<InstructionEvaluator>(
          this->heap_memory,
          this->symbol_table,
          this->output_log,
          this->processName 
      ))

{
  evaluator->handle_declare("x", Atom(static_cast<uint16_t>(0)));
}

PCB::PCB(std::string procName, const std::vector<Expr>& instrs)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs),
      evaluator(std::make_unique<InstructionEvaluator>(
          this->heap_memory,
          this->symbol_table,
          this->output_log,
          this->processName 
      ))
        
{
  evaluator->handle_declare("x", Atom(static_cast<uint16_t>(0)));
} //BTW I DONT GET PARA SAN YUNG MGA HANDLE_DECLARE HERE, 

PCB::PCB(std::string procName, const std::vector<Expr>& instrs, size_t memory_size)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs),
      heap_memory(memory_size - 64, 0),
      evaluator(std::make_unique<InstructionEvaluator>(
          this->heap_memory,
          this->symbol_table,
          this->output_log,
          this->processName 
      ))
{
  evaluator->handle_declare("x", Atom(static_cast<uint16_t>(0)));
}


void PCB::step() {
  if (isSleeping()) {
    decrementSleepCycles();
    return;
  }
  
  if (currentInstruction < instructions.size()) {
    executeCurrentInstruction();
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
      uint16_t cycles = evaluator->resolve_atom_value(*instr.atom_value);
      setSleepCycles(cycles);
      return true;
    }
    
    //WHY ARE WE IMPLEMENTING THESE FUNCTIONS HERE HERE?
    //if (instr.type == Expr::READ){
    //  if (symbol_table_size >= symbol_table_limit) {
    //    throw std::runtime_error("Symbol table limit reached"); 
    //    return true;
    //  }
    //  else if (instr.atom_value->number_value >= heap_memory.size()) {
    //    throw std::runtime_error("Heap address out of bounds");
    //    return true;
    //    // violation error and then shut down the process
    //  } else {
    //    // check if logic is good
    //    symbol_table[instr.var_name] = heap_memory[instr.atom_value->number_value];
    //    Atom temp_atom("READ operation: " + instr.var_name + " = " + std::to_string(symbol_table[instr.var_name]), Atom::STRING);
    //    symbol_table_size++;
    //    evaluator->handle_print(temp_atom, processName);
    //    return true;
    //  }
    //}

    //if (instr.type == Expr::WRITE) {
    //  // check if heap address is valid
    //  if (instr.lhs->number_value >= heap_memory.size()) {
    //    throw std::runtime_error("Heap address out of bounds");
    //    return true;
    //    // violation error and then shut down the process
    //  }
    //  // check if logic is good
    //  heap_memory[instr.lhs->number_value] = instr.rhs->number_value;
    //  Atom temp_atom("WRITE operation: " + std::to_string(instr.lhs->number_value) + " = " + std::to_string(instr.rhs->number_value), Atom::STRING);
    //  evaluator->handle_print(temp_atom, processName);
    //  return true;
    //}    
    
    
    if (instr.type == Expr::CALL && instr.var_name == "PRINT") {
        if (instr.atom_value) { 
            Expr print_instr = instr;
            if (instr.atom_value->type == Atom::STRING && instr.atom_value->string_value.empty()) {
                print_instr.atom_value = std::make_unique<Atom>("Hello world from " + processName + "!", Atom::STRING);
            }
            evaluator->handle_print(*print_instr.atom_value, processName);
        } else if (instr.lhs && instr.rhs) { 
            std::string lhs_str = evaluator->print_atom_to_string(*instr.lhs);
            std::string rhs_str = evaluator->print_atom_to_string(*instr.rhs);
            Atom temp_atom(lhs_str + rhs_str, Atom::STRING);
            evaluator->handle_print(temp_atom, processName);
        } else {
            
            evaluator->evaluate(instr);
        }
        return true;
    }
    
    
    evaluator->evaluate(instr);
    return true;
  } catch (const std::exception& e) {
    return false;
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
