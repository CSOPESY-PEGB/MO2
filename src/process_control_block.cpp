#include "process_control_block.hpp"

#include <format>
#include <sstream>

namespace osemu {
std::atomic<uint32_t> PCB::next_pid{1}; 
const std::string PCB::storage_file = "storage.txt";
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
      heap_memory(memory_size, 0), // no need to subtract 64 since we're already taking that into account!
      evaluator(std::make_unique<InstructionEvaluator>(
          this->heap_memory,
          this->symbol_table,
          this->output_log,
          this->processName
      )),
      mixed_value_storage(65472, uint16_t{0}) // Initialize mixed_value_storage with 65472 elements of type uint16_t
{ 
  // initialize the mixed_value_storage
  for (const auto& expr : instrs) {
    mixed_value_storage.emplace_back(expr);
  }
  evaluator->handle_declare("x", Atom(static_cast<uint16_t>(0)));
}

PCB::PCB(std::string procName, const std::vector<Expr>& instrs, size_t memory_size, size_t mem_per_frame)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      mem_per_frame(mem_per_frame),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs),
      heap_memory(memory_size, 0), // no need to subtract 64 since we're already taking that into account!
      evaluator(std::make_unique<InstructionEvaluator>(
          this->heap_memory,
          this->symbol_table,
          this->output_log,
          this->processName
      )),
      mixed_value_storage(65472, uint16_t{0}) // Initialize mixed_value_storage with 65472 elements of type uint16_t
{ 
  // initialize the mixed_value_storage
  for (const auto& expr : instrs) {
    mixed_value_storage.emplace_back(expr);
  }
  this->total_frames_allocated = memory_size / mem_per_frame;
  // round up to the nearest frame size
  if (memory_size % mem_per_frame != 0) {
    this->total_frames_allocated++;
  }
  // set page_table size
  this->page_table.resize(this->total_frames_allocated, 0);
  evaluator->handle_declare("x", Atom(static_cast<uint16_t>(0)));
}


//
// PCB::PCB(std::string procName, const std::vector<Expr>& instrs, size_t memory_size)
//     : processID(next_pid++),
//       processName(std::move(procName)),
//       currentInstruction(0),
//       totalInstructions(instrs.size()),
//       creationTime(std::chrono::system_clock::now()),
//       assignedCore(std::nullopt),
//       sleepCyclesRemaining(0),
//       instructions(instrs),
//       heap_memory(memory_size - (instrs.size() - 64), 0)
// {
//   evaluator->handle_declare("x", Atom(static_cast<uint16_t>(0)));
// }


void PCB::store() {
    std::ofstream out(storage_file, std::ios::binary | std::ios::app);
    if (!out) {
        throw std::runtime_error("Cannot open storage file for writing.");
    }

    // Write PCB ID
    out.write(reinterpret_cast<const char*>(&processID), sizeof(processID));

    // Write program size
    size_t program_size = mixed_value_storage.size();
    out.write(reinterpret_cast<const char*>(&program_size), sizeof(program_size));

    // Write program instructions
    for (const auto& instruction : mixed_value_storage) {
        uint8_t type = static_cast<uint8_t>(instruction.index());
        out.write(reinterpret_cast<const char*>(&type), sizeof(type));

        if (std::holds_alternative<uint16_t>(instruction)) {
            uint16_t val = std::get<uint16_t>(instruction);
            out.write(reinterpret_cast<const char*>(&val), sizeof(val));
        } else if (std::holds_alternative<Expr>(instruction)) {
            const Expr& expr = std::get<Expr>(instruction);
            expr.write(out);
        }
    }
}

bool PCB::load() {
    std::ifstream in(storage_file, std::ios::binary);
    if (!in) {
        return false; // File doesn't exist or cannot be opened.
    }

    while (in.peek() != EOF) {
        uint32_t current_id;
        in.read(reinterpret_cast<char*>(&current_id), sizeof(current_id));

        if (in.gcount() == 0) break; // End of file

        size_t program_size;
        in.read(reinterpret_cast<char*>(&program_size), sizeof(program_size));

        if (current_id == this->processID) {
            mixed_value_storage.clear();
            for (size_t i = 0; i < program_size; ++i) {
                uint8_t type;
                in.read(reinterpret_cast<char*>(&type), sizeof(type));

                if (type == 0) { // uint16_t
                    uint16_t val;
                    in.read(reinterpret_cast<char*>(&val), sizeof(val));
                    mixed_value_storage.emplace_back(val);
                } else if (type == 1) { // Expr
                    Expr expr;
                    expr.read(in);
                    mixed_value_storage.emplace_back(std::move(expr));
                }
            }
            return true; // Found and loaded
        } else {
            // Skip this PCB's data
            for (size_t i = 0; i < program_size; ++i) {
                uint8_t type;
                in.read(reinterpret_cast<char*>(&type), sizeof(type));
                if (type == 0) {
                    in.seekg(sizeof(uint16_t), std::ios::cur);
                } else if (type == 1) {
                    // This is tricky, we need to deserialize to skip correctly.
                    // A better format would have stored the size of the block.
                    // For now, we deserialize to a dummy object.
                    Expr dummy;
                    dummy.read(in);
                }
            }
        }
    }
    return false; // Not found
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

bool PCB::isThisAddressInMemory(size_t address){
  size_t page = address / mem_per_frame;
  // round up to the nearest frame size
  if (address % mem_per_frame != 0) {
    page++;
  }
  if (std::find(page_table.begin(), page_table.end(), page) != page_table.end()) {
        std::cout << "Value found!\n";
        return true;
    } else {
        std::cout << "Value not found.\n";
        return false;
  }

}


bool PCB::executeCurrentInstruction() {
  if (currentInstruction >= instructions.size()) {
    return false;
  }

  if (isThisAddressInMemory(currentInstruction)){
    std::cout << "Address " << currentInstruction << " is in memory.\n";
  } else {
    std::cout << "Address " << currentInstruction << " is not in memory.\n";
  }
  
  try {
    const auto& instr = std::get<Expr>(mixed_value_storage[currentInstruction]);
    if (instr.type == Expr::CALL && instr.var_name == "SLEEP" && instr.atom_value) {
      uint16_t cycles = evaluator->resolve_atom_value(*instr.atom_value);
      setSleepCycles(cycles);
      return true;
    }
    

    if (instr.type == Expr::READ){
      if (symbol_table_size >= symbol_table_limit) {
        throw std::runtime_error("Symbol table limit reached");
        return true;
      }
      else if (instr.atom_value->number_value >= heap_memory.size()) {
        throw std::runtime_error("Heap address out of bounds");
        return true;
        // violation error and then shut down the process
      } else {
        // check if logic is good
        symbol_table[instr.var_name] = heap_memory[instr.atom_value->number_value];
        Atom temp_atom("READ operation: " + instr.var_name + " = " + std::to_string(symbol_table[instr.var_name]), Atom::STRING);
        symbol_table_size++;
        evaluator->handle_print(temp_atom, processName);
        return true;
      }
    }

    if (instr.type == Expr::WRITE) {
      // check if heap address is valid
      if (instr.lhs->number_value >= heap_memory.size()) {
        throw std::runtime_error("Heap address out of bounds");
        return true;
        // violation error and then shut down the process
      }
      // check if logic is good
      heap_memory[instr.lhs->number_value] = instr.rhs->number_value;
      Atom temp_atom("WRITE operation: " + std::to_string(instr.lhs->number_value) + " = " + std::to_string(instr.rhs->number_value), Atom::STRING);
      evaluator->handle_print(temp_atom, processName);
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
