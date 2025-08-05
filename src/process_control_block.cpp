#include "process_control_block.hpp"

#include <format>
#include <fstream>
#include <sstream>

namespace osemu {
std::atomic<uint32_t> PCB::next_pid{1}; 

PCB::PCB(std::string procName, size_t totalLines, size_t mem_per_frame)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(totalLines),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      mem_per_frame(mem_per_frame),
      heap_end(heap_end),
      evaluator(std::make_unique<InstructionEvaluator>(
          this->page_table,
          this->symbol_table,
          this->output_log,
          this->processName,
          this->mem_per_frame
      ))
{
  page_table.reserve(65536 / mem_per_frame);
}

PCB::PCB(std::string procName, const std::vector<Expr>& instrs, size_t mem_per_frame)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs),
      mem_per_frame(mem_per_frame),
      evaluator(std::make_unique<InstructionEvaluator>(
          this->page_table,
          this->symbol_table,
          this->output_log,
          this->processName,
          this->mem_per_frame
      ))
        
{
  page_table.reserve(65536 / mem_per_frame);
} //BTW I DONT GET PARA SAN YUNG MGA HANDLE_DECLARE HERE, 

PCB::PCB(std::string procName, const std::vector<Expr>& instrs, size_t memory_size, size_t mem_per_frame)
    : processID(next_pid++),
      processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(instrs.size()),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(std::nullopt),
      sleepCyclesRemaining(0),
      instructions(instrs),
      evaluator(std::make_unique<InstructionEvaluator>(
          this->page_table,
          this->symbol_table,
          this->output_log,
          this->processName,
          this->mem_per_frame
      ))
{
  page_table.reserve(65536 / mem_per_frame);
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

InstructionExecutionInfo PCB::step() {
  if (isSleeping()) {
    decrementSleepCycles();
    return InstructionExecutionInfo();
  }

  //if instruction is not in memory
  //if(instruction is not in memory)
  //    return InstructionExecutionInfo(InstructionResult::PAGE_FAULT, page_number_to_request)
  
  if (currentInstruction < instructions.size()) {
    try{
      executeCurrentInstruction();
      ++currentInstruction;
      return InstructionExecutionInfo();
    }catch (const std::runtime_error& e) {
    std::string msg = e.what();

      if (msg.starts_with("BAD_ALLOC")) {
          std::cerr << "Memory allocation error\n";
          return InstructionExecutionInfo(InstructionResult::PROCESS_COMPLETE, 0);

      } else if (msg.starts_with("PAGE_FAULT")) {
          // Extract numeric part after "PAGE_FAULT"
          std::string address_str = msg.substr(std::string("PAGE_FAULT").length());
          try {
              uint32_t address = std::stoul(address_str);
              return InstructionExecutionInfo(InstructionResult::PAGE_FAULT, address);
          } catch (const std::exception& parse_err) {
              std::cerr << "Failed to parse address from PAGE_FAULT message: " << address_str << "\n";
          }
      }
    } catch (...) {
      std::cerr << "Unknown error occurred during instruction execution\n";
    }
    
  } else if (isComplete()){
    return InstructionExecutionInfo(InstructionResult::PROCESS_COMPLETE, 0);
  }
  return InstructionExecutionInfo();
}


bool PCB::isComplete() const { return currentInstruction >= totalInstructions; }

bool PCB::store(std::string& storage_file) {
      std::vector<std::variant<uint16_t,Expr>> mixed_value_storage(65472, uint16_t{0});

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
      return true;
  }

/*
bool PCB::load(std::string storage_file) {
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
*/

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
