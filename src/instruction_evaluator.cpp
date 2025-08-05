#include "instruction_evaluator.hpp"

#include "memory_manager.hpp" // Needs full definition to call methods
#include <format>
#include <chrono>

namespace osemu {

InstructionEvaluator::InstructionEvaluator(
    std::unordered_map<std::string, uint16_t>& symbol_table,
    std::vector<std::string>& output_log,
    std::string& process_name,
    size_t& memory_size)
    : symbol_table_(symbol_table),
      output_log_(output_log),
      process_name_(process_name),
      memory_size_ref_(memory_size) {}

uint16_t InstructionEvaluator::get_or_create_variable_address(const std::string& var_name) {
    if (symbol_table_.count(var_name)) {
        return symbol_table_.at(var_name);
    }

    if (symbol_table_.size() >= 32) {
        throw ResourceLimitException("Symbol table limit reached (32 variables max).");
    }

    // Stack grows downwards from the top of virtual memory. Each var is 2 bytes.
    size_t new_var_offset = (symbol_table_.size() + 1) * 2;
    if (memory_size_ref_ < 64 || new_var_offset > 64) {
         throw std::runtime_error("Stack overflow: too many variables for 64-byte stack.");
    }

    uint16_t new_address = memory_size_ref_ - new_var_offset;
    symbol_table_[var_name] = new_address;
    return new_address;
}

uint16_t InstructionEvaluator::resolve_atom_value(const Atom& atom, MemoryManager& mm, uint32_t pcb_id) {
    switch (atom.type) {
        case Atom::NAME:
            if (symbol_table_.count(atom.string_value)) {
                uint16_t var_virtual_addr = symbol_table_.at(atom.string_value);
                // DELEGATE memory reading to the MemoryManager
                return mm.read_u16(pcb_id, var_virtual_addr);
            }
            return 0; // Per spec, uninitialized variable is 0.

        case Atom::NUMBER:
            return atom.number_value;

        case Atom::STRING:
            // Handle hex strings like "0x500" as numbers
            if (atom.string_value.starts_with("0x")) {
                try {
                    return static_cast<uint16_t>(std::stoul(atom.string_value, nullptr, 16));
                } catch(...) {
                    throw std::runtime_error("Invalid hex address string: " + atom.string_value);
                }
            }
            throw std::runtime_error("Cannot resolve string to a numeric value.");

        default:
            throw std::runtime_error("Unknown atom type in resolve_atom_value.");
    }
}

std::string InstructionEvaluator::print_atom_to_string(const Atom& atom, MemoryManager& mm, uint32_t pcb_id) {
    switch (atom.type) {
        case Atom::STRING:
            return atom.string_value;
        case Atom::NUMBER:
            return std::to_string(atom.number_value);
        case Atom::NAME: {
            uint16_t value = resolve_atom_value(atom, mm, pcb_id);
            return std::to_string(value);
        }
        default:
            throw std::runtime_error("Unknown atom type in print_atom_to_string.");
    }
}

void InstructionEvaluator::evaluate(const Expr& expr, MemoryManager& mm, uint32_t pcb_id) {
    switch (expr.type) {
        // DECLARE, READ, WRITE, and SLEEP are handled in the PCB now.
        // This function handles the rest.
        case Expr::ADD: {
            uint16_t left_val = resolve_atom_value(*expr.lhs, mm, pcb_id);
            uint16_t right_val = resolve_atom_value(*expr.rhs, mm, pcb_id);
            uint32_t result32 = static_cast<uint32_t>(left_val) + static_cast<uint32_t>(right_val);
            uint16_t result = (result32 > 65535) ? 65535 : static_cast<uint16_t>(result32);
            uint16_t dest_address = get_or_create_variable_address(expr.var_name);
            mm.write_u16(pcb_id, dest_address, result);
            break;
        }

        case Expr::SUB: {
            uint16_t left_val = resolve_atom_value(*expr.lhs, mm, pcb_id);
            uint16_t right_val = resolve_atom_value(*expr.rhs, mm, pcb_id);
            uint16_t result = (left_val >= right_val) ? (left_val - right_val) : 0;
            uint16_t dest_address = get_or_create_variable_address(expr.var_name);
            mm.write_u16(pcb_id, dest_address, result);
            break;
        }

      case Expr::CALL: {
          if (expr.var_name == "PRINT") {
            std::string output;
            if (expr.atom_value) { // This handles PRINT(arg)
              output = print_atom_to_string(*expr.atom_value, mm, pcb_id);
            } else if (expr.lhs && expr.rhs) { // This handles PRINT(arg1 + arg2)
              output = print_atom_to_string(*expr.lhs, mm, pcb_id) +
                       print_atom_to_string(*expr.rhs, mm, pcb_id);
            } else {
              throw std::runtime_error("Invalid PRINT call format.");
            }

            auto now = std::chrono::system_clock::now();
            auto timestamp = std::format("{:%m/%d/%Y %I:%M:%S %p}", now);
            output_log_.push_back(std::format("({}) \"{}\"", timestamp, output));
          } else {
            // This is for SLEEP(x), etc.
            // Your existing logic for SLEEP inside PCB::executeCurrentInstruction is fine.
            // This part of the evaluator should only handle non-special calls like PRINT.
            throw std::runtime_error("Unknown function call in evaluator: " + expr.var_name);
          }
          break;
      }

        case Expr::FOR: {
            uint16_t iterations = resolve_atom_value(*expr.n, mm, pcb_id);
            for (uint16_t i = 0; i < iterations; ++i) {
                for (const auto& instruction : expr.body) {
                    // Recursive call for nested instructions
                    evaluate(instruction, mm, pcb_id);
                }
            }
            break;
        }

        default:
            // This function should not be called for other instruction types.
            // If it is, it's a logic error in the PCB.
            break;
    }
}

} // namespace osemu