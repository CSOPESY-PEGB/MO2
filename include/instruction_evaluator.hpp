#ifndef OSEMU_INSTRUCTION_EVALUATOR_H_
#define OSEMU_INSTRUCTION_EVALUATOR_H_

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "instruction_parser.hpp"

namespace osemu {

// Forward declarations to break circular dependencies
class MemoryManager;
class PCB;

class InstructionEvaluator {
private:
  // References to the PCB's state
  std::unordered_map<std::string, uint16_t>& symbol_table_;
  std::vector<std::string>& output_log_;
  std::string& process_name_;
  size_t& memory_size_ref_; // Reference to the PCB's total memory size

public:
  InstructionEvaluator(
      std::unordered_map<std::string, uint16_t>& symbol_table,
      std::vector<std::string>& output_log,
      std::string& process_name,
      size_t& memory_size
  );

  // Public helper for calculating stack addresses.
  uint16_t get_or_create_variable_address(const std::string& var_name);

  // Core evaluation logic for non-memory-access instructions
  void evaluate(const Expr& expr, MemoryManager& mm, uint32_t pcb_id);

  // Public functions to resolve values, which requires the memory manager
  uint16_t resolve_atom_value(const Atom& atom, MemoryManager& mm, uint32_t pcb_id);
  std::string print_atom_to_string(const Atom& atom, MemoryManager& mm, uint32_t pcb_id);

  // Public log access
  const std::vector<std::string>& get_output_log() const { return output_log_; }
};

} // namespace osemu

#endif