#ifndef OSEMU_INSTRUCTION_EVALUATOR_H_
#define OSEMU_INSTRUCTION_EVALUATOR_H_

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "instruction_parser.hpp"

namespace osemu {

class MemoryManager;

class InstructionEvaluator {
private:
  std::unordered_map<std::string, uint16_t>& symbol_table_;
  std::vector<std::string>& output_log_;
  std::string& process_name_;
  size_t& memory_size_ref_;

public:
  InstructionEvaluator(
      std::unordered_map<std::string, uint16_t>& symbol_table,
      std::vector<std::string>& output_log,
      std::string& process_name,
      size_t& memory_size
  );

  uint16_t get_or_create_variable_address(const std::string& var_name);
  void evaluate(const Expr& expr, MemoryManager& mm, uint32_t pcb_id);
  uint16_t resolve_atom_value(const Atom& atom, MemoryManager& mm, uint32_t pcb_id);
  std::string print_atom_to_string(const Atom& atom, MemoryManager& mm, uint32_t pcb_id);

  const std::vector<std::string>& get_output_log() const { return output_log_; }
};

}

#endif