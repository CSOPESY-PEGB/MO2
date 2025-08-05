#ifndef OSEMU_INSTRUCTION_GENERATOR_H_
#define OSEMU_INSTRUCTION_GENERATOR_H_

#include <vector>
#include <random>
#include <cstdint> // For uint32_t
#include "instruction_parser.hpp"

namespace osemu {

class InstructionGenerator {
public:
  InstructionGenerator();

  // This is the main public function.
  std::vector<Expr> generateRandomProgram(
      uint32_t min_instructions,
      uint32_t max_instructions,
      const std::string& process_name,
      size_t memory_size,
      uint32_t frame_size);

private:
  // --- NEW HELPER FUNCTION DECLARATIONS ---
  std::string generateVariableName(int index);
  size_t generateHeapAddress(size_t heap_size);

  // Private member variables for random number generation
  std::mt19937 rng;
  std::uniform_int_distribution<uint16_t> value_dist;
  std::uniform_int_distribution<uint16_t> add_value_dist;
};

} // namespace osemu

#endif