#ifndef OSEMU_INSTRUCTION_GENERATOR_H_
#define OSEMU_INSTRUCTION_GENERATOR_H_

#include <vector>
#include <random>
#include <cstdint>
#include "instruction_parser.hpp"

namespace osemu {

class InstructionGenerator {
public:
  InstructionGenerator();

  std::vector<Expr> generateRandomProgram(
      uint32_t min_instructions,
      uint32_t max_instructions,
      const std::string& process_name,
      size_t memory_size,
      uint32_t frame_size);

private:
  std::mt19937 rng;
};

}

#endif