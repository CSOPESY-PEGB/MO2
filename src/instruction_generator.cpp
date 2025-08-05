#include "instruction_generator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
#include <numeric>

namespace osemu {
InstructionGenerator::InstructionGenerator()
    : rng(std::random_device{}()) {}
static std::string generateVariableName(std::mt19937& rng) {
  std::uniform_int_distribution<int> dist(0, 31);
  return "v" + std::to_string(dist(rng));
}
// This generator creates a "hostile" program designed to cause lots of page faults
// by accessing memory pages sequentially but in a random order.
std::vector<Expr> InstructionGenerator::generateRandomProgram(
    uint32_t min_instructions,
    uint32_t max_instructions,
    const std::string& process_name,
    size_t memory_size,
    uint32_t frame_size) {

  std::uniform_int_distribution<size_t> count_dist(min_instructions, max_instructions);
  size_t instruction_count = count_dist(rng);

  std::vector<Expr> instructions;
  if (instruction_count == 0) return instructions;
  instructions.reserve(instruction_count);

  const size_t STACK_SIZE = 64;
  size_t heap_size = (memory_size > STACK_SIZE) ? (memory_size - STACK_SIZE) : 0;

  if (heap_size == 0 || frame_size == 0) {
    return instructions;
  }

  uint32_t num_pages = heap_size / frame_size;
  if (num_pages == 0) return instructions;

  std::vector<uint32_t> page_indices(num_pages);
  std::iota(page_indices.begin(), page_indices.end(), 0);
  std::shuffle(page_indices.begin(), page_indices.end(), rng);

  // --- FIX: Pre-declare all 32 possible variables ---
  // This ensures they all exist and have a location on the stack, maximizing
  // the chances that different instructions will need to access different stack pages.
  for (int i = 0; i < 32; ++i) {
    std::string var_name = "v" + std::to_string(i);
    instructions.push_back(Expr::make_declare(var_name, std::make_unique<Atom>(static_cast<uint16_t>(i))));
  }

  size_t page_access_counter = 0;
  // Adjust loop to account for the declarations we already added
  size_t write_instructions_to_generate = (instruction_count > 32) ? (instruction_count - 32) : 1;

  for (size_t i = 0; i < write_instructions_to_generate; ++i) {
    uint32_t target_page_index = page_indices[page_access_counter];
    uint16_t target_address = target_page_index * frame_size;

    auto addr_to_write = std::make_unique<Atom>(target_address);
    addr_to_write->type = Atom::NUMBER;

    // --- FIX: Use a RANDOM variable as the source, not always "v0" ---
    auto val_to_write = std::make_unique<Atom>(generateVariableName(rng), Atom::NAME);
    instructions.push_back(Expr::make_write(std::move(addr_to_write), std::move(val_to_write)));

    page_access_counter = (page_access_counter + 1) % num_pages;
    if (page_access_counter == 0) {
      std::shuffle(page_indices.begin(), page_indices.end(), rng);
    }
  }

  return instructions;
}
}