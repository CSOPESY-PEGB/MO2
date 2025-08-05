#include "instruction_generator.hpp"

#include <algorithm>
#include <cmath>  // For std::log2, etc.
#include <vector>

namespace osemu {

InstructionGenerator::InstructionGenerator()
    : rng(std::random_device{}()),
      value_dist(1, 1000),
      add_value_dist(1, 10) {}

std::string InstructionGenerator::generateVariableName(int index) {
    // Generates predictable variable names like v0, v1... up to v31
    return "v" + std::to_string(index % 32);
}



// In instruction_generator.cpp

// Find this helper function and make sure it uses the full range.
// Your existing one might be limiting the range.
size_t InstructionGenerator::generateHeapAddress(size_t heap_size) {
  if (heap_size <= sizeof(uint16_t)) return 0;
  // This distribution ensures you can get addresses anywhere in the heap.
  std::uniform_int_distribution<size_t> dist(0, heap_size - sizeof(uint16_t));
  return dist(rng);
}


std::vector<Expr> InstructionGenerator::generateRandomProgram(
    uint32_t min_instructions,
    uint32_t max_instructions,
    const std::string& process_name,
    size_t memory_size,
    uint32_t frame_size) { // <-- The new parameter is available here

    std::uniform_int_distribution<size_t> count_dist(min_instructions, max_instructions);
    size_t instruction_count = count_dist(rng);

    std::vector<Expr> instructions;
    instructions.reserve(instruction_count);

    const size_t STACK_SIZE = 64;
    size_t heap_size = (memory_size > STACK_SIZE) ? (memory_size - STACK_SIZE) : 0;

    if (heap_size == 0 || frame_size == 0) {
        return instructions; // Cannot generate memory-accessing instructions
    }

    // Dynamically calculate the number of pages using the passed-in frame_size
    uint32_t num_pages = heap_size / frame_size;
    if (num_pages == 0) return instructions; // Not enough heap space for even one page

    // --- HOSTILE PAGE ACCESS LOGIC ---
    std::vector<uint32_t> page_indices(num_pages);
    std::iota(page_indices.begin(), page_indices.end(), 0);
    std::shuffle(page_indices.begin(), page_indices.end(), rng);

    // instructions.push_back(Expr::make_declare("v0", std::make_unique<Atom>(static_cast<uint16_t>(123))));

    size_t page_access_counter = 0;
    for (size_t i = 0; i < instruction_count; ++i) {
        uint32_t target_page = page_indices[page_access_counter];

        // Calculate the address using the dynamic frame_size
        uint16_t target_address = target_page * frame_size;

        auto addr_to_write = std::make_unique<Atom>(target_address);
        auto val_to_write = std::make_unique<Atom>("v0", Atom::NAME);
        instructions.push_back(Expr::make_write(std::move(addr_to_write), std::move(val_to_write)));

        page_access_counter++;
        if (page_access_counter >= num_pages) {
            std::shuffle(page_indices.begin(), page_indices.end(), rng);
            page_access_counter = 0;
        }
    }

    return instructions;
}
// std::vector<Expr> InstructionGenerator::generateRandomProgram(
//     uint32_t min_instructions,
//     uint32_t max_instructions,
//     const std::string& process_name,
//     size_t memory_size) {
//
//     std::uniform_int_distribution<size_t> count_dist(min_instructions, max_instructions);
//     size_t instruction_count = count_dist(rng);
//
//     std::vector<Expr> instructions;
//     instructions.reserve(instruction_count);
//
//     // The heap is all memory *except* the top 64 bytes reserved for the stack.
//     const size_t STACK_SIZE = 64;
//     size_t heap_size = (memory_size > STACK_SIZE) ? (memory_size - STACK_SIZE) : 0;
//
//     // Build a list of possible instructions we can generate.
//     enum class InstrType { ADD, PRINT, DECLARE, READ, WRITE, SLEEP };
//     std::vector<InstrType> possible_instructions;
//
//     possible_instructions.push_back(InstrType::ADD);
//     possible_instructions.push_back(InstrType::PRINT);
//     possible_instructions.push_back(InstrType::DECLARE);
//     possible_instructions.push_back(InstrType::SLEEP);
//
//     if (heap_size > 0) {
//         possible_instructions.push_back(InstrType::READ);
//         possible_instructions.push_back(InstrType::WRITE);
//     }
//
//     std::uniform_int_distribution<size_t> instr_dist(0, possible_instructions.size() - 1);
//
//     for (size_t i = 0; i < instruction_count; ++i) {
//         InstrType choice = possible_instructions[instr_dist(rng)];
//
//         switch (choice) {
//             case InstrType::ADD: {
//                 auto lhs = std::make_unique<Atom>(generateVariableName(i), Atom::NAME);
//                 auto rhs = std::make_unique<Atom>(add_value_dist(rng));
//                 instructions.push_back(Expr::make_add(generateVariableName(i + 1), std::move(lhs), std::move(rhs)));
//                 break;
//             }
//             case InstrType::PRINT: {
//                 auto lhs = std::make_unique<Atom>("Value of ", Atom::STRING);
//                 auto rhs = std::make_unique<Atom>(generateVariableName(i), Atom::NAME);
//                 instructions.push_back(Expr::make_call_concat("PRINT", std::move(lhs), std::move(rhs)));
//                 break;
//             }
//             case InstrType::DECLARE: {
//                 uint16_t value = value_dist(rng);
//                 auto val_atom = std::make_unique<Atom>(value);
//                 instructions.push_back(Expr::make_declare(generateVariableName(i), std::move(val_atom)));
//                 break;
//             }
//             case InstrType::READ: {
//                 auto addr_atom = std::make_unique<Atom>(static_cast<uint16_t>(generateHeapAddress(heap_size)));
//                 instructions.push_back(Expr::make_read(generateVariableName(i), std::move(addr_atom)));
//                 break;
//             }
//             case InstrType::WRITE: {
//                 auto addr_atom = std::make_unique<Atom>(static_cast<uint16_t>(generateHeapAddress(heap_size)));
//                 auto val_atom = std::make_unique<Atom>(generateVariableName(i), Atom::NAME);
//                 instructions.push_back(Expr::make_write(std::move(addr_atom), std::move(val_atom)));
//                 break;
//             }
//             case InstrType::SLEEP: {
//                  std::uniform_int_distribution<uint16_t> sleep_dist(1, 5);
//                  auto sleep_atom = std::make_unique<Atom>(sleep_dist(rng));
//                  instructions.push_back(Expr::make_call("SLEEP", std::move(sleep_atom)));
//                  break;
//             }
//         }
//     }
//
//     // // To be safe, always declare the first variable 'v0' so it exists for PRINT/ADD operations.
//     // if (!instructions.empty()) {
//     //     instructions.insert(instructions.begin(), Expr::make_declare("v0", std::make_unique<Atom>(static_cast<uint16_t>(0))));
//     // }
//
//     return instructions;
// }

} // namespace osemu