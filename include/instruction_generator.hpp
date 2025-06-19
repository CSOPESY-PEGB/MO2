#ifndef OSEMU_INSTRUCTION_GENERATOR_H_
#define OSEMU_INSTRUCTION_GENERATOR_H_

#include <vector>
#include <random>
#include "instruction_parser.hpp"

namespace osemu {

class InstructionGenerator {
private:
    std::mt19937 rng;
    std::uniform_int_distribution<int> instruction_type_dist;
    std::uniform_int_distribution<uint16_t> value_dist;
    std::uniform_int_distribution<int> var_name_dist;
    std::uniform_int_distribution<int> for_count_dist;
    std::uniform_int_distribution<int> for_body_size_dist;
    
    std::string generateVariableName();
    Expr generatePrintInstruction(const std::string& process_name);
    Expr generateDeclareInstruction();
    Expr generateAddInstruction();
    Expr generateSubtractInstruction();
    Expr generateSleepInstruction();
    Expr generateForInstruction(int max_depth = 3);
    
public:
    InstructionGenerator();
    
    std::vector<Expr> generateInstructions(size_t count, const std::string& process_name);
    std::vector<Expr> generateRandomProgram(size_t min_instructions, size_t max_instructions, const std::string& process_name);
};

}  // namespace osemu

#endif  // OSEMU_INSTRUCTION_GENERATOR_H_