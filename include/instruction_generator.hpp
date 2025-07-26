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
    std::uniform_int_distribution<uint16_t> add_value_dist;

    std::string generateVariableName();
    size_t generateAddress(size_t memory_size);
    size_t generate_power_of_two(size_t, size_t);
    Expr generatePrintInstruction(const std::string& process_name);
    Expr generateDeclareInstruction();
    Expr generateAddInstruction();
    Expr generateSubtractInstruction();
    Expr generateSleepInstruction();
    Expr generateForInstruction(int max_depth = 3);


   public:
    InstructionGenerator();
    
    std::vector<Expr> generateInstructions(size_t count,
                                           const std::string& process_name,
                                           size_t memory_size);
                                           
    std::vector<Expr> generateRandomProgram(size_t min_instructions,
                                            size_t max_instructions,
                                            const std::string& process_name,
                                            size_t min_mem,
                                            size_t max_mem);
};

}  

#endif  
