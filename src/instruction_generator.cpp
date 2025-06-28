#include "instruction_generator.hpp"
#include <random>

namespace osemu {

InstructionGenerator::InstructionGenerator() 
    : rng(std::random_device{}()),
      instruction_type_dist(0, 5), 
      value_dist(1, 1000),
      var_name_dist(0, 25), 
      for_count_dist(1, 5),
      for_body_size_dist(1, 3),
      add_value_dist(1,10)
{
}

std::string InstructionGenerator::generateVariableName() {
    char var_char = 'a' + var_name_dist(rng);
    return std::string(1, var_char);
}

Expr InstructionGenerator::generatePrintInstruction(const std::string& process_name) {
    
    std::string message = "Hello world from " + process_name + "!";
    auto atom = std::make_unique<Atom>(message, Atom::STRING);
    return Expr::make_call("PRINT", std::move(atom));
}

Expr InstructionGenerator::generateDeclareInstruction() {
    std::string var_name = generateVariableName();
    uint16_t value = value_dist(rng);
    auto atom = std::make_unique<Atom>(value);
    return Expr::make_declare(var_name, std::move(atom));
}

Expr InstructionGenerator::generateAddInstruction() {
    std::string result_var = generateVariableName();
    std::string operand1 = generateVariableName();
    uint16_t operand2_val = value_dist(rng);
    
    auto lhs = std::make_unique<Atom>(operand1, Atom::NAME);
    auto rhs = std::make_unique<Atom>(operand2_val);
    
    return Expr::make_add(result_var, std::move(lhs), std::move(rhs));
}

Expr InstructionGenerator::generateSubtractInstruction() {
    std::string result_var = generateVariableName();
    std::string operand1 = generateVariableName();
    uint16_t operand2_val = value_dist(rng);
    
    auto lhs = std::make_unique<Atom>(operand1, Atom::NAME);
    auto rhs = std::make_unique<Atom>(operand2_val);
    
    return Expr::make_sub(result_var, std::move(lhs), std::move(rhs));
}

Expr InstructionGenerator::generateSleepInstruction() {
    std::uniform_int_distribution<uint16_t> sleep_dist(1, 10); 
    uint16_t sleep_cycles = sleep_dist(rng);
    auto atom = std::make_unique<Atom>(sleep_cycles);
    return Expr::make_call("SLEEP", std::move(atom));
}

Expr InstructionGenerator::generateForInstruction(int max_depth) {
    if (max_depth <= 0) {
        
        return generatePrintInstruction("nested");
    }
    
    std::vector<Expr> body;
    int body_size = for_body_size_dist(rng);
    
    for (int i = 0; i < body_size; i++) {
        int instr_type = instruction_type_dist(rng);
        switch (instr_type) {
            case 0: body.push_back(generatePrintInstruction("loop")); break;
            case 1: body.push_back(generateDeclareInstruction()); break;
            case 2: body.push_back(generateAddInstruction()); break;
            case 3: body.push_back(generateSubtractInstruction()); break;
            case 4: body.push_back(generateSleepInstruction()); break;
            case 5: 
                if (max_depth > 1) {
                    body.push_back(generateForInstruction(max_depth - 1));
                } else {
                    body.push_back(generatePrintInstruction("nested"));
                }
                break;
        }
    }
    
    uint16_t loop_count = for_count_dist(rng);
    auto count_atom = std::make_unique<Atom>(loop_count);
    
    return Expr::make_for(std::move(body), std::move(count_atom));
}

std::vector<Expr> InstructionGenerator::generateInstructions(size_t count, const std::string& process_name) {
    std::vector<Expr> instructions;
    instructions.reserve(count);
    
    for (size_t i = 0; i < count; i++) {
        if (i % 2 == 0) {
            auto lhs = std::make_unique<Atom>("Value from: ", Atom::STRING);
            auto rhs = std::make_unique<Atom>("x", Atom::NAME);
            instructions.push_back(Expr::make_call_concat("PRINT", std::move(lhs), std::move(rhs)));
        } else {
            uint16_t add_val = add_value_dist(rng);
            auto lhs = std::make_unique<Atom>("x", Atom::NAME);
            auto rhs = std::make_unique<Atom>(add_val);
            instructions.push_back(Expr::make_add("x", std::move(lhs), std::move(rhs)));
        }
    }
    
    return instructions;
}

std::vector<Expr> InstructionGenerator::generateRandomProgram(size_t min_instructions, size_t max_instructions, const std::string& process_name) {
    std::uniform_int_distribution<size_t> count_dist(min_instructions, max_instructions);
    size_t instruction_count = count_dist(rng);
    
    return generateInstructions(instruction_count, process_name);
}

}  
