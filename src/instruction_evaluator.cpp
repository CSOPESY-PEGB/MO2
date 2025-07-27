#include "instruction_evaluator.hpp"
#include <string>
#include <vector>
#include <memory>
#include <variant>
#include <iostream>
#include <unordered_map>
#include <format>
#include <chrono>
#include <memory>

namespace osemu {

InstructionEvaluator::InstructionEvaluator(
    std::vector<uint8_t>& heap_memory, 
    std::unordered_map<std::string, uint16_t>& symbol_table, 
    std::vector<std::string>& output_log, 
    std::string& process_name) 
    : heap_memory(heap_memory), 
    symbol_table(symbol_table), 
    output_log(output_log), 
    process_name(process_name) {}

void InstructionEvaluator::evaluate(const Expr& expr) {
    switch (expr.type) {
        case Expr::DECLARE: {
            if (expr.var_name.empty() || !expr.atom_value) {
                throw std::runtime_error("Invalid DECLARE: missing variable name or value");
            }
            handle_declare(expr.var_name, *expr.atom_value);
            break;
        }

        case Expr::READ: {
            if (expr.var_name.empty() || !expr.atom_value){
                throw std::runtime_error("Invalid READ!");
            }
            handle_read(expr.var_name, *expr.atom_value);
            break;
        }

        case Expr::WRITE:{
            //idk how to validate this actually, this is temporary lang
            if (!expr.lhs->number_value > heap_memory.size()) {
                throw std::runtime_error("Invalid WRITE!");
            }
            handle_write(*expr.lhs, *expr.rhs);
            break;
        }
        
        case Expr::CALL: {
            if (expr.var_name == "PRINT") {
                if (expr.atom_value) { 
                    handle_print(*expr.atom_value, "");
                } else if (expr.lhs && expr.rhs) { 
                    std::string lhs_str = print_atom_to_string(*expr.lhs);
                    std::string rhs_str = print_atom_to_string(*expr.rhs);
                    Atom concatenated_atom(lhs_str + rhs_str, Atom::STRING);
                    handle_print(concatenated_atom, "");
                } else {
                    throw std::runtime_error("Invalid PRINT call: malformed arguments");
                }
            } else if (expr.var_name == "SLEEP") {
                if (!expr.atom_value) {
                    throw std::runtime_error("Invalid SLEEP call: missing argument");
                }
                handle_sleep(*expr.atom_value);
            } else {
                throw std::runtime_error("Unknown function: " + expr.var_name);
            }
            break;
        }
        
        case Expr::ADD: {
            if (expr.var_name.empty() || !expr.lhs || !expr.rhs) {
                throw std::runtime_error("Invalid ADD: missing variable name or operands");
            }
            handle_add(expr.var_name, *expr.lhs, *expr.rhs);
            break;
        }
        
        case Expr::SUB: {
            if (expr.var_name.empty() || !expr.lhs || !expr.rhs) {
                throw std::runtime_error("Invalid SUB: missing variable name or operands");
            }
            handle_sub(expr.var_name, *expr.lhs, *expr.rhs);
            break;
        }
        
        case Expr::FOR: {
            if (!expr.n) {
                throw std::runtime_error("Invalid FOR: missing loop count");
            }
            handle_for(expr.body, *expr.n);
            break;
        }
        
        case Expr::CONSTANT:
        case Expr::VOID_EXPR:
            
            break;
            
        default:
            throw std::runtime_error("Unknown expression type");
    }
}

void InstructionEvaluator::evaluate_program(const std::vector<Expr>& program) {
    for (const auto& expr : program) {
        evaluate(expr);
    }
}

//I changed this since now variables are stored in the symbol table and memory;
//Basic example of how it works say we have 128 bytes in memory right?
//Top 64 bits are allocated to the symbol table 0x40 - 0x80
//Our symbol table now stores {variable_name : address} instead of previous {variable_name : value}
//now resolve atom value basically goes heap_memory[symbol_table[variable_name]]
uint16_t InstructionEvaluator::resolve_atom_value(const Atom& atom) {
    switch (atom.type) {
        case Atom::NAME: {
            auto it = symbol_table.find(atom.string_value);
            if (it != symbol_table.end()) {
                uint16_t address = it->second; //address of the variable in the memory 

                //combine the two bits
                uint16_t combined = static_cast<uint16_t>(heap_memory[address+1]) << 8;
                combined |= static_cast<uint16_t>(heap_memory[address]);

                return combined;

            } else {
                return 0;
            }
        }
        case Atom::NUMBER:
            return atom.number_value;
        case Atom::STRING:
            throw std::runtime_error("Cannot convert string to numeric value");
        default:
            throw std::runtime_error("Unknown atom type");
    }
}

std::string InstructionEvaluator::print_atom_to_string(const Atom& atom) {
    switch (atom.type) {
        case Atom::STRING:
            return atom.string_value;
        case Atom::NUMBER:
            return std::to_string(atom.number_value);
        case Atom::NAME: {
            
            auto it = symbol_table.find(atom.string_value);
            if (it != symbol_table.end()) {
                uint16_t address = it->second; //address of the variable in the memory 
                
                //combine the two bits
                uint16_t combined = static_cast<uint16_t>(heap_memory[address+1]) << 8;
                combined |= static_cast<uint16_t>(heap_memory[address]);

                return std::to_string(combined);
            } else {
                return "0"; 
            }
        }
        default:
            throw std::runtime_error("Unknown atom type in print");
    }
}



void InstructionEvaluator::handle_declare(const std::string& var_name, const Atom& value) {

    //check first if we have exceeded 32 variables
    if(symbol_table.size() >=32 && !symbol_table.contains(var_name)){
        throw std::runtime_error("EXCEEDED variables");
    }

    int16_t mem_lower;
    int16_t mem_higher;

    if(symbol_table.contains(var_name)){
        mem_lower = symbol_table[var_name];
        mem_higher = mem_lower + 1;
    }else {
        //basically get the lower and upper half of the address
        size_t index = symbol_table.size() + 1; //index of variable being declared
        mem_lower = heap_memory.size() - (index * 2);
        mem_higher = mem_lower + 1;
        symbol_table[var_name] = mem_lower;
    }


    if (value.type == Atom::NUMBER) {
        //assign value to the address
        heap_memory[mem_lower] = static_cast<uint8_t>(value.number_value);
        heap_memory[mem_higher] = static_cast<uint8_t>(value.number_value >> 8);

    } else if (value.type == Atom::NAME) {
        uint16_t val = resolve_atom_value(value);
        heap_memory[mem_lower] = static_cast<uint8_t>(val);
        heap_memory[mem_higher] = static_cast<uint8_t>(val >> 8);
    } else {
        throw std::runtime_error("DECLARE requires numeric value or variable reference");
    }

    
}

void InstructionEvaluator::handle_read(const std::string& var_name, const Atom& rhs){

    //check first if we have exceeded 32 variables
    if(symbol_table.size() >=32 && !symbol_table.contains(var_name)){
        throw std::runtime_error("EXCEEDED variables");
    }

    uint16_t mem_lower;
    uint16_t mem_higher;

    //read the two bytes (uint16_t...)
    uint8_t low = heap_memory[rhs.number_value]; //lower word
    uint8_t high = heap_memory[rhs.number_value+1]; //upper word

    //combine two bytes to form uint16
    uint16_t combined = static_cast<uint16_t>(high) << 8;
    combined |= static_cast<uint16_t>(low);

    //assign to a variable
    if(symbol_table.contains(var_name)){
        mem_lower = symbol_table[var_name];
        mem_higher = mem_lower + 1;
    }else {
        //basically get the lower and upper half of the address
        size_t index = symbol_table.size() + 1; //index of variable being declared
        mem_lower = heap_memory.size() - (index * 2);
        mem_higher = mem_lower + 1;
        symbol_table[var_name] = mem_lower;
    }

    heap_memory[mem_lower] = low;
    heap_memory[mem_higher] = high;
    
};

void InstructionEvaluator::handle_write(const Atom& address, const Atom& rhs){
    
    //TODO: ADD VALIDATION HERE IF NEED

    uint16_t writing_lower = address.number_value; //we will write here lsb
    uint16_t writing_higher = address.number_value + 1; //we will write here msb
    uint16_t to_write = (rhs.type == Atom::NAME) ? resolve_atom_value(rhs) : rhs.number_value; //this is what we will write

    heap_memory[writing_lower] = static_cast<uint8_t>(to_write);
    heap_memory[writing_higher] = static_cast<uint8_t>(to_write >> 8);

    
}

std::string InstructionEvaluator::handle_print(const Atom& arg, const std::string& process_name) {
    std::string output = print_atom_to_string(arg);
    
    auto now = std::chrono::system_clock::now();
    auto truncated_time = std::chrono::time_point_cast<std::chrono::seconds>(now);
    std::string timestamp = std::format("{:%m/%d/%Y %I:%M:%S %p}", truncated_time);
 
    std::string log_entry;
    if (!process_name.empty()) {
        log_entry = std::format("({}) \"{}\"", timestamp, output);
    } else {
        log_entry = std::format("({}) \"{}\"", timestamp, output);
    }
    
    output_log.push_back(log_entry);
    return output;
}

void InstructionEvaluator::handle_sleep(const Atom& duration) {
    uint16_t cycles = resolve_atom_value(duration);
    
    
}

void InstructionEvaluator::handle_add(const std::string& var, const Atom& lhs, const Atom& rhs) {
    // Step 1: Resolve operand values (this part was already correct)
    uint16_t left_val = resolve_atom_value(lhs);
    uint16_t right_val = resolve_atom_value(rhs);
    
    // Step 2: Perform calculation (this part was already correct)
    uint32_t result32 = static_cast<uint32_t>(left_val) + static_cast<uint32_t>(right_val);
    uint16_t result = (result32 > 65535) ? 65535 : static_cast<uint16_t>(result32);
    
    // Step 3: Get or create the address for the destination variable.
    // (This reuses the same logic from your other functions)
    uint16_t dest_address;
    if (symbol_table.contains(var)) {
        dest_address = symbol_table[var];
    } else {
        // You could also throw an error here if you want ADD to only work on existing variables.
        // For now, we'll allow it to create a new variable.
        if (symbol_table.size() >= 32) {
            throw std::runtime_error("Variable limit reached, cannot create: " + var);
        }
        size_t index = symbol_table.size() + 1;
        dest_address = heap_memory.size() - (index * 2);
        symbol_table[var] = dest_address;
    }

    // Step 4: Write the result to the destination address in memory (Little-Endian).
    // THIS IS THE KEY FIX.
    heap_memory[dest_address]     = static_cast<uint8_t>(result);
    heap_memory[dest_address + 1] = static_cast<uint8_t>(result >> 8);
}

void InstructionEvaluator::handle_sub(const std::string& var, const Atom& lhs, const Atom& rhs) {
    // Step 1: Resolve operand values (this part was already correct)
    uint16_t left_val = resolve_atom_value(lhs);
    uint16_t right_val = resolve_atom_value(rhs);
    
    // Step 2: Perform calculation (this part was already correct)
    uint32_t result32 = static_cast<uint32_t>(left_val) - static_cast<uint32_t>(right_val);
    uint16_t result = (result32 > 65535) ? 65535 : static_cast<uint16_t>(result32);
    
    // Step 3: Get or create the address for the destination variable.
    // (This reuses the same logic from your other functions)
    uint16_t dest_address;
    if (symbol_table.contains(var)) {
        dest_address = symbol_table[var];
    } else {
        // You could also throw an error here if you want ADD to only work on existing variables.
        // For now, we'll allow it to create a new variable.
        if (symbol_table.size() >= 32) {
            throw std::runtime_error("Variable limit reached, cannot create: " + var);
        }
        size_t index = symbol_table.size() + 1;
        dest_address = heap_memory.size() - (index * 2);
        symbol_table[var] = dest_address;
    }

    // Step 4: Write the result to the destination address in memory (Little-Endian).
    // THIS IS THE KEY FIX.
    heap_memory[dest_address]     = static_cast<uint8_t>(result);
    heap_memory[dest_address + 1] = static_cast<uint8_t>(result >> 8);
}

void InstructionEvaluator::handle_for(const std::vector<Expr>& body, const Atom& count) {
    uint16_t iterations = resolve_atom_value(count);
    
    for (uint16_t i = 0; i < iterations; i++) {
        for (const auto& instruction : body) {
            evaluate(instruction);
        }
    }
}

void InstructionEvaluator::clear_variables() {
    symbol_table.clear();
}

}