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
    std::vector<PageTableEntry>& page_table, 
    std::unordered_map<std::string, uint16_t>& symbol_table, 
    std::vector<std::string>& output_log, 
    std::string& process_name, size_t mem_per_frame) 
    : page_table(page_table), 
    symbol_table(symbol_table), 
    output_log(output_log), 
    process_name(process_name), mem_per_frame(mem_per_frame) {

    }

uint16_t InstructionEvaluator::get_or_create_variable_address(const std::string& var_name) {
    // 1. Check if the variable already exists.
    if (symbol_table.contains(var_name)) {
        return symbol_table[var_name]; // Return its existing address.
    }

    // 2. If it doesn't exist, create it. First, check limits.
    if (symbol_table.size() >= 32) { // 32 is your symbol_table_limit
        throw std::runtime_error("Variable limit reached, cannot create: " + var_name);
    }

    // 3. Calculate the new address from the top of the heap.
    // The number of variables that will exist *after* this one is created.
    size_t new_var_count = symbol_table.size() + 1;
    
    // Check if there's enough space for another 2-byte variable.
    if (65536 < new_var_count * 2) {
        throw std::runtime_error("Out of stack memory for new variable: " + var_name);
    }

    uint16_t new_address = 65536 - (new_var_count * 2);
    
    // 4. Add the new variable to the symbol table.
    symbol_table[var_name] = new_address;
    
    return new_address;
}

uint16_t InstructionEvaluator::read_u16_from_heap(uint16_t address) {
    // Safety check
    if (address >= 65536){
        std::cout << "MEMORY ACCESS VIOLATION: OUT OF BOUNDS" << std::endl;
        throw std::runtime_error("MEMORY ACCESS VIOLATION" + std::to_string(address));
    }
    // check if address is in memory, if not call page fault
    int page_needed = address / mem_per_frame;
    if (!page_table[page_needed].valid) {
        std::cout << "PAGE FAULT : " + std::to_string(address) << std::endl;
        throw std::runtime_error("Not loaded, requesting page fault" + std::to_string(address));
    }

    //if loaded we can just read normally
    uint16_t upper = page_table[page_needed].data[address + 1];
    uint16_t lower = page_table[page_needed].data[address];
    return (upper << 8) | lower;
}

void InstructionEvaluator::write_u16_to_heap(uint16_t address, uint16_t value){
    if (address >= heap_end) {
        throw std::runtime_error("MEMORY VIOLATION: Attempt to WRITE into protected instruction/symbol table area at address " + std::to_string(address));
    }
    write_u16_to_mem(address, value);
}

//privileged write
void InstructionEvaluator::write_u16_to_mem(uint16_t address, uint16_t value) {
    if (address >= 65536){
        std::cout << "MEMORY ACCESS VIOLATION: OUT OF BOUNDS" << std::endl;
        throw std::runtime_error("MEMORY ACCESS VIOLATION" + std::to_string(address));
    }

    // check if address is in memory, if not call page fault
    int page_needed = address / mem_per_frame;
    if (!page_table[page_needed].valid) {
        std::cout << "PAGE FAULT : " + std::to_string(address) << std::endl;
        throw std::runtime_error("NOT LOADED, requesting page fault" + std::to_string(address));
    }

    //mark page as dirty
    page_table[page_needed].dirty = true;

    // Little-Endian write
    page_table[page_needed].data[address] = static_cast<uint8_t>(value);
    page_table[page_needed].data[address + 1] = static_cast<uint8_t>(value >> 8);
}

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
        case Atom::NAME: 
        if (symbol_table.contains(atom.string_value)) {
            uint16_t address = symbol_table.at(atom.string_value);
            return read_u16_from_heap(address);
        }
        return 0; // Return 0 if variable not found.
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
            // 1. Resolve the variable name to its numeric value.
            uint16_t value = resolve_atom_value(atom);
            // 2. Convert that number to a string and return it.
            return std::to_string(value);
        }
        default:
            throw std::runtime_error("Unknown atom type in print");
    }
}


void InstructionEvaluator::handle_declare(const std::string& var_name, const Atom& value) {
    uint16_t dest_address = get_or_create_variable_address(var_name);
    uint16_t value_to_store = resolve_atom_value(value);
    write_u16_to_mem(dest_address, value_to_store);
}

void InstructionEvaluator::handle_read(const std::string& var_name, const Atom& rhs) {
    uint16_t source_address = rhs.number_value;
    uint16_t value_read = read_u16_from_heap(source_address);
    uint16_t dest_address = get_or_create_variable_address(var_name);
    write_u16_to_mem(dest_address, value_read); // This should be privileged_write...
}


//ADD VALIDATION HERE!!
void InstructionEvaluator::handle_write(const Atom& address_atom, const Atom& rhs) {
    uint16_t dest_address = address_atom.number_value;
    uint16_t value_to_write = resolve_atom_value(rhs);
    write_u16_to_heap(dest_address, value_to_write);
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
    uint16_t left_val = resolve_atom_value(lhs);
    uint16_t right_val = resolve_atom_value(rhs);
    
    uint32_t result32 = static_cast<uint32_t>(left_val) + static_cast<uint32_t>(right_val);
    uint16_t result = (result32 > 65535) ? 65535 : static_cast<uint16_t>(result32);
    
    uint16_t dest_address = get_or_create_variable_address(var);
    write_u16_to_mem(dest_address, result);
}

void InstructionEvaluator::handle_sub(const std::string& var, const Atom& lhs, const Atom& rhs) {
    uint16_t left_val = resolve_atom_value(lhs);
    uint16_t right_val = resolve_atom_value(rhs);
    
    uint16_t result = (left_val >= right_val) ? (left_val - right_val) : 0;
    
    uint16_t dest_address = get_or_create_variable_address(var);
    write_u16_to_mem(dest_address, result);
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