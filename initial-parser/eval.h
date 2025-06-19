#pragma once
#include "parser.h"
#include <unordered_map>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

class Evaluator {
private:
    std::unordered_map<std::string, uint16_t> variables;
    
    // Helper functions
    uint16_t resolve_atom_value(const Atom& atom);
    void print_atom(const Atom& atom);
    
public:
    Evaluator();
    
    // Main evaluation function
    void evaluate(const Expr& expr);
    void evaluate_program(const std::vector<Expr>& program);
    
    // Instruction handlers
    void handle_declare(const std::string& var_name, const Atom& value);
    void handle_print(const Atom& arg);
    void handle_sleep(const Atom& duration);
    void handle_add(const std::string& var, const Atom& lhs, const Atom& rhs);
    void handle_sub(const std::string& var, const Atom& lhs, const Atom& rhs);
    void handle_for(const std::vector<Expr>& body, const Atom& count);
    
    // Utility functions
    void clear_variables();
    void dump_variables() const;
};