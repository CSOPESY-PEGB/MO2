#include "eval.h"
#include <stdexcept>
#include <sstream>

Evaluator::Evaluator() {
    // Initialize with empty variable map
}

void Evaluator::evaluate(const Expr& expr) {
    switch (expr.type) {
        case Expr::DECLARE: {
            if (expr.var_name.empty() || !expr.atom_value) {
                throw std::runtime_error("Invalid DECLARE: missing variable name or value");
            }
            handle_declare(expr.var_name, *expr.atom_value);
            break;
        }
        
        case Expr::CALL: {
            if (!expr.atom_value) {
                throw std::runtime_error("Invalid CALL: missing argument");
            }
            if (expr.var_name == "PRINT") {
                handle_print(*expr.atom_value);
            } else if (expr.var_name == "SLEEP") {
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
            // No operation needed for constants and void
            break;
            
        default:
            throw std::runtime_error("Unknown expression type");
    }
}

void Evaluator::evaluate_program(const std::vector<Expr>& program) {
    for (const auto& expr : program) {
        evaluate(expr);
    }
}

uint16_t Evaluator::resolve_atom_value(const Atom& atom) {
    switch (atom.type) {
        case Atom::NAME: {
            auto it = variables.find(atom.string_value);
            if (it != variables.end()) {
                return it->second;
            } else {
                // Variable not found, return 0 as per spec
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

void Evaluator::print_atom(const Atom& atom) {
    switch (atom.type) {
        case Atom::STRING:
            std::cout << atom.string_value;
            break;
        case Atom::NUMBER:
            std::cout << atom.number_value;
            break;
        case Atom::NAME: {
            // Resolve variable name to its value
            auto it = variables.find(atom.string_value);
            if (it != variables.end()) {
                std::cout << it->second;
            } else {
                std::cout << 0; // Default value for undefined variables
            }
            break;
        }
        default:
            throw std::runtime_error("Unknown atom type in print");
    }
}

void Evaluator::handle_declare(const std::string& var_name, const Atom& value) {
    if (value.type == Atom::NUMBER) {
        variables[var_name] = value.number_value;
    } else if (value.type == Atom::NAME) {
        // Resolve the variable value
        variables[var_name] = resolve_atom_value(value);
    } else {
        throw std::runtime_error("DECLARE requires numeric value or variable reference");
    }
}

void Evaluator::handle_print(const Atom& arg) {
    print_atom(arg);
    std::cout << std::endl;
}

void Evaluator::handle_sleep(const Atom& duration) {
    uint16_t ms = resolve_atom_value(duration);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void Evaluator::handle_add(const std::string& var, const Atom& lhs, const Atom& rhs) {
    uint16_t left_val = resolve_atom_value(lhs);
    uint16_t right_val = resolve_atom_value(rhs);
    
    // Perform addition with overflow protection (uint16 max is 65535)
    uint32_t result = static_cast<uint32_t>(left_val) + static_cast<uint32_t>(right_val);
    if (result > 65535) {
        result = 65535; // Clamp to max uint16 as per spec
    }
    
    variables[var] = static_cast<uint16_t>(result);
}

void Evaluator::handle_sub(const std::string& var, const Atom& lhs, const Atom& rhs) {
    uint16_t left_val = resolve_atom_value(lhs);
    uint16_t right_val = resolve_atom_value(rhs);
    
    // Perform subtraction with underflow protection (uint16 min is 0)
    uint16_t result;
    if (left_val >= right_val) {
        result = left_val - right_val;
    } else {
        result = 0; // Clamp to min uint16 as per spec
    }
    
    variables[var] = result;
}

void Evaluator::handle_for(const std::vector<Expr>& body, const Atom& count) {
    uint16_t iterations = resolve_atom_value(count);
    
    for (uint16_t i = 0; i < iterations; i++) {
        for (const auto& instruction : body) {
            evaluate(instruction);
        }
    }
}

void Evaluator::clear_variables() {
    variables.clear();
}

void Evaluator::dump_variables() const {
    std::cout << "Variables:" << std::endl;
    for (const auto& pair : variables) {
        std::cout << "  " << pair.first << " = " << pair.second << std::endl;
    }
}
