#include "instruction_parser.hpp"
#include <cctype>
#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <format>

namespace osemu {

std::string InstructionParser::ltrim(const std::string& input) {
    size_t start = 0;
    while (start < input.length() && std::isspace(static_cast<unsigned char>(input[start]))) {
        start++;
    }
    return input.substr(start);
}

bool InstructionParser::consume_tag(const std::string& input, const std::string& tag, std::string& remaining) {
    std::string trimmed = ltrim(input);
    if (trimmed.length() >= tag.length() && trimmed.substr(0, tag.length()) == tag) {
        remaining = trimmed.substr(tag.length());
        return true;
    }
    return false;
}

ParseResult InstructionParser::parse_string(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    
    if (trimmed.empty() || trimmed[0] != '"') {
        return ParseResult(false, trimmed, "Expected opening quote");
    }
    
    size_t i = 1;
    while (i < trimmed.length() && trimmed[i] != '"') {
        i++;
    }
    
    if (i >= trimmed.length()) {
        return ParseResult(false, trimmed, "Expected closing quote");
    }
    
    std::string str_content = trimmed.substr(1, i - 1);
    result = Atom(str_content, Atom::STRING);
    
    return ParseResult(true, trimmed.substr(i + 1));
}

ParseResult InstructionParser::parse_name(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    
    if (trimmed.empty() || !std::isalpha(static_cast<unsigned char>(trimmed[0]))) {
        return ParseResult(false, trimmed, "Expected alphabetic character");
    }
    
    size_t i = 0;
    while (i < trimmed.length() && std::isalpha(static_cast<unsigned char>(trimmed[i]))) {
        i++;
    }
    
    std::string name = trimmed.substr(0, i);
    result = Atom(name, Atom::NAME);
    
    return ParseResult(true, trimmed.substr(i));
}

ParseResult InstructionParser::parse_number(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    
    if (trimmed.empty() || !std::isdigit(static_cast<unsigned char>(trimmed[0]))) {
        return ParseResult(false, trimmed, "Expected digit");
    }
    
    size_t i = 0;
    while (i < trimmed.length() && std::isdigit(static_cast<unsigned char>(trimmed[i]))) {
        i++;
    }
    
    std::string number_str = trimmed.substr(0, i);
    try {
        unsigned long number_val = std::stoul(number_str);
        if (number_val > 65535) {
            return ParseResult(false, trimmed, "Number out of range for uint16_t");
        }
        result = Atom(static_cast<uint16_t>(number_val));
    } catch (const std::invalid_argument&) {
        return ParseResult(false, trimmed, "Invalid number format");
    } catch (const std::out_of_range&) {
        return ParseResult(false, trimmed, "Number out of range for uint16_t");
    }
    
    return ParseResult(true, trimmed.substr(i));
}

ParseResult InstructionParser::parse_atom(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    
    ParseResult string_result = parse_string(trimmed, result);
    if (string_result.success) {
        return string_result;
    }
    
    ParseResult name_result = parse_name(trimmed, result);
    if (name_result.success) {
        return name_result;
    }
    
    ParseResult number_result = parse_number(trimmed, result);
    if (number_result.success) {
        return number_result;
    }
    
    return ParseResult(false, trimmed, "Expected string, name, or number");
}

ParseResult InstructionParser::parse_declare(const std::string& input, Expr& result) {
    std::string remaining;
    if (!consume_tag(input, "DECLARE", remaining)) {
        return ParseResult(false, input, "Expected DECLARE");
    }
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, "(", remaining)) {
        return ParseResult(false, remaining, "Expected opening parenthesis");
    }
    
    Atom name_atom(0);
    ParseResult name_result = parse_name(remaining, name_atom);
    if (!name_result.success) {
        return ParseResult(false, remaining, "Expected variable name");
    }
    remaining = name_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ",", remaining)) {
        return ParseResult(false, remaining, "Expected comma");
    }
    
    Atom value_atom(0);
    ParseResult value_result = parse_atom(remaining, value_atom);
    if (!value_result.success) {
        return ParseResult(false, remaining, "Expected value");
    }
    remaining = value_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ")", remaining)) {
        return ParseResult(false, remaining, "Expected closing parenthesis");
    }
    
    result = Expr::make_declare(name_atom.string_value, 
                               std::make_unique<Atom>(value_atom));
    
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_add(const std::string& input, Expr& result) {
    std::string remaining;
    if (!consume_tag(input, "ADD", remaining)) {
        return ParseResult(false, input, "Expected ADD");
    }
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, "(", remaining)) {
        return ParseResult(false, remaining, "Expected opening parenthesis");
    }
    
    Atom var_atom(0);
    ParseResult var_result = parse_name(remaining, var_atom);
    if (!var_result.success) {
        return ParseResult(false, remaining, "Expected variable name");
    }
    remaining = var_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ",", remaining)) {
        return ParseResult(false, remaining, "Expected comma");
    }
    
    Atom lhs_atom(0);
    ParseResult lhs_result = parse_atom(remaining, lhs_atom);
    if (!lhs_result.success) {
        return ParseResult(false, remaining, "Expected left operand");
    }
    remaining = lhs_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ",", remaining)) {
        return ParseResult(false, remaining, "Expected comma");
    }
    
    Atom rhs_atom(0);
    ParseResult rhs_result = parse_atom(remaining, rhs_atom);
    if (!rhs_result.success) {
        return ParseResult(false, remaining, "Expected right operand");
    }
    remaining = rhs_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ")", remaining)) {
        return ParseResult(false, remaining, "Expected closing parenthesis");
    }
    
    result = Expr::make_add(var_atom.string_value,
                           std::make_unique<Atom>(lhs_atom),
                           std::make_unique<Atom>(rhs_atom));
    
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_sub(const std::string& input, Expr& result) {
    std::string remaining;
    if (!consume_tag(input, "SUBTRACT", remaining)) {
        return ParseResult(false, input, "Expected SUBTRACT");
    }
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, "(", remaining)) {
        return ParseResult(false, remaining, "Expected opening parenthesis");
    }
    
    Atom var_atom(0);
    ParseResult var_result = parse_name(remaining, var_atom);
    if (!var_result.success) {
        return ParseResult(false, remaining, "Expected variable name");
    }
    remaining = var_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ",", remaining)) {
        return ParseResult(false, remaining, "Expected comma");
    }
    
    Atom lhs_atom(0);
    ParseResult lhs_result = parse_atom(remaining, lhs_atom);
    if (!lhs_result.success) {
        return ParseResult(false, remaining, "Expected left operand");
    }
    remaining = lhs_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ",", remaining)) {
        return ParseResult(false, remaining, "Expected comma");
    }
    
    Atom rhs_atom(0);
    ParseResult rhs_result = parse_atom(remaining, rhs_atom);
    if (!rhs_result.success) {
        return ParseResult(false, remaining, "Expected right operand");
    }
    remaining = rhs_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ")", remaining)) {
        return ParseResult(false, remaining, "Expected closing parenthesis");
    }
    
    result = Expr::make_sub(var_atom.string_value,
                           std::make_unique<Atom>(lhs_atom),
                           std::make_unique<Atom>(rhs_atom));
    
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_call(const std::string& input, Expr& result) {
    std::string trimmed = ltrim(input);
    
    Atom name_atom(0);
    ParseResult name_result = parse_name(trimmed, name_atom);
    if (!name_result.success) {
        return ParseResult(false, trimmed, "Expected function name");
    }
    std::string remaining = name_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, "(", remaining)) {
        return ParseResult(false, remaining, "Expected opening parenthesis");
    }
    
    Atom arg_atom(0);
    ParseResult arg_result = parse_atom(remaining, arg_atom);
    if (!arg_result.success) {
        return ParseResult(false, remaining, "Expected argument");
    }
    remaining = arg_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ")", remaining)) {
        return ParseResult(false, remaining, "Expected closing parenthesis");
    }
    
    result = Expr::make_call(name_atom.string_value,
                            std::make_unique<Atom>(arg_atom));
    
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_for(const std::string& input, Expr& result) {
    std::string remaining;
    if (!consume_tag(input, "FOR", remaining)) {
        return ParseResult(false, input, "Expected FOR");
    }
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, "(", remaining)) {
        return ParseResult(false, remaining, "Expected opening parenthesis");
    }
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, "[", remaining)) {
        return ParseResult(false, remaining, "Expected opening bracket");
    }
    
    std::vector<Expr> body;
    remaining = ltrim(remaining);
    
    while (!remaining.empty() && remaining[0] != ']') {
        Expr expr(Expr::VOID_EXPR);
        ParseResult expr_result = parse_expr(remaining, expr);
        if (!expr_result.success) {
            break;
        }
        body.push_back(std::move(expr));
        remaining = ltrim(expr_result.remaining);
        
        if (!remaining.empty() && remaining[0] == ',') {
            remaining = remaining.substr(1);
            remaining = ltrim(remaining);
        }
    }
    
    if (!consume_tag(remaining, "]", remaining)) {
        return ParseResult(false, remaining, "Expected closing bracket");
    }
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ",", remaining)) {
        return ParseResult(false, remaining, "Expected comma");
    }
    
    Atom n_atom(0);
    ParseResult n_result = parse_atom(remaining, n_atom);
    if (!n_result.success) {
        return ParseResult(false, remaining, "Expected loop count");
    }
    remaining = n_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ")", remaining)) {
        return ParseResult(false, remaining, "Expected closing parenthesis");
    }
    
    result = Expr::make_for(std::move(body), std::make_unique<Atom>(n_atom));
    
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_expr(const std::string& input, Expr& result) {
    std::string trimmed = ltrim(input);
    
    ParseResult declare_result = parse_declare(trimmed, result);
    if (declare_result.success) {
        return declare_result;
    }
    
    ParseResult add_result = parse_add(trimmed, result);
    if (add_result.success) {
        return add_result;
    }
    
    ParseResult sub_result = parse_sub(trimmed, result);
    if (sub_result.success) {
        return sub_result;
    }
    
    ParseResult for_result = parse_for(trimmed, result);
    if (for_result.success) {
        return for_result;
    }
    
    ParseResult call_result = parse_call(trimmed, result);
    if (call_result.success) {
        return call_result;
    }
    
    return ParseResult(false, trimmed, "Expected expression");
}

ParseResult InstructionParser::parse_program(const std::string& input, std::vector<Expr>& result) {
    std::string remaining = input;
    result.clear();
    
    while (!remaining.empty()) {
        remaining = ltrim(remaining);
        if (remaining.empty()) {
            break;
        }
        
        Expr expr(Expr::VOID_EXPR);
        ParseResult expr_result = parse_expr(remaining, expr);
        if (!expr_result.success) {
            return ParseResult(false, remaining, expr_result.error_msg);
        }
        
        result.push_back(std::move(expr));
        remaining = expr_result.remaining;
    }
    
    return ParseResult(true, remaining);
}

// InstructionEvaluator implementation

InstructionEvaluator::InstructionEvaluator() {
    // Initialize with empty variable map and output log
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
        
        case Expr::CALL: {
            if (!expr.atom_value) {
                throw std::runtime_error("Invalid CALL: missing argument");
            }
            if (expr.var_name == "PRINT") {
                // For now, use empty process name - will be set by caller
                handle_print(*expr.atom_value, "");
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

void InstructionEvaluator::evaluate_program(const std::vector<Expr>& program) {
    for (const auto& expr : program) {
        evaluate(expr);
    }
}

uint16_t InstructionEvaluator::resolve_atom_value(const Atom& atom) {
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

std::string InstructionEvaluator::print_atom_to_string(const Atom& atom) {
    switch (atom.type) {
        case Atom::STRING:
            return atom.string_value;
        case Atom::NUMBER:
            return std::to_string(atom.number_value);
        case Atom::NAME: {
            // Resolve variable name to its value
            auto it = variables.find(atom.string_value);
            if (it != variables.end()) {
                return std::to_string(it->second);
            } else {
                return "0"; // Default value for undefined variables
            }
        }
        default:
            throw std::runtime_error("Unknown atom type in print");
    }
}

void InstructionEvaluator::handle_declare(const std::string& var_name, const Atom& value) {
    if (value.type == Atom::NUMBER) {
        variables[var_name] = value.number_value;
    } else if (value.type == Atom::NAME) {
        // Resolve the variable value
        variables[var_name] = resolve_atom_value(value);
    } else {
        throw std::runtime_error("DECLARE requires numeric value or variable reference");
    }
}

std::string InstructionEvaluator::handle_print(const Atom& arg, const std::string& process_name) {
    std::string output = print_atom_to_string(arg);
    // Create log entry with timestamp
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
    // Note: In the actual emulator, this should interact with the CPU cycle system
    // For now, we'll just store the sleep duration - the scheduler will handle it
}

void InstructionEvaluator::handle_add(const std::string& var, const Atom& lhs, const Atom& rhs) {
    uint16_t left_val = resolve_atom_value(lhs);
    uint16_t right_val = resolve_atom_value(rhs);
    
    // Perform addition with overflow protection (uint16 max is 65535)
    uint32_t result = static_cast<uint32_t>(left_val) + static_cast<uint32_t>(right_val);
    if (result > 65535) {
        result = 65535; // Clamp to max uint16 as per spec
    }
    
    variables[var] = static_cast<uint16_t>(result);
}

void InstructionEvaluator::handle_sub(const std::string& var, const Atom& lhs, const Atom& rhs) {
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

void InstructionEvaluator::handle_for(const std::vector<Expr>& body, const Atom& count) {
    uint16_t iterations = resolve_atom_value(count);
    
    for (uint16_t i = 0; i < iterations; i++) {
        for (const auto& instruction : body) {
            evaluate(instruction);
        }
    }
}

void InstructionEvaluator::clear_variables() {
    variables.clear();
}

void InstructionEvaluator::dump_variables() const {
    // This could be used for debugging - variables are stored in the evaluator
}

}  // namespace osemu