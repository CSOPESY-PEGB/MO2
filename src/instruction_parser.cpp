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

ParseResult InstructionParser::parse_address(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    size_t pos = 0; // Will be updated by stoull to show how many chars were parsed
    try {
        // stoull will parse hex if the string starts with 0x and base is 0.
        // We use base 0 for auto-detection.
        unsigned long long number_val = std::stoull(trimmed, &pos, 0);

        // After the call, we check if it actually parsed a hex number.
        // 1. `pos` must be > 2 (at least "0x" and one digit).
        // 2. The original string must start with "0x" or "0X".
        if (pos < 3 || (trimmed.substr(0, 2) != "0x" && trimmed.substr(0, 2) != "0X")) {
            // It might have parsed a decimal or octal number, which we don't want.
            return ParseResult(false, trimmed, "Address must be a hexadecimal number starting with 0x");
        }
        
        if (number_val > 65535) {
            return ParseResult(false, trimmed, "Address value out of range for uint16_t");
        }
        
        result = Atom(static_cast<uint16_t>(number_val));
        
        // The remaining string is the original trimmed string starting from `pos`.
        return ParseResult(true, trimmed.substr(pos));

    } catch (const std::invalid_argument&) {
        // stoull throws this if no conversion could be performed at all.
        return ParseResult(false, trimmed, "Invalid address format");
    } catch (const std::out_of_range&) {
        // stoull throws this if the number is too big for an unsigned long long.
        return ParseResult(false, trimmed, "Address value out of range for uint16_t");
    }
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

    ParseResult address_result = parse_address(trimmed, result);
    if (address_result.success){
        return address_result;
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


ParseResult InstructionParser::parse_read(const std::string& input, Expr& result){
    std::string remaining;
    if (!consume_tag(input, "READ", remaining)) {
        return ParseResult(false, input, "Expected READ");
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
    
    Atom address_atom(0);
    ParseResult address_result = parse_atom(remaining, address_atom);
    if (!address_result.success) {
        return ParseResult(false, remaining, "Expected address value");
    }
    remaining = address_result.remaining;
    
    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ")", remaining)) {
        return ParseResult(false, remaining, "Expected closing parenthesis");
    }
    
    // Assuming Expr::make_read(string var_name, unique_ptr<Atom> address) exists
    result = Expr::make_read(var_atom.string_value, 
                             std::make_unique<Atom>(address_atom));
    
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_write(const std::string& input, Expr& result){
    std::string remaining;
    if (!consume_tag(input, "WRITE", remaining)) {
        return ParseResult(false, input, "Expected WRITE");
    }

    remaining = ltrim(remaining);
    if (!consume_tag(remaining, "(", remaining)) {
        return ParseResult(false, remaining, "Expected opening parenthesis");
    }

    Atom address_atom(0);
    ParseResult address_result = parse_atom(remaining, address_atom);
    if (!address_result.success) {
        return ParseResult(false, remaining, "Expected address value");
    }
    remaining = address_result.remaining;

    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ",", remaining)) {
        return ParseResult(false, remaining, "Expected comma");
    }

    Atom value_atom(0);
    ParseResult value_result = parse_atom(remaining, value_atom);
    if (!value_result.success) {
        return ParseResult(false, remaining, "Expected value to write (variable or literal)");
    }
    remaining = value_result.remaining;

    remaining = ltrim(remaining);
    if (!consume_tag(remaining, ")", remaining)) {
        return ParseResult(false, remaining, "Expected closing parenthesis");
    }
    
    // Assuming Expr::make_write(unique_ptr<Atom> address, unique_ptr<Atom> value) exists
    result = Expr::make_write(std::make_unique<Atom>(address_atom),
                              std::make_unique<Atom>(value_atom));

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
    
    
    Atom lhs_atom(0);
    ParseResult lhs_res = parse_atom(remaining, lhs_atom);
    if (lhs_res.success) {
        std::string temp_remaining = ltrim(lhs_res.remaining);
        if (!temp_remaining.empty() && temp_remaining[0] == '+') {
            consume_tag(temp_remaining, "+", temp_remaining); 
            
            Atom rhs_atom(0);
            ParseResult rhs_res = parse_atom(ltrim(temp_remaining), rhs_atom);
            if (!rhs_res.success) {
                return ParseResult(false, temp_remaining, "Expected right-hand side for concatenation");
            }
            
            remaining = ltrim(rhs_res.remaining);
            if (!consume_tag(remaining, ")", remaining)) {
                return ParseResult(false, remaining, "Expected closing parenthesis after concatenation");
            }
            
            result = Expr::make_call_concat(name_atom.string_value, 
                                            std::make_unique<Atom>(lhs_atom),
                                            std::make_unique<Atom>(rhs_atom));
            return ParseResult(true, remaining);
        }
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

    ParseResult read_result = parse_read(trimmed, result);
    if (read_result.success){
        return read_result;
    }

    ParseResult write_result = parse_write(trimmed, result);
    if(write_result.success){
        return write_result;
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

}  
