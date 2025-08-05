#include "instruction_parser.hpp"
#include <cctype>
#include <algorithm>
#include <stdexcept>
#include <sstream>

namespace osemu {

// --- IMPLEMENTATION OF Expr RULE OF 5 ---
Expr::Expr(Type t) : type(t) {}

Expr::Expr(const Expr& other)
    : type(other.type), var_name(other.var_name) {
    if (other.atom_value) atom_value = std::make_unique<Atom>(*other.atom_value);
    if (other.lhs) lhs = std::make_unique<Atom>(*other.lhs);
    if (other.rhs) rhs = std::make_unique<Atom>(*other.rhs);
    if (other.n) n = std::make_unique<Atom>(*other.n);
    body = other.body;
}

Expr& Expr::operator=(const Expr& other) {
    if (this != &other) {
        type = other.type;
        var_name = other.var_name;
        atom_value = other.atom_value ? std::make_unique<Atom>(*other.atom_value) : nullptr;
        lhs = other.lhs ? std::make_unique<Atom>(*other.lhs) : nullptr;
        rhs = other.rhs ? std::make_unique<Atom>(*other.rhs) : nullptr;
        n = other.n ? std::make_unique<Atom>(*other.n) : nullptr;
        body = other.body;
    }
    return *this;
}

Expr::Expr(Expr&& other) noexcept = default;
Expr& Expr::operator=(Expr&& other) noexcept = default;

// --- IMPLEMENTATION OF Expr STATIC FACTORY METHODS ---
Expr Expr::make_declare(std::string name, std::unique_ptr<Atom> value) {
    Expr e(DECLARE);
    e.var_name = std::move(name);
    e.atom_value = std::move(value);
    return e;
}

Expr Expr::make_call(std::string name, std::unique_ptr<Atom> arg) {
    Expr e(CALL);
    e.var_name = std::move(name);
    if(arg) e.atom_value = std::move(arg);
    return e;
}

Expr Expr::make_add(std::string var, std::unique_ptr<Atom> lhs_ptr, std::unique_ptr<Atom> rhs_ptr) {
    Expr e(ADD);
    e.var_name = std::move(var);
    e.lhs = std::move(lhs_ptr);
    e.rhs = std::move(rhs_ptr);
    return e;
}

Expr Expr::make_sub(std::string var, std::unique_ptr<Atom> lhs_ptr, std::unique_ptr<Atom> rhs_ptr) {
    Expr e(SUB);
    e.var_name = std::move(var);
    e.lhs = std::move(lhs_ptr);
    e.rhs = std::move(rhs_ptr);
    return e;
}

Expr Expr::make_for(std::vector<Expr> body_vec, std::unique_ptr<Atom> n_ptr) {
    Expr e(FOR);
    e.body = std::move(body_vec);
    e.n = std::move(n_ptr);
    return e;
}

Expr Expr::make_read(std::string var, std::unique_ptr<Atom> address) {
    Expr e(READ);
    e.var_name = std::move(var);
    e.atom_value = std::move(address);
    return e;
}

Expr Expr::make_write(std::unique_ptr<Atom> address, std::unique_ptr<Atom> value) {
    Expr e(WRITE);
    e.lhs = std::move(address);
    e.rhs = std::move(value);
    return e;
}

Expr Expr::make_call_concat(std::string name, std::unique_ptr<Atom> lhs_ptr, std::unique_ptr<Atom> rhs_ptr) {
    Expr e(CALL);
    e.var_name = std::move(name);
    e.lhs = std::move(lhs_ptr);
    e.rhs = std::move(rhs_ptr);
    return e;
}

// --- IMPLEMENTATION OF InstructionParser ---
std::string InstructionParser::ltrim(const std::string& input) {
    size_t start = input.find_first_not_of(" \t\n\r");
    return (start == std::string::npos) ? "" : input.substr(start);
}

bool InstructionParser::consume_tag(const std::string& input, const std::string& tag, std::string& remaining) {
    std::string trimmed = ltrim(input);
    if (trimmed.rfind(tag, 0) == 0) {
        remaining = trimmed.substr(tag.length());
        return true;
    }
    return false;
}

ParseResult InstructionParser::parse_string(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    if (trimmed.empty() || trimmed[0] != '"') return ParseResult(false, trimmed, "Expected opening quote");

    size_t i = 1;
    while (i < trimmed.length() && (trimmed[i] != '"' || trimmed[i-1] == '\\')) i++;
    if (i >= trimmed.length()) return ParseResult(false, trimmed, "Expected closing quote");

    result = Atom(trimmed.substr(1, i - 1), Atom::STRING);
    return ParseResult(true, trimmed.substr(i + 1));
}

ParseResult InstructionParser::parse_name(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    if (trimmed.empty() || !isalpha(static_cast<unsigned char>(trimmed[0]))) return ParseResult(false, trimmed);

    size_t i = 1;
    while (i < trimmed.length() && (isalnum(static_cast<unsigned char>(trimmed[i])) || trimmed[i] == '_')) i++;

    result = Atom(trimmed.substr(0, i), Atom::NAME);
    return ParseResult(true, trimmed.substr(i));
}

ParseResult InstructionParser::parse_number(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    if (trimmed.empty() || !isdigit(static_cast<unsigned char>(trimmed[0]))) return ParseResult(false, trimmed);

    size_t pos = 0;
    try {
        unsigned long val = std::stoul(trimmed, &pos, 10);
        if (val > 65535) return ParseResult(false, trimmed, "Number out of range for uint16_t");
        result = Atom(static_cast<uint16_t>(val));
    } catch (...) {
        return ParseResult(false, trimmed, "Invalid number format");
    }
    return ParseResult(true, trimmed.substr(pos));
}

ParseResult InstructionParser::parse_address(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);
    if (trimmed.rfind("0x", 0) != 0) return ParseResult(false, trimmed);

    size_t pos = 0;
    try {
        unsigned long long val = std::stoull(trimmed, &pos, 16);
        if (pos <= 2) return ParseResult(false, trimmed);
        if (val > 65535) return ParseResult(false, trimmed, "Address out of range");
        result = Atom(trimmed.substr(0, pos), Atom::STRING);
        return ParseResult(true, trimmed.substr(pos));
    } catch (...) {
        return ParseResult(false, trimmed, "Invalid hex format");
    }
}

ParseResult InstructionParser::parse_atom(const std::string& input, Atom& result) {
    std::string trimmed = ltrim(input);

    auto res = parse_string(trimmed, result); if (res.success) return res;
    res = parse_address(trimmed, result); if (res.success) return res;
    res = parse_name(trimmed, result); if (res.success) return res;
    res = parse_number(trimmed, result); if (res.success) return res;

    return ParseResult(false, trimmed, "Expected string, name, hex address, or number");
}

ParseResult InstructionParser::parse_declare(const std::string& input, Expr& result) {
    std::string remaining;
    if (!consume_tag(input, "DECLARE", remaining)) return ParseResult(false, input);
    if (!consume_tag(remaining, "(", remaining)) return ParseResult(false, remaining, "Expected '(' after DECLARE");

    Atom name_atom(0);
    auto name_res = parse_name(remaining, name_atom);
    if (!name_res.success) return ParseResult(false, remaining, "Expected variable name in DECLARE");
    remaining = name_res.remaining;

    if (!consume_tag(remaining, ",", remaining)) return ParseResult(false, remaining, "Expected ',' in DECLARE");

    Atom value_atom(0);
    auto value_res = parse_atom(remaining, value_atom);
    if (!value_res.success) return ParseResult(false, remaining, "Expected value in DECLARE");
    remaining = value_res.remaining;

    if (!consume_tag(remaining, ")", remaining)) return ParseResult(false, remaining, "Expected ')' to close DECLARE");

    result = Expr::make_declare(name_atom.string_value, std::make_unique<Atom>(value_atom));
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_add(const std::string& input, Expr& result) {
    std::string remaining;
    const std::string op = "ADD";
    if (!consume_tag(input, op, remaining)) return ParseResult(false, input);
    if (!consume_tag(remaining, "(", remaining)) return ParseResult(false, remaining, "Expected '(' after " + op);

    Atom var_atom(0);
    auto var_res = parse_name(remaining, var_atom);
    if (!var_res.success) return ParseResult(false, remaining, "Expected destination variable in " + op);
    remaining = var_res.remaining;

    if (!consume_tag(remaining, ",", remaining)) return ParseResult(false, remaining, "Expected ',' in " + op);

    Atom lhs_atom(0);
    auto lhs_res = parse_atom(remaining, lhs_atom);
    if (!lhs_res.success) return ParseResult(false, remaining, "Expected left-hand operand in " + op);
    remaining = lhs_res.remaining;

    if (!consume_tag(remaining, ",", remaining)) return ParseResult(false, remaining, "Expected ',' in " + op);

    Atom rhs_atom(0);
    auto rhs_res = parse_atom(remaining, rhs_atom);
    if (!rhs_res.success) return ParseResult(false, remaining, "Expected right-hand operand in " + op);
    remaining = rhs_res.remaining;

    if (!consume_tag(remaining, ")", remaining)) return ParseResult(false, remaining, "Expected ')' to close " + op);

    result = Expr::make_add(var_atom.string_value, std::make_unique<Atom>(lhs_atom), std::make_unique<Atom>(rhs_atom));
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_sub(const std::string& input, Expr& result) {
    std::string remaining;
    const std::string op = "SUB";
    if (!consume_tag(input, op, remaining)) return ParseResult(false, input);
    if (!consume_tag(remaining, "(", remaining)) return ParseResult(false, remaining, "Expected '(' after " + op);

    Atom var_atom(0);
    auto var_res = parse_name(remaining, var_atom);
    if (!var_res.success) return ParseResult(false, remaining, "Expected destination variable in " + op);
    remaining = var_res.remaining;

    if (!consume_tag(remaining, ",", remaining)) return ParseResult(false, remaining, "Expected ',' in " + op);

    Atom lhs_atom(0);
    auto lhs_res = parse_atom(remaining, lhs_atom);
    if (!lhs_res.success) return ParseResult(false, remaining, "Expected left-hand operand in " + op);
    remaining = lhs_res.remaining;

    if (!consume_tag(remaining, ",", remaining)) return ParseResult(false, remaining, "Expected ',' in " + op);

    Atom rhs_atom(0);
    auto rhs_res = parse_atom(remaining, rhs_atom);
    if (!rhs_res.success) return ParseResult(false, remaining, "Expected right-hand operand in " + op);
    remaining = rhs_res.remaining;

    if (!consume_tag(remaining, ")", remaining)) return ParseResult(false, remaining, "Expected ')' to close " + op);

    result = Expr::make_sub(var_atom.string_value, std::make_unique<Atom>(lhs_atom), std::make_unique<Atom>(rhs_atom));
    return ParseResult(true, remaining);
}


ParseResult InstructionParser::parse_read(const std::string& input, Expr& result){
    std::string remaining;
    if (!consume_tag(input, "READ", remaining)) return ParseResult(false, input);
    if (!consume_tag(remaining, "(", remaining)) return ParseResult(false, remaining, "Expected '(' after READ");

    Atom var_atom(0);
    auto var_res = parse_name(remaining, var_atom);
    if (!var_res.success) return ParseResult(false, remaining, "Expected variable name in READ");
    remaining = var_res.remaining;

    if (!consume_tag(remaining, ",", remaining)) return ParseResult(false, remaining, "Expected ',' in READ");

    Atom addr_atom(0);
    auto addr_res = parse_atom(remaining, addr_atom);
    if (!addr_res.success) return ParseResult(false, remaining, "Expected address in READ");
    remaining = addr_res.remaining;

    if (!consume_tag(remaining, ")", remaining)) return ParseResult(false, remaining, "Expected ')' to close READ");

    result = Expr::make_read(var_atom.string_value, std::make_unique<Atom>(addr_atom));
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_write(const std::string& input, Expr& result){
    std::string remaining;
    if (!consume_tag(input, "WRITE", remaining)) return ParseResult(false, input);
    if (!consume_tag(remaining, "(", remaining)) return ParseResult(false, remaining, "Expected '(' after WRITE");

    Atom addr_atom(0);
    auto addr_res = parse_atom(remaining, addr_atom);
    if (!addr_res.success) return ParseResult(false, remaining, "Expected address in WRITE");
    remaining = addr_res.remaining;

    if (!consume_tag(remaining, ",", remaining)) return ParseResult(false, remaining, "Expected ',' in WRITE");

    Atom value_atom(0);
    auto value_res = parse_atom(remaining, value_atom);
    if (!value_res.success) return ParseResult(false, remaining, "Expected value in WRITE");
    remaining = value_res.remaining;

    if (!consume_tag(remaining, ")", remaining)) return ParseResult(false, remaining, "Expected ')' to close WRITE");

    result = Expr::make_write(std::make_unique<Atom>(addr_atom), std::make_unique<Atom>(value_atom));
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_call(const std::string& input, Expr& result) {
    std::string remaining;
    Atom name_atom(0);
    auto name_res = parse_name(input, name_atom);
    if (!name_res.success) return ParseResult(false, input);
    remaining = name_res.remaining;

    if (!consume_tag(remaining, "(", remaining)) return ParseResult(false, remaining, "Expected '(' for function call");

    std::string after_paren = ltrim(remaining);
    if (after_paren.rfind(")", 0) == 0) {
        result = Expr::make_call(name_atom.string_value, nullptr);
        return ParseResult(true, after_paren.substr(1));
    }

    Atom lhs_atom(0);
    auto lhs_res = parse_atom(remaining, lhs_atom);
    if (!lhs_res.success) return ParseResult(false, remaining, "Expected argument in call");

    remaining = ltrim(lhs_res.remaining);
    if (consume_tag(remaining, "+", remaining)) {
        Atom rhs_atom(0);
        auto rhs_res = parse_atom(remaining, rhs_atom);
        if (!rhs_res.success) return ParseResult(false, remaining, "Expected right-hand side of concatenation");
        remaining = ltrim(rhs_res.remaining);

        if (!consume_tag(remaining, ")", remaining)) return ParseResult(false, remaining, "Expected ')' after concatenation");
        result = Expr::make_call_concat(name_atom.string_value, std::make_unique<Atom>(lhs_atom), std::make_unique<Atom>(rhs_atom));
    } else {
        if (!consume_tag(remaining, ")", remaining)) return ParseResult(false, remaining, "Expected ')' after single argument");
        result = Expr::make_call(name_atom.string_value, std::make_unique<Atom>(lhs_atom));
    }

    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_for(const std::string& input, Expr& result) {
    std::string remaining;
    if (!consume_tag(input, "FOR", remaining)) return ParseResult(false, input);
    if (!consume_tag(remaining, "(", remaining)) return ParseResult(false, remaining, "Expected '(' after FOR");
    if (!consume_tag(remaining, "[", remaining)) return ParseResult(false, remaining, "Expected '[' for loop body");

    std::vector<Expr> body;
    remaining = ltrim(remaining);
    while (remaining[0] != ']') {
        Expr expr(Expr::VOID_EXPR);
        auto expr_res = parse_expr(remaining, expr);
        if (!expr_res.success) return ParseResult(false, remaining, "Invalid expression in FOR body");
        body.push_back(std::move(expr));
        remaining = ltrim(expr_res.remaining);
        if (consume_tag(remaining, ",", remaining)) remaining = ltrim(remaining);
    }

    if (!consume_tag(remaining, "]", remaining)) return ParseResult(false, remaining, "Expected ']' to close loop body");
    if (!consume_tag(remaining, ",", remaining)) return ParseResult(false, remaining, "Expected ',' after loop body");

    Atom n_atom(0);
    auto n_res = parse_atom(remaining, n_atom);
    if (!n_res.success) return ParseResult(false, remaining, "Expected loop count");
    remaining = n_res.remaining;

    if (!consume_tag(remaining, ")", remaining)) return ParseResult(false, remaining, "Expected ')' to close FOR");

    result = Expr::make_for(std::move(body), std::make_unique<Atom>(n_atom));
    return ParseResult(true, remaining);
}

ParseResult InstructionParser::parse_expr(const std::string& input, Expr& result) {
    std::string trimmed = ltrim(input);

    if (trimmed.rfind("DECLARE", 0) == 0) return parse_declare(trimmed, result);
    if (trimmed.rfind("ADD", 0) == 0) return parse_add(trimmed, result);
    if (trimmed.rfind("SUB", 0) == 0) return parse_sub(trimmed, result);
    if (trimmed.rfind("READ", 0) == 0) return parse_read(trimmed, result);
    if (trimmed.rfind("WRITE", 0) == 0) return parse_write(trimmed, result);
    if (trimmed.rfind("FOR", 0) == 0) return parse_for(trimmed, result);

    auto call_res = parse_call(trimmed, result);
    if (call_res.success) return call_res;

    return ParseResult(false, trimmed, "Unknown or invalid expression");
}

ParseResult InstructionParser::parse_program(const std::string& input, std::vector<Expr>& result) {
    result.clear();
    std::stringstream ss(input);
    std::string line;
    int line_num = 0;

    while(std::getline(ss, line)) {
        line_num++;
        line = ltrim(line);
        if (line.empty() || line[0] == '#') continue;

        Expr expr(Expr::VOID_EXPR);
        ParseResult expr_result = parse_expr(line, expr);
        if (!expr_result.success) {
            return ParseResult(false, line, expr_result.error_msg + " (line " + std::to_string(line_num) + ")");
        }
        result.push_back(std::move(expr));
    }

    return ParseResult(true, "");
}

}