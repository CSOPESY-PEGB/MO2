#pragma once
#include <string>
#include <vector>
#include <memory>
#include <variant>
#include <iostream>

struct Atom {
    enum Type { STRING, NAME, NUMBER };
    Type type;
    std::string string_value;
    uint16_t number_value;
    
    Atom(std::string s, Type t) : type(t), string_value(std::move(s)), number_value(0) {}
    Atom(uint16_t n) : type(NUMBER), number_value(n) {}
    
    std::string to_string() const {
        switch (type) {
            case STRING:
            case NAME:
                return string_value;
            case NUMBER:
                return std::to_string(number_value);
        }
        return "";
    }
};

struct Expr {
    enum Type { DECLARE, CALL, CONSTANT, VOID_EXPR, ADD, SUB, FOR };
    Type type;
    
    std::string var_name;
    std::unique_ptr<Atom> atom_value;
    std::unique_ptr<Atom> lhs;
    std::unique_ptr<Atom> rhs;
    std::unique_ptr<Atom> n;
    std::vector<Expr> body;
    
    Expr(Type t) : type(t) {}
    
    static Expr make_declare(std::string name, std::unique_ptr<Atom> value) {
        Expr e(DECLARE);
        e.var_name = std::move(name);
        e.atom_value = std::move(value);
        return e;
    }
    
    static Expr make_call(std::string name, std::unique_ptr<Atom> arg) {
        Expr e(CALL);
        e.var_name = std::move(name);
        e.atom_value = std::move(arg);
        return e;
    }
    
    static Expr make_add(std::string var, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs) {
        Expr e(ADD);
        e.var_name = std::move(var);
        e.lhs = std::move(lhs);
        e.rhs = std::move(rhs);
        return e;
    }
    
    static Expr make_sub(std::string var, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs) {
        Expr e(SUB);
        e.var_name = std::move(var);
        e.lhs = std::move(lhs);
        e.rhs = std::move(rhs);
        return e;
    }
    
    static Expr make_for(std::vector<Expr> body, std::unique_ptr<Atom> n) {
        Expr e(FOR);
        e.body = std::move(body);
        e.n = std::move(n);
        return e;
    }
    
    static Expr make_constant(std::unique_ptr<Atom> value) {
        Expr e(CONSTANT);
        e.atom_value = std::move(value);
        return e;
    }
};

struct ParseResult {
    bool success;
    std::string remaining;
    std::string error_msg;
    
    ParseResult(bool s, const std::string& r, const std::string& e = "") 
        : success(s), remaining(r), error_msg(e) {}
};

class Parser {
public:
    static std::string ltrim(const std::string& input);
    static ParseResult parse_string(const std::string& input, Atom& result);
    static ParseResult parse_name(const std::string& input, Atom& result);
    static ParseResult parse_number(const std::string& input, Atom& result);
    static ParseResult parse_atom(const std::string& input, Atom& result);
    static ParseResult parse_declare(const std::string& input, Expr& result);
    static ParseResult parse_add(const std::string& input, Expr& result);
    static ParseResult parse_sub(const std::string& input, Expr& result);
    static ParseResult parse_call(const std::string& input, Expr& result);
    static ParseResult parse_for(const std::string& input, Expr& result);
    static ParseResult parse_expr(const std::string& input, Expr& result);
    static ParseResult parse_program(const std::string& input, std::vector<Expr>& result);
    
private:
    static bool consume_tag(const std::string& input, const std::string& tag, std::string& remaining);
};
