#ifndef OSEMU_INSTRUCTION_PARSER_H_
#define OSEMU_INSTRUCTION_PARSER_H_

#include <string>
#include <vector>
#include <memory>
#include <variant>
#include <iostream>
#include <unordered_map>

namespace osemu {

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
    
    // Copy constructor
    Expr(const Expr& other)
        : type(other.type), var_name(other.var_name) {
        if (other.atom_value) {
            atom_value = std::make_unique<Atom>(*other.atom_value);
        }
        if (other.lhs) {
            lhs = std::make_unique<Atom>(*other.lhs);
        }
        if (other.rhs) {
            rhs = std::make_unique<Atom>(*other.rhs);
        }
        if (other.n) {
            n = std::make_unique<Atom>(*other.n);
        }
        body = other.body;
    }
    
    // Copy assignment operator
    Expr& operator=(const Expr& other) {
        if (this != &other) {
            type = other.type;
            var_name = other.var_name;
            atom_value.reset();
            if (other.atom_value) {
                atom_value = std::make_unique<Atom>(*other.atom_value);
            }
            
            lhs.reset();
            if (other.lhs) {
                lhs = std::make_unique<Atom>(*other.lhs);
            }
            
            rhs.reset();
            if (other.rhs) {
                rhs = std::make_unique<Atom>(*other.rhs);
            }
            
            n.reset();
            if (other.n) {
                n = std::make_unique<Atom>(*other.n);
            }
            
            body = other.body;
        }
        return *this;
    }
    
    // Move constructor
    Expr(Expr&& other) noexcept
        : type(other.type), var_name(std::move(other.var_name)),
          atom_value(std::move(other.atom_value)),
          lhs(std::move(other.lhs)), rhs(std::move(other.rhs)),
          n(std::move(other.n)), body(std::move(other.body)) {}
    
    // Move assignment operator
    Expr& operator=(Expr&& other) noexcept {
        if (this != &other) {
            type = other.type;
            var_name = std::move(other.var_name);
            atom_value = std::move(other.atom_value);
            lhs = std::move(other.lhs);
            rhs = std::move(other.rhs);
            n = std::move(other.n);
            body = std::move(other.body);
        }
        return *this;
    }
    
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

class InstructionParser {
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

class InstructionEvaluator {
private:
    std::unordered_map<std::string, uint16_t> variables;
    std::vector<std::string> output_log;
    
    uint16_t resolve_atom_value(const Atom& atom);
    std::string print_atom_to_string(const Atom& atom);
    
public:
    InstructionEvaluator();
    
    void evaluate(const Expr& expr);
    void evaluate_program(const std::vector<Expr>& program);
    
    void handle_declare(const std::string& var_name, const Atom& value);
    std::string handle_print(const Atom& arg, const std::string& process_name);
    void handle_sleep(const Atom& duration);
    void handle_add(const std::string& var, const Atom& lhs, const Atom& rhs);
    void handle_sub(const std::string& var, const Atom& lhs, const Atom& rhs);
    void handle_for(const std::vector<Expr>& body, const Atom& count);
    
    void clear_variables();
    void dump_variables() const;
    const std::vector<std::string>& get_output_log() const { return output_log; }
    void clear_output_log() { output_log.clear(); }
};

}  // namespace osemu

#endif  // OSEMU_INSTRUCTION_PARSER_H_