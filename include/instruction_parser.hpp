#ifndef OSEMU_INSTRUCTION_PARSER_H_
#define OSEMU_INSTRUCTION_PARSER_H_

#include <string>
#include <vector>
#include <memory>
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
        return "UNKNOWN_ATOM";
    }
};

struct Expr {
    enum Type { DECLARE, CALL, CONSTANT, VOID_EXPR, ADD, SUB, FOR, READ, WRITE };
    Type type;

    std::string var_name;
    std::unique_ptr<Atom> atom_value; // For single-arg calls, DECLARE, READ
    std::unique_ptr<Atom> lhs;        // For ADD, SUB, WRITE, concat-CALL
    std::unique_ptr<Atom> rhs;        // For ADD, SUB, WRITE, concat-CALL
    std::unique_ptr<Atom> n;          // For FOR loops
    std::vector<Expr> body;           // For FOR loops

    Expr(Type t);

    // Rule of 5 for proper resource management with unique_ptrs
    Expr(const Expr& other);
    Expr& operator=(const Expr& other);
    Expr(Expr&& other) noexcept;
    Expr& operator=(Expr&& other) noexcept;

    static Expr make_declare(std::string name, std::unique_ptr<Atom> value);
    static Expr make_call(std::string name, std::unique_ptr<Atom> arg);
    static Expr make_add(std::string var, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs);
    static Expr make_sub(std::string var, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs);
    static Expr make_for(std::vector<Expr> body, std::unique_ptr<Atom> n);
    static Expr make_read(std::string var_name, std::unique_ptr<Atom> address);
    static Expr make_write(std::unique_ptr<Atom> address, std::unique_ptr<Atom> value);
    static Expr make_call_concat(std::string name, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs);
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
    static ParseResult parse_program(const std::string& input, std::vector<Expr>& result);

private:
    static std::string ltrim(const std::string& input);
    static bool consume_tag(const std::string& input, const std::string& tag, std::string& remaining);

    static ParseResult parse_string(const std::string& input, Atom& result);
    static ParseResult parse_name(const std::string& input, Atom& result);
    static ParseResult parse_number(const std::string& input, Atom& result);
    static ParseResult parse_address(const std::string& input, Atom& result);
    static ParseResult parse_atom(const std::string& input, Atom& result);

    static ParseResult parse_declare(const std::string& input, Expr& result);
    static ParseResult parse_add(const std::string& input, Expr& result);
    static ParseResult parse_sub(const std::string& input, Expr& result);
    static ParseResult parse_read(const std::string& input, Expr& result);
    static ParseResult parse_write(const std::string& input, Expr& result);
    static ParseResult parse_call(const std::string& input, Expr& result);
    static ParseResult parse_for(const std::string& input, Expr& result);
    static ParseResult parse_expr(const std::string& input, Expr& result);
};

}

#endif