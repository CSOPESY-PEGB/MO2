#ifndef PCB_HPP
#define PCB_HPP

#include <iostream>
#include <vector>
#include <variant>
#include <string>
#include <memory>
#include <fstream>
#include <stdexcept>
#include <utility>

// Forward declaration of Expr
struct Expr;

// Represents a basic unit of data in an expression.
struct Atom {
    enum Type { STRING, NAME, NUMBER };
    Type type;
    std::string string_value;
    uint16_t number_value;

    // Constructors
    Atom(std::string s, Type t) : type(t), string_value(std::move(s)), number_value(0) {}
    Atom(uint16_t n) : type(NUMBER), number_value(n), string_value("") {}
    // Default constructor for deserialization
    Atom() : type(NUMBER), number_value(0) {}


    std::string to_string() const {
        switch (type) {
            case STRING:
            case NAME:
                return string_value;
            case NUMBER:
                return std::to_string(number_value);
        }
        return "THIS SHOULDNT HAPPEN";
    }

    // Serialization
    void write(std::ofstream& out) const;
    // Deserialization
    void read(std::ifstream& in);
};

// Represents an expression in the program.
struct Expr {
    enum Type { DECLARE, CALL, CONSTANT, VOID_EXPR, ADD, SUB, FOR, READ, WRITE };
    Type type;

    std::string var_name;
    std::unique_ptr<Atom> atom_value;
    std::unique_ptr<Atom> lhs;
    std::unique_ptr<Atom> rhs;
    std::unique_ptr<Atom> n;
    std::vector<Expr> body;

    // Constructors
    Expr(Type t = VOID_EXPR) : type(t) {}
    Expr(const Expr& other);
    Expr& operator=(const Expr& other);
    Expr(Expr&& other) noexcept;
    Expr& operator=(Expr&& other) noexcept;

    // Factory methods
    static Expr make_declare(std::string name, std::unique_ptr<Atom> value);
    static Expr make_call(std::string name, std::unique_ptr<Atom> arg);
    static Expr make_call_concat(std::string name, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs);
    static Expr make_add(std::string var, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs);
    static Expr make_sub(std::string var, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs);
    static Expr make_for(std::vector<Expr> body, std::unique_ptr<Atom> n);
    static Expr make_constant(std::unique_ptr<Atom> value);
    static Expr make_read(std::string var_name, std::unique_ptr<Atom> address);
    static Expr make_write(std::unique_ptr<Atom> address, std::unique_ptr<Atom> value);

    // Serialization and Deserialization
    void write(std::ofstream& out) const;
    void read(std::ifstream& in);
};

// Represents a Process Control Block.
class pcb {
private:
    uint32_t pcb_id;
    std::vector<std::variant<uint16_t, Expr>> program;
    static const std::string storage_file;

    // Helper functions for serialization
    void write_string(std::ofstream& out, const std::string& s) const;
    void write_atom(std::ofstream& out, const Atom& atom) const;
    void write_expr(std::ofstream& out, const Expr& expr) const;

    // Helper functions for deserialization
    std::string read_string(std::ifstream& in);
    Atom read_atom(std::ifstream& in);
    Expr read_expr(std::ifstream& in);


public:
    pcb(uint32_t id) : pcb_id(id) {}

    // Getters
    uint32_t get_id() const { return pcb_id; }
    const std::vector<std::variant<uint16_t, Expr>>& get_program() const { return program; }

    // Modifiers
    void add_instruction(const std::variant<uint16_t, Expr>& instruction) {
        program.push_back(instruction);
    }

    // Store the PCB's program to a shared file.
    void store();

    // Load the PCB's program from the shared file.
    bool load();
};

#endif // PCB_HPP
