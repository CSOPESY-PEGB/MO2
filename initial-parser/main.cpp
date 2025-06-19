#include "parser.h"
#include "eval.h"
#include <fstream>
#include <sstream>

void print_atom(const Atom& atom) {
    std::cout << atom.to_string();
}

void print_expr(const Expr& expr, int indent = 0) {
    std::string spaces(indent * 2, ' ');
    
    switch (expr.type) {
        case Expr::DECLARE:
            std::cout << spaces << "DECLARE(" << expr.var_name << ", ";
            print_atom(*expr.atom_value);
            std::cout << ")";
            break;
        case Expr::CALL:
            std::cout << spaces << expr.var_name << "(";
            print_atom(*expr.atom_value);
            std::cout << ")";
            break;
        case Expr::ADD:
            std::cout << spaces << "ADD(" << expr.var_name << ", ";
            print_atom(*expr.lhs);
            std::cout << ", ";
            print_atom(*expr.rhs);
            std::cout << ")";
            break;
        case Expr::SUB:
            std::cout << spaces << "SUB(" << expr.var_name << ", ";
            print_atom(*expr.lhs);
            std::cout << ", ";
            print_atom(*expr.rhs);
            std::cout << ")";
            break;
        case Expr::FOR:
            std::cout << spaces << "FOR([";
            for (size_t i = 0; i < expr.body.size(); ++i) {
                if (i > 0) std::cout << ", ";
                print_expr(expr.body[i], 0);
            }
            std::cout << "], ";
            print_atom(*expr.n);
            std::cout << ")";
            break;
        default:
            std::cout << spaces << "UNKNOWN";
            break;
    }
}

int main(int argc, char* argv[]) {
    std::string filename = "input.opesy";
    if (argc > 1) {
        filename = argv[1];
    }
    
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string input = buffer.str();
    
    std::vector<Expr> program;
    ParseResult result = Parser::parse_program(input, program);
    
    if (!result.success) {
        std::cerr << "Parse error: " << result.error_msg << std::endl;
        std::cerr << "Remaining input: " << result.remaining << std::endl;
        return 1;
    }
    
    std::cout << "Parsed program:" << std::endl;
    for (const auto& expr : program) {
        print_expr(expr);
        std::cout << std::endl;
    }
    
    std::cout << "\nExecuting program:" << std::endl;
    try {
        Evaluator evaluator;
        evaluator.evaluate_program(program);
        std::cout << "\nProgram execution completed successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Runtime error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}