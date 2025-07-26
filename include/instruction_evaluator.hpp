#ifndef OSEMU_INSTRUCTION_EVALUATOR__H_
#define OSEMU_INSTRUCTION_EVALUATOR_H_

#include <string>
#include <vector>
#include <memory>
#include <variant>
#include <iostream>
#include <unordered_map>
#include "instruction_parser.hpp"

namespace osemu {
    class InstructionEvaluator {
    private:
        std::unordered_map<std::string, uint16_t> variables;
        std::vector<std::string> output_log;

    public:
        InstructionEvaluator();

        uint16_t resolve_atom_value(const Atom& atom);
        std::string print_atom_to_string(const Atom& atom);

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
}


#endif
