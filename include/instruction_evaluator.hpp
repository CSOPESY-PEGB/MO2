#ifndef OSEMU_INSTRUCTION_EVALUATOR_H_
#define OSEMU_INSTRUCTION_EVALUATOR_H_

#include <string>
#include <vector>
#include <memory>
#include <variant>
#include <iostream>
#include <unordered_map>
#include "instruction_parser.hpp"
#include "process_control_block.hpp"

namespace osemu {

    class PCB;
    struct PageTableEntry;

    class InstructionEvaluator {
    private:
        //for now, we won't page this into separate blocks first but we will soon.
        //std::vector<uint8_t>& heap_memory; 
        //we replace heap_memory for the page table
        std::vector<PageTableEntry>& page_table;

        //here we store string(variable name):address(logical memory address, starts from the top of heap_memory) 
        std::unordered_map<std::string, uint16_t>& symbol_table; 
        //output log
        std::vector<std::string>& output_log;
        std::string& process_name;

        size_t mem_per_frame;
        uint16_t heap_end;

        //helper functions for dealing with memory
        uint16_t get_or_create_variable_address(const std::string& var_name);
        uint16_t read_u16_from_heap(uint16_t address);
        void write_u16_to_heap(uint16_t address, uint16_t value);
        void write_u16_to_mem(uint16_t address, uint16_t value);

    public:
        InstructionEvaluator(std::vector<PageTableEntry>& page_table, 
            std::unordered_map<std::string, uint16_t>& symbol_table, 
            std::vector<std::string>& output_log, 
            std::string& process_name, size_t mem_per_frame);

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
        void handle_read(const std::string& var_name, const Atom& rhs);
        void handle_write(const Atom& address, const Atom& rhs);

        void clear_variables();
        void dump_variables() const;
        const std::vector<std::string>& get_output_log() const { return output_log; }
        void clear_output_log() { output_log.clear(); }
    };
}


#endif
