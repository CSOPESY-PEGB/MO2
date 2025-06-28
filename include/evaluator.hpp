#ifndef OSEMU_EVALUATOR_H_
#define OSEMU_EVALUATOR_H_

#include <string>
#include <vector>
#include <map>
#include "instruction_parser.hpp"

namespace osemu {

class Evaluator {
public:
    void evaluate(const Expr& expr);
    void handle_print(const Atom& atom, const std::string& processName);
    const std::vector<std::string>& get_output_log() const;

private:
    std::map<std::string, uint16_t> variables_;
    std::vector<std::string> output_log_;
};

} 

#endif 
