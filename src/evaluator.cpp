#include "evaluator.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>

namespace osemu {

void Evaluator::evaluate(const Expr& expr) {
    // Implementation of instruction evaluation will go here
}

void Evaluator::handle_print(const Atom& atom, const std::string& processName) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);

    std::stringstream ss;
    ss << "(" << std::put_time(&tm, "%m/%d/%Y %I:%M:%S%p") << ") ";

    if (atom.type == Atom::STRING) {
        ss << "\"" << atom.string_value << "\"";
    } else if (atom.type == Atom::NAME) {
        if (variables_.count(atom.string_value)) {
            ss << "\"" << variables_[atom.string_value] << "\"";
        } else {
            ss << "Variable " << atom.string_value << " not found";
        }
    }
    output_log_.push_back(ss.str());
}

const std::vector<std::string>& Evaluator::get_output_log() const {
    return output_log_;
}

} // namespace osemu
