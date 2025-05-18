#include "parser.hpp"

#include <sstream>

std::vector<std::string> parse_tokens(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> out;
  for (std::string tok; iss >> tok;) out.push_back(tok);
  return out;
}
