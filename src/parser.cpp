#include "parser.hpp"

#include <sstream>

namespace osemu {

std::vector<std::string> ParseTokens(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;

  while (iss >> token) {
    tokens.push_back(token);
  }

  return tokens;
}

}
