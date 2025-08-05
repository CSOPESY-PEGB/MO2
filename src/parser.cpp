#include "parser.hpp"

#include <sstream>

namespace osemu {

std::vector<std::string> ParseTokens(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;
  bool in_quotes = false;
  std::string current_quoted_token;

  while(iss >> std::ws >> token) {
    if (!in_quotes) {
      if (token.front() == '"') {
        if (token.back() == '"' && token.length() > 1) {
          tokens.push_back(token);
        } else {
          in_quotes = true;
          current_quoted_token = token;
        }
      } else {
        tokens.push_back(token);
      }
    } else {
      current_quoted_token += " " + token;
      if (token.back() == '"' && (token.length() == 1 || token[token.length() - 2] != '\\')) {
        in_quotes = false;
        tokens.push_back(current_quoted_token);
        current_quoted_token.clear();
      }
    }
  }

  return tokens;
}

}