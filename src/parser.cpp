#include "parser.hpp"

#include <sstream>

namespace osemu {

std::vector<std::string> ParseTokens(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;

  while (iss >> token) {
    //if token starts with a ", start another chain until find a token that ends with "
    if(token.starts_with('"')){
      std::string subtoken;
      while(iss >> subtoken){
        token.append(" " + subtoken);
        if(subtoken.ends_with('"') && !subtoken.ends_with("\\\"")){
          break;
        }
      }
    }
    tokens.push_back(token);
  }

  return tokens;
}

}
