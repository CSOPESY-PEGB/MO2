#ifndef OSEMU_PARSER_H_
#define OSEMU_PARSER_H_

#include <string>
#include <vector>

namespace osemu {

std::vector<std::string> ParseTokens(const std::string& line);

}  

#endif  
