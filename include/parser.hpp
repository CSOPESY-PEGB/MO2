#ifndef OSEMU_PARSER_H_
#define OSEMU_PARSER_H_

#include <string>
#include <vector>

namespace osemu {

std::vector<std::string> parse_tokens(const std::string& line);

}  // namespace osemu

#endif  // OSEMU_PARSER_H_
