#include <iostream>
#include <memory>
#include <string>

#include "commands.hpp"
#include "config.hpp"
#include "console.hpp"
#include "dispatcher.hpp"
#include "parser.hpp"
#include "scheduler.hpp"

int main() {
  using namespace osemu;

  Config cfg;
  Scheduler scheduler;
  console_prompt();

  std::string line;
  while (std::cout << "~ " << std::flush && std::getline(std::cin, line)) {
    if (line.empty()) continue;

    auto tokens = ParseTokens(line);
    if (tokens.empty()) continue;

    try {
      Commands cmd = from_str(tokens.front());
      tokens.erase(tokens.begin());
      dispatch(cmd, tokens, cfg, scheduler);

      if (cmd == Commands::Exit) {
        break;
      }
    } catch (const std::invalid_argument& ex) {
      std::cerr << "Error: " << ex.what() << '\n';
    } catch (const std::exception& ex) {
      std::cerr << "An unexpected error occurred: " << ex.what() << '\n';
    }
  }

  std::cout << "Emulator has shut down cleanly." << std::endl;
  return 0;
}