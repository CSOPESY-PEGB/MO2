#include "screen.hpp"
#include "parser.hpp"
#include "commands.hpp"
#include "console.hpp"
#include <iostream>
#include <unordered_set>
#include <vector>

void screen(std::vector<std::string> &args, std::unordered_set<PCB> &procs, bool& screenSession) {
  if (args.size() != 2) {
    std::cout << "Usage:\n"
              << "  screen -r <name>   Resume and display the named screen session\n"
              << "  screen -s <name>   Start a new screen session with the given name\n";
    return;
  }
  if (args[0] == "-s") {
    const auto pcb = PCB(args[1], 100);
    try {
      procs.insert(pcb);
      std::cout << "PCB " << pcb.processName << " created\n";
    } catch (const std::exception& e) {
      std::cerr << e.what() << '\n';
    }
  } else if (args[0] == "-r") {
    PCB searchPCB(args[1], 0);
    auto it = procs.find(searchPCB);

    if (it != procs.end()) {
      screenSession = true;
      std::cout << "\x1b[2J\x1b[H";
      const PCB& pcb = *it;
      std::cout << "Found Process: " << pcb.status() << '\n';

      std::string line;

      while (screenSession && std::cout << "~ " && std::getline(std::cin, line)) {
        //
          auto tokens = parse_tokens(line);
          if (tokens.empty()) continue;

          try {
            Commands cmd = from_str(tokens.front());
            tokens.erase(tokens.begin());
            if (cmd == Commands::Exit) { // If the user types "exit", leave the session
              std::cout << "Exiting session for process: " << pcb.processName << '\n';
              screenSession = false; // Exit the loop
              std::cout << "\x1b[2J\x1b[H";
              console_prompt();
            } else {
              std::cout << "Unhandled command in screen session.\n";
            }
          } catch (const std::exception& ex) {
            std::cerr << ex.what() << '\n';
          }
        //
      }

    } else {
        std::cout << "Couldn't find process named: " << args[1] << '\n';
    }
}
}


