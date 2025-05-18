#include "screen.hpp"

#include <iostream>
#include <unordered_set>
#include <vector>

void screen(std::vector<std::string> &args, std::unordered_set<PCB> &procs) {
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
        const PCB& pcb = *it;
        std::cout << "Found Process: " << pcb.status() << '\n';
    } else {
        std::cout << "Couldn't find process named: " << args[1] << '\n';
    }
}
}