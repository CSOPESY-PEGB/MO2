#include "screen.hpp"

#include <iostream>
#include <unordered_set>
#include <vector>

void DisplayScreenUsage() {
  std::cout
      << "Usage:\n"
      << "  screen -r <name>   Resume and display the named screen session\n"
      << "  screen -s <name>   Start a new screen session with the given "
         "name\n";
}
void CreatePcb(std::vector<std::string>& args, std::unordered_set<PCB>& procs) {
  const auto pcb = PCB(args[1], 100);
  try {
    procs.insert(pcb);
    std::cout << "PCB " << pcb.processName << " created\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }
}
void FindProcessStatus(std::vector<std::string>& args,
                       std::unordered_set<PCB>& procs) {
  PCB searchPCB(args[1], 0);
  auto it = procs.find(searchPCB);

  if (it != procs.end()) {
    const PCB& pcb = *it;
    std::cout << "Found Process: " << pcb.status() << '\n';
  } else {
    std::cout << "Couldn't find process named: " << args[1] << '\n';
  }
}
void screen(std::vector<std::string>& args, std::unordered_set<PCB>& procs) {
  if (args.size() != 2) {
    DisplayScreenUsage();
    return;
  }
  if (args[0] == "-s") {
    CreatePcb(args, procs);
  } else if (args[0] == "-r") {
    FindProcessStatus(args, procs);
  }
}