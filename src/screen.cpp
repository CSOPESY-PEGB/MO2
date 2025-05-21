#include "screen.hpp"
#include <iostream>
#include <unordered_set>
#include <vector>

enum class ScreenCommand {
    Resume,
    Start,
    Unknown
};

static constexpr int EXPECTED_ARGS_COUNT = 2;

void display_usage() {
    std::cout << "Usage:\n"
              << "  screen -r <name>   Resume and display the named screen session\n"
              << "  screen -s <name>   Start a new screen session with the given name\n";
}

ScreenCommand parse_command(const std::string& cmd) {
    if (cmd == "-r") return ScreenCommand::Resume;
    if (cmd == "-s") return ScreenCommand::Start;
    return ScreenCommand::Unknown;
}

bool create_process(const std::string& process_name, std::unordered_set<PCB>& processes) {
    const auto process = PCB(process_name, 0);
    try {
        if (!processes.insert(process).second) {
            std::cerr << "Process already exists\n";
            return false;
        }
        std::cout << "Process " << process.processName << " created\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error creating process: " << e.what() << '\n';
        return false;
    }
}

bool find_process_status(const std::string& process_name, const std::unordered_set<PCB>& processes) {
    const PCB search_process(process_name, 0);
    auto it = processes.find(search_process);
    
    if (it != processes.end()) {
        std::cout << "Found Process: " << it->status() << '\n';
        return true;
    }
    
    std::cout << "Couldn't find process named: " << process_name << '\n';
    return false;
}

bool screen(const std::vector<std::string>& args, std::unordered_set<PCB>& processes) {
    if (args.size() != EXPECTED_ARGS_COUNT) {
        display_usage();
        return false;
    }

    const auto command = parse_command(args[0]);
    const auto& process_name = args[1];

    switch (command) {
        case ScreenCommand::Start:
            return create_process(process_name, processes);
        case ScreenCommand::Resume:
            return find_process_status(process_name, processes);
        default:
            display_usage();
            return false;
    }
}