#include "screen.hpp"
#include "parser.hpp"
#include "commands.hpp"
#include "console.hpp"

#include <iostream>
#include <unordered_set>
#include <vector>
#include <string>

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
    PCB pcb(process_name, /* initial priority or status */ 100);
    try {
        if (!processes.insert(pcb).second) {
            std::cerr << "Process already exists: " << process_name << "\n";
            return false;
        }
        std::cout << "Process " << pcb.processName << " created\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error creating process: " << e.what() << "\n";
        return false;
    }
}

void screen(std::vector<std::string>& args,
            std::unordered_set<PCB>& processes,
            bool& screenSession)
{
    if (args.size() != EXPECTED_ARGS_COUNT) {
        display_usage();
        return;
    }

    const ScreenCommand cmd = parse_command(args[0]);
    const std::string& name = args[1];

    switch (cmd) {
      case ScreenCommand::Start:
        create_process(name, processes);
        break;

      case ScreenCommand::Resume: {
        PCB probe(name, 0);
        auto it = processes.find(probe);
        if (it == processes.end()) {
            std::cout << "Couldn't find process named: " << name << "\n";
            break;
        }

        // Found it, enter interactive session
        screenSession = true;
        std::cout << "\x1b[2J\x1b[H";       // clear screen + home
        const PCB& pcb = *it;
        std::cout << "Resumed Process: " << pcb.status() << "\n";

        std::string line;
        while (screenSession && (std::cout << "~ ") && std::getline(std::cin, line)) {
            auto tokens = parse_tokens(line);
            if (tokens.empty()) continue;

            try {
                Commands c = from_str(tokens.front());
                tokens.erase(tokens.begin());

                if (c == Commands::Exit) {
                    std::cout << "Exiting session for process: " << pcb.processName << "\n";
                    screenSession = false;
                    std::cout << "\x1b[2J\x1b[H";   // clear before returning to prompt
                    console_prompt();
                } else {
                    // handle other in‐session commands here
                    std::cout << "Unhandled command in screen session.\n";
                }
            } catch (const std::exception& ex) {
                std::cerr << ex.what() << "\n";
            }
        }
        break;
      }

      default:
        display_usage();
        break;
    }
}
