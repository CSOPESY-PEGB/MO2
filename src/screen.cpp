#include "screen.hpp"

#include <iostream>
#include <string>
#include <vector>

#include "scheduler.hpp"
#include "process_control_block.hpp"
namespace osemu {

namespace {

enum class ScreenCommand {
    Resume,
    Start,
    Unknown
};

constexpr int EXPECTED_ARGS_COUNT = 2;

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

void create_process(const std::string& process_name, Scheduler& scheduler) {
    try {
        // Create a shared_ptr, as required by our system
        auto pcb = std::make_shared<PCB>(process_name, 100);
        scheduler.submit_process(pcb);
    } catch (const std::exception& e) {
        std::cerr << "Error creating process: " << e.what() << "\n";
    }
}

}

    void screen(std::vector<std::string>& args, Scheduler& scheduler) {
    // The command should be "screen -s <name>", so we expect 2 arguments after "screen"
    if (args.size() != 2) {
        display_usage();
        return;
    }

    const ScreenCommand cmd = parse_command(args[0]);
    const std::string& name = args[1];

    switch (cmd) {
        case ScreenCommand::Start:
            create_process(name, scheduler);
            break;

        case ScreenCommand::Unknown:
        default:
            std::cout << "Unknown screen command: " << args[0] << "\n";
            display_usage();
            break;
    }
}

}
