#include "dispatcher.hpp"

#include <cstdlib>
#include <iostream>

#include "console.hpp"
#include "screen.hpp"

namespace osemu {

void dispatch(Commands cmd, std::vector<std::string>& args, Config& cfg,
              std::unordered_set<PCB>& procs, bool& screenSession) {
    switch (cmd) {
        case Commands::Initialize:
            try {
                cfg = Config::fromFile(args.empty() ? "config.txt" : args[0]);
            } catch (const std::exception& e) {
                std::cerr << "Error initializing config: " << e.what() << '\n';
            }
            break;
            
        case Commands::SchedulerTest:
        case Commands::Screen:
            screen(args, procs, screenSession);
            break;
            
        case Commands::SchedulerStop:
        case Commands::ReportUtil:
            std::cout << "TODO: Implement command " << static_cast<int>(cmd) << '\n';
            break;

        case Commands::Clear:
            std::cout << "\x1b[2J\x1b[H";
            console_prompt();
            break;

        case Commands::Exit:
            std::exit(0);
    }
}

}
