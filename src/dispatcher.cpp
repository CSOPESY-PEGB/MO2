// dispatcher.cpp
#include "dispatcher.hpp"

#include <iostream>

#include "console.hpp"
#include "screen.hpp"
#include "scheduler.hpp" // We need the full definition here to call its methods

namespace osemu {

    void dispatch(Commands cmd,
                  std::vector<std::string>& args,
                  Config& cfg,
                  Scheduler& scheduler) {
        switch (cmd) {
            case Commands::Initialize:
                try {
                    cfg = Config::fromFile(args.empty() ? "config.txt" : args[0]);
                } catch (const std::exception& e) {
                    std::cerr << "Error initializing config: " << e.what() << '\n';
                }
                break;

            case Commands::Screen:
                // The call to screen is now simpler, without screenSession
                screen(args, scheduler);
                break;

            case Commands::ScreenLs: // NEW: Handle the 'screen -ls' command
                scheduler.print_status();
                break;

            case Commands::SchedulerTest:
            case Commands::SchedulerStop:
            case Commands::ReportUtil:
                std::cout << "TODO: Implement command '" << static_cast<int>(cmd) << "'\n";
                break;

            case Commands::Clear:
                std::cout << "\x1b[2J\x1b[H";
                console_prompt();
                break;

            case Commands::Exit:
                // This case should be empty. The main loop handles the exit logic.
                break;
        }
    }

} // namespace osemu