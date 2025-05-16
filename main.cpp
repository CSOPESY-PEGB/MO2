#include <iostream>
#include <string>
#include <unordered_map>
#include <stdexcept>

enum class Commands {
    Initialize,
    Screen,
    SchedulerTest,
    SchedulerStop,
    ReportUtil,
    Clear,
    Exit
};

// TODO: Implement using magic_enum in actual MP. ts pmo frfr mb gng🥀🥀
using CommandMap = std::unordered_map<std::string_view, Commands>;
static const CommandMap cmd_map = {
    {"initialize",    Commands::Initialize},
    {"screen",        Commands::Screen},    
    {"scheduler-test",Commands::SchedulerTest},
    {"scheduler-stop",Commands::SchedulerStop},
    {"report-util",   Commands::ReportUtil},
    {"clear",         Commands::Clear},
    {"exit",          Commands::Exit},
};

Commands from_str(const std::string_view cmd) {
    const auto it = cmd_map.find(cmd);
    if (it == cmd_map.end()) {
        throw std::invalid_argument("Unknown command: " + std::string(cmd));
    }
    return it->second;
}

// TODO: Replace with calls to initscr if using ncurses.
void console_prompt() {
    
    std::cout << R"(
░▒▓███████▓▒░░▒▓████████▓▒░▒▓██████▓▒░░▒▓███████▓▒░
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░     ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░     ░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░
░▒▓███████▓▒░░▒▓██████▓▒░░▒▓█▓▒▒▓███▓▒░▒▓███████▓▒░
░▒▓█▓▒░      ░▒▓█▓▒░     ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░
░▒▓█▓▒░      ░▒▓█▓▒░     ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░
░▒▓█▓▒░      ░▒▓████████▓▒░▒▓██████▓▒░░▒▓█▓▒░
)"; 
    std::cout << "\e[1;32mHello, Welcome to PEGP Command line! \e[0m " << std::endl;
    std::cout << "\e[3;33mType 'exit' to quit, 'clear' to clear the screen.\e[0m" << std::endl;

}



int main() {
    std::string input;
    console_prompt();

    while (std::cout << "~ ", std::cin >> input) {
        // Initialize | SchedulerTest | Screen | SchedulerStop | ReportUtil.
        // u guys could refactor this sanay lang talaga ako sa matching pattern
        // sa rust hehe
        try {
            switch (Commands cmd = from_str(input)) {
                case Commands::Initialize:
                case Commands::SchedulerTest:
                case Commands::Screen:
                case Commands::SchedulerStop:
                case Commands::ReportUtil:
                    std::cout << input << " command recognized. Doing something." << std::endl;
                    break;

                case Commands::Clear:
                    system("clear"); // TODO: Implement cls/clear
                    console_prompt();
                    break;

                case Commands::Exit:
                    return 0;
            }
        } catch (const std::invalid_argument& e) {
            std::cerr << e.what() << std::endl;
        }
    }

    return 0;
}
