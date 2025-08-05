#include "screen.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "console.hpp"  
#include "instruction_generator.hpp"
#include "process_control_block.hpp"
#include "instruction_parser.hpp"
#include "scheduler.hpp"

namespace osemu {
namespace {

std::string unescape_string(const std::string& s) {
  std::string out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] == '\\' && i + 1 < s.length()) {
      out += s[i + 1];
      i++;
    } else {
      out += s[i];
    }
  }
  return out;
}

bool is_power_of_2_size_t(size_t value) {
  return value > 0 && (value & (value - 1)) == 0;
}

bool validate_memory_size(size_t memory_size) {
  if (memory_size < 64 || memory_size > 65536) {
    std::cout << "Invalid memory allocation: " << memory_size 
              << ". Must be between 64 and 65536 bytes." << std::endl;
    return false;
  }
  if (!is_power_of_2_size_t(memory_size)) {
    std::cout << "Invalid memory allocation: " << memory_size 
              << ". Must be a power of 2." << std::endl;
    return false;
  }
  return true;
}

std::shared_ptr<PCB> find_process(const std::string& process_name, Scheduler& scheduler) {
  return scheduler.find_process_by_name(process_name);
}

void view_process_screen(const std::string& process_name, Scheduler& scheduler) {
  std::shared_ptr<PCB> pcb = find_process(process_name, scheduler);

  if (!pcb) {
      std::cout << "Process '" << process_name << "' not found." << std::endl;
      return;
  }
  
  std::cout << "\x1b[2J\x1b[H"; // Clear screen
  std::string input_line;

  while (true) {
    pcb = find_process(process_name, scheduler);
    if (!pcb) {
        std::cout << "Process '" << process_name << "' has finished or been terminated." << std::endl;
        break;
    }

    std::cout << "--- Process Viewer: " << process_name << " ---\n";
    std::cout << "Status: " << pcb->status() << "\n\n";

    std::cout << "Logs:\n";
    const auto& logs = pcb->getExecutionLogs();
    if (logs.empty()) {
      std::cout << "(No logs yet)\n";
    } else {
      for (const auto& log : logs) {
        std::cout << "  " << log << "\n";
      }
    }
    std::cout << "\n(Type 'exit' to return, 'clear' to refresh)\n";
    std::cout << "process-viewer:\\> ";
    if (!std::getline(std::cin, input_line)) break; 

    if (input_line == "exit") break;
    if (input_line == "clear") {
      std::cout << "\x1b[2J\x1b[H";
      continue;
    }
  }

  std::cout << "\x1b[2J\x1b[H";
  console_prompt();
}

bool create_process(const std::string& process_name, size_t memory_size, Scheduler& scheduler, Config& config) {
  if (scheduler.find_process_by_name(process_name) != nullptr) {
    std::cerr << "Error: Process '" << process_name << "' already exists." << std::endl;
    return false;
  }
  if (!validate_memory_size(memory_size)) return false;

  InstructionGenerator generator;
  auto instructions = generator.generateRandomProgram(config.minInstructions, config.maxInstructions, process_name, memory_size, config.mem_per_frame);
  if (instructions.empty()) {
      std::cerr << "Error: Could not generate instructions for process '" << process_name << "'. Check memory/frame configuration." << std::endl;
      return false;
  }

  auto pcb = std::make_shared<PCB>(process_name, instructions, memory_size);
  scheduler.submit_process(pcb);
  std::cout << "Created process '" << process_name << "' with " << memory_size << " bytes of memory." << std::endl;
  return true;
}

void create_process_from_file(const std::string& filename, const std::string& process_name, size_t memory_size, Scheduler& scheduler) {
  if (scheduler.find_process_by_name(process_name) != nullptr) {
    std::cerr << "Error: Process '" << process_name << "' already exists." << std::endl;
    return;
  }
   if (!validate_memory_size(memory_size)) return;

  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string input = buffer.str();

  std::vector<Expr> program;
  ParseResult result = InstructionParser::parse_program(input, program);

  if (!result.success) {
    std::cerr << "Parse error: " << result.error_msg << std::endl;
    std::cerr << "At: " << result.remaining << std::endl;
    return;
  }

  auto pcb = std::make_shared<PCB>(process_name, program, memory_size);
  scheduler.submit_process(pcb);
  std::cout << "Created process '" << process_name << "' from file '" << filename
            << "' with " << program.size() << " instructions and " << memory_size << "B memory." << std::endl;
}
//
// void create_process_from_string(const std::string& process_name, size_t memory, const std::string& instructions_str, Scheduler& scheduler) {
//     if (scheduler.find_process_by_name(process_name) != nullptr) {
//         std::cerr << "Error: Process '" << process_name << "' already exists." << std::endl;
//         return;
//     }
//     if (!validate_memory_size(memory)) return;
//
//     std::string final_program_string_for_parser;
//     std::string initial_string = instructions_str;
//
//     if (initial_string.length() > 1 && initial_string.front() == '"' && initial_string.back() == '"') {
//         initial_string = initial_string.substr(1, initial_string.length() - 2);
//     }
//
//     std::string unescaped_string = unescape_string(initial_string);
//     std::stringstream ss(unescaped_string);
//     std::string instruction_line;
//
//     while (std::getline(ss, instruction_line, ';')) {
//         auto first = instruction_line.find_first_not_of(" \t\n\r");
//         if (std::string::npos == first) continue;
//         auto last = instruction_line.find_last_not_of(" \t\n\r");
//         instruction_line = instruction_line.substr(first, (last - first + 1));
//
//         auto cmd_end = instruction_line.find_first_of(" \t(");
//         std::string command = instruction_line.substr(0, cmd_end);
//
//         if (command == "DECLARE" || command == "ADD" || command == "SUB" || command == "READ" || command == "WRITE" || command == "SLEEP") {
//             std::string args_part = instruction_line.substr(command.length());
//             auto args_start = args_part.find_first_not_of(" \t");
//             if(args_start != std::string::npos) {
//                 args_part = args_part.substr(args_start);
//                 std::replace(args_part.begin(), args_part.end(), ' ', ',');
//                 final_program_string_for_parser += command + "(" + args_part + ")\n";
//             }
//         } else {
//             final_program_string_for_parser += instruction_line + "\n";
//         }
//     }
//
//     std::vector<Expr> program;
//     ParseResult result = InstructionParser::parse_program(final_program_string_for_parser, program);
//
//     if (!result.success) {
//         std::cerr << "Parse error in custom command: " << result.error_msg << std::endl;
//         std::cerr << "On line: " << result.remaining << std::endl;
//         std::cerr << "--- Processed Program String ---\n" << final_program_string_for_parser << "------------------------------\n";
//         return;
//     }
//
//     auto pcb = std::make_shared<PCB>(process_name, program, memory);
//     scheduler.submit_process(pcb);
//     std::cout << "Created custom process '" << process_name << "' with " << memory << " bytes." << std::endl;
// }


void create_process_from_string(const std::string& process_name, size_t memory, const std::string& instructions_str, Scheduler& scheduler) {
    if (scheduler.find_process_by_name(process_name) != nullptr) {
        std::cerr << "Error: Process '" << process_name << "' already exists." << std::endl;
        return;
    }
    if (!validate_memory_size(memory)) return;

    std::string initial_string = instructions_str;

    // Remove outer quotes from the entire command string if they exist
    if (initial_string.length() > 1 && initial_string.front() == '"' && initial_string.back() == '"') {
        initial_string = initial_string.substr(1, initial_string.length() - 2);
    }

    // Unescape characters like \" to "
    std::string unescaped_string = unescape_string(initial_string);

    // This will hold the final string that the parser can understand
    std::string final_program_string_for_parser;

    // Split the full command string by semicolons to get individual instructions
    std::stringstream ss(unescaped_string);
    std::string instruction_line;
    while (std::getline(ss, instruction_line, ';')) {
        // Trim leading/trailing whitespace from the instruction
        auto first = instruction_line.find_first_not_of(" \t\n\r");
        if (std::string::npos == first) continue; // Skip empty instructions
        auto last = instruction_line.find_last_not_of(" \t\n\r");
        instruction_line = instruction_line.substr(first, (last - first + 1));

        // Find the command part (e.g., "DECLARE", "WRITE")
        auto cmd_end = instruction_line.find_first_of(" \t");
        std::string command = instruction_line.substr(0, cmd_end);

        // Check if the command needs reformatting
        if (command == "DECLARE" || command == "ADD" || command == "SUB" || command == "READ" || command == "WRITE" || command == "SLEEP") {
            // This is a simple command that needs to be wrapped in parentheses
            // and have its spaces converted to commas.
            std::string args_part = instruction_line.substr(command.length());

            // Trim whitespace from the arguments part
            auto args_start = args_part.find_first_not_of(" \t");
            if(args_start != std::string::npos) {
                args_part = args_part.substr(args_start);
                // Replace spaces with commas
                std::replace(args_part.begin(), args_part.end(), ' ', ',');
                final_program_string_for_parser += command + "(" + args_part + ")\n";
            } else {
                // Should not happen for these commands, but handle it
                final_program_string_for_parser += command + "()\n";
            }
        } else {
            // This is a complex command like PRINT with its own parentheses.
            // Trust that it's already in the correct format and pass it through.
            final_program_string_for_parser += instruction_line + "\n";
        }
    }

    std::vector<Expr> program;
    ParseResult result = InstructionParser::parse_program(final_program_string_for_parser, program);

    if (!result.success) {
        std::cerr << "Parse error in custom command: " << result.error_msg << std::endl;
        std::cerr << "On line: " << result.remaining << std::endl;
        std::cerr << "--- Processed Program String ---\n" << final_program_string_for_parser << "------------------------------\n";
        return;
    }

    auto pcb = std::make_shared<PCB>(process_name, program, memory);
    scheduler.submit_process(pcb);
    std::cout << "Created custom process '" << process_name << "' with " << memory << " bytes." << std::endl;
}

enum class ScreenCommand { Start, Resume, List, File, Custom, Unknown };

void display_usage() {
  std::cout
      << "Usage:\n"
      << "  screen -s <name> <memory_size>         Start a new random process.\n"
      << "  screen -r <name>                       View a running/finished process.\n"
      << "  screen -ls                             List all processes (now 'process-smi').\n"
      << "  screen -f <file> <name> <memory_size>  Load process from a .opesy file.\n"
      << "  screen -c <name> <memory_size> \"...\"   Create a process with custom instructions.\n" ;
}

ScreenCommand parse_command(const std::string& cmd) {
  if (cmd == "-s") return ScreenCommand::Start;
  if (cmd == "-r") return ScreenCommand::Resume;
  if (cmd == "-ls") return ScreenCommand::List;
  if (cmd == "-f") return ScreenCommand::File;
  if (cmd == "-c") return ScreenCommand::Custom;
  return ScreenCommand::Unknown;
}

}  

void screen(std::vector<std::string>& args, Scheduler& scheduler, Config& config) {
  if (args.empty()) {
    display_usage();
    return;
  }

  const ScreenCommand cmd = parse_command(args[0]);
  switch (cmd) {
    case ScreenCommand::Start: {
      if (args.size() != 3) {
        std::cout << "Usage: screen -s <name> <memory_size>" << std::endl;
        return;
      }
      try {
        size_t mem_size = std::stoull(args[2]);
        create_process(args[1], mem_size, scheduler, config);
      } catch (const std::exception& e) {
        std::cerr << "Error: Invalid memory size '" << args[2] << "'. " << e.what() << std::endl;
      }
      break;
    }
    case ScreenCommand::Resume:
      if (args.size() != 2) {
        std::cout << "Usage: screen -r <name>" << std::endl;
        return;
      }
      view_process_screen(args[1], scheduler);
      break;

    case ScreenCommand::List:
      scheduler.print_status();
      break;

    case ScreenCommand::File: {
      if (args.size() != 4) {
        std::cout << "Usage: screen -f <filepath> <process_name> <memory_size>" << std::endl;
        return;
      }
      try {
        size_t mem_size  = std::stoull(args[3]);
        create_process_from_file(args[1], args[2], mem_size, scheduler);
      } catch (const std::exception& e) {
        std::cerr << "Error: Invalid memory size '" << args[3] << "'. " << e.what() << std::endl;
      }
      break;
    }

    case ScreenCommand::Custom: {
      if(args.size() != 4){
        std::cout << "Usage: screen -c <name> <memory_size> \"<instructions>\"" << std::endl;
        return;
      } 
      try {
        size_t mem = std::stoul(args[2]);
        create_process_from_string(args[1], mem, args[3], scheduler);
      } catch(const std::exception& e) {
        std::cout << "Error: Invalid memory size '" << args[2] << "'. " << e.what() << std::endl;
      }
      break;
    }
    case ScreenCommand::Unknown:
    default:
      std::cout << "Unknown screen command: " << args[0] << "\n";
      display_usage();
      break;
  }
}

}