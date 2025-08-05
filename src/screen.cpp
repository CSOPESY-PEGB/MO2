
#include "screen.hpp"

#include <atomic>  
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include <thread>  
#include <vector>
#include <chrono>

#include "console.hpp"  
#include "instruction_generator.hpp"
#include "process_control_block.hpp"
#include "instruction_parser.hpp"
#include "scheduler.hpp"

namespace osemu {
namespace {

std::string unescape_string(std::string s) {
  std::string out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] == '\\' && i + 1 < s.length()) {
      // We just skip the backslash and take the next character literally
      out += s[i + 1];
      i++; // Increment i an extra time to skip the escaped character
    } else {
      out += s[i];
    }
  }
  return out;
}
bool is_power_of_2(size_t value) {
  return value > 0 && (value & (value - 1)) == 0;
}

bool validate_memory_size(size_t memory_size) {
  if (memory_size < 64 || memory_size > 65536) {
    std::cout << "Invalid memory allocation: " << memory_size 
              << ". Must be between 64 and 65536 bytes." << std::endl;
    return false;
  }
  if (!is_power_of_2(memory_size)) {
    std::cout << "Invalid memory allocation: " << memory_size 
              << ". Must be a power of 2." << std::endl;
    return false;
  }
  return true;
}

std::shared_ptr<PCB> find_process(const std::string& process_name,
                                  Scheduler& scheduler) {
  if (scheduler.find_process_by_name(process_name) != nullptr) {
    return scheduler.find_process_by_name(process_name);
  }

  return nullptr;
}


void view_process_screen(const std::string& process_name, Scheduler& scheduler) {
  // scheduler gets the process
  try {
    std::shared_ptr<PCB> pcb = find_process(process_name, scheduler);

    if (!pcb) {
      // Check if process was terminated due to memory access violation
      // This would require tracking terminated processes with error reasons
      std::cout << "Process " << process_name << " not found." << std::endl;
      return;
    }
    std::cout << "\x1b[2J\x1b[H";


    std::string input_line;
    while (true) {
      std::cout << "Process name: " << process_name << std::endl;
      std::cout << "ID: "  << pcb->processID<< std::endl;
      std::cout << "Logs:" << std::endl;

      const auto& logs = pcb->getExecutionLogs();
      if (logs.empty()) {
        std::cout << "(No logs yet)" << std::endl;
      } else {
        for (const auto& log : logs) {
          std::cout << log << std::endl;
        }
      }

      std::cout << std::endl;
      std::cout << "Current instruction line: "<< pcb->currentInstruction << std::endl;
      std::cout << "Lines of code: " << pcb-> totalInstructions << std::endl;
      std::cout << std::endl;

      std::cout << "root:\\> ";
      if (!std::getline(std::cin, input_line)) {
        break;
      }

      if (input_line == "exit") {
        break;
      } else if (input_line == "process-smi") {

        std::cout << "\x1b[2J\x1b[H";
        continue;
      } else {
        std::cout << "Unknown command: " << input_line << std::endl;
        std::cout << "Available commands: process-smi, exit" << std::endl;
      }
    }


    std::cout << "\x1b[2J\x1b[H";
    console_prompt();
  }
  catch (const std::exception& e) {
    std::cout << e.what() << std::endl;

  }

}

bool create_process(const std::string& process_name, size_t memory_size, Scheduler& scheduler, Config& config) {
  if (scheduler.find_process_by_name(process_name) != nullptr) {
    std::cerr << "Error: Process '" << process_name << "' already exists." << std::endl;
    return false;
  }
  if (!validate_memory_size(memory_size)) {
    return false;
  }
  // MemoryManager* mm = scheduler.get_memory_manager();
  // if (!mm) {
  //   std::cerr << "Error: Memory Manager is not initialized." << std::endl;
  //   return false;
  // }
  //
  // uint32_t frames_needed = (memory_size + config.mem_per_frame - 1) / config.mem_per_frame;
  // uint32_t frames_available = mm->get_free_frame_count();
  //
  // if (frames_needed > frames_available) {
  //   std::cerr << "Error: Not enough memory to create process '" << process_name
  //             << "'. Required frames: " << frames_needed
  //             << ", Available frames: " << frames_available << std::endl;
  //   return false; // REJECT THE PROCESS
  // }

  // if (memory_size > config.max_overall_mem) {
  //   std::cerr << std::format("Error: Process memory {}B exceeds system limit {}B.", memory_size, config.max_overall_mem) << std::endl;
  //   return false;
  // }

  InstructionGenerator generator;
  auto instructions = generator.generateRandomProgram(config.minInstructions, config.maxInstructions, process_name, memory_size, config.mem_per_frame);
  auto pcb = std::make_shared<PCB>(process_name, instructions, memory_size);

  scheduler.submit_process(pcb);
  std::cout << "Created process '" << process_name << "' with " << memory_size << " bytes of memory." << std::endl;
  return true;
}




void create_process_from_file(const std::string& filename,
                              const std::string& process_name,
                              size_t memory_size, Scheduler& scheduler) {
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
    std::cerr << "Remaining input: " << result.remaining << std::endl;
    return;
  }


  auto pcb = std::make_shared<PCB>(process_name, program, memory_size);

  std::cout << "Created process '" << process_name << "' from file '" << filename
            << "' with " << program.size() << " instructions." << std::endl;

  scheduler.submit_process(pcb);
}

// std::string unescapeQuotes(const std::string& str) {
//     std::string result;
//     bool inEscape = false;
//
//     for (size_t i = 0; i < str.length(); ++i) {
//         if (str[i] == '\\' && i + 1 < str.length() && str[i + 1] == '"') {
//             // Skip the escape sequence for a quote
//             result += '"';
//             ++i; // Skip the next character (the quote)
//         } else {
//             result += str[i];
//         }
//     }
//
//     return result;
// }
//
// std::string transformInstructions(const std::string& input) {
//     std::string result;
//     std::string tempInput = input;
//
//     // Remove the starting and ending double quotes
//     if (!tempInput.empty() && tempInput[0] == '"') {
//         tempInput = tempInput.substr(1, tempInput.size() - 2);  // Remove quotes
//     }
//
//     // Unescape all double quotes
//     tempInput = unescapeQuotes(tempInput);
//
//     // Split the input by semicolons and convert to proper format
//     std::istringstream stream(tempInput);
//     std::string instruction;
//
//     while (std::getline(stream, instruction, ';')) {
//         // Trim whitespace
//         size_t start = instruction.find_first_not_of(" \t");
//         if (start == std::string::npos) continue; // Skip empty instructions
//         size_t end = instruction.find_last_not_of(" \t");
//         instruction = instruction.substr(start, end - start + 1);
//
//         if (instruction.empty()) continue;
//
//         // Parse different instruction types
//         if (instruction.find("DECLARE ") == 0) {
//             // Format: DECLARE varA 10 -> DECLARE(varA, 10)
//             std::istringstream iss(instruction);
//             std::string cmd, var, value;
//             iss >> cmd >> var >> value;
//             result += cmd + "(" + var + ", " + value + ")\n";
//         }
//         else if (instruction.find("ADD ") == 0) {
//             // Format: ADD varA varA varB -> ADD(varA, varA, varB)
//             std::istringstream iss(instruction);
//             std::string cmd, var1, var2, var3;
//             iss >> cmd >> var1 >> var2 >> var3;
//             result += cmd + "(" + var1 + ", " + var2 + ", " + var3 + ")\n";
//         }
//         else if (instruction.find("SUB ") == 0) {
//             // Format: SUB varA varB varC -> SUB(varA, varB, varC)
//             std::istringstream iss(instruction);
//             std::string cmd, var1, var2, var3;
//             iss >> cmd >> var1 >> var2 >> var3;
//             result += cmd + "(" + var1 + ", " + var2 + ", " + var3 + ")\n";
//         }
//         else if (instruction.find("WRITE ") == 0) {
//             // Format: WRITE 0x500 varA -> WRITE(0x500, varA)
//             std::istringstream iss(instruction);
//             std::string cmd, addr, value;
//             iss >> cmd >> addr >> value;
//             result += cmd + "(" + addr + ", " + value + ")\n";
//         }
//         else if (instruction.find("READ ") == 0) {
//             // Format: READ varC 0x500 -> READ(varC, 0x500)
//             std::istringstream iss(instruction);
//             std::string cmd, var, addr;
//             iss >> cmd >> var >> addr;
//             result += cmd + "(" + var + ", " + addr + ")\n";
//         }
//         else if (instruction.find("PRINT(") == 0) {
//             // Already in correct format, just add newline
//             result += instruction + "\n";
//         }
//         else if (instruction.find("PRINT ") == 0) {
//             // Handle PRINT with space - this shouldn't happen in the test format but just in case
//             result += instruction + "\n";
//         }
//         else if (instruction.find("SLEEP ") == 0) {
//             // Format: SLEEP 1 -> SLEEP(1)
//             std::istringstream iss(instruction);
//             std::string cmd, value;
//             iss >> cmd >> value;
//             result += cmd + "(" + value + ")\n";
//         }
//         else {
//             // For any other format, try to parse as generic command
//             std::istringstream iss(instruction);
//             std::string cmd;
//             iss >> cmd;
//
//             result += cmd + "(";
//             std::string arg;
//             bool first = true;
//             while (iss >> arg) {
//                 if (!first) result += ", ";
//                 result += arg;
//                 first = false;
//             }
//             result += ")\n";
//         }
//     }
//
//     return result;
// }





void create_process_from_string(const std::string& process_name, size_t memory, const std::string& instructions_str, Config& config, Scheduler& scheduler) {
    // --- Step 1: Validation ---
    if (scheduler.find_process_by_name(process_name) != nullptr) {
        std::cerr << "Error: Process '" << process_name << "' already exists." << std::endl;
        return;
    }
    if (!validate_memory_size(memory)) {
        return;
    }
    MemoryManager* mm = scheduler.get_memory_manager();
    if (!mm) { std::cerr << "Error: Memory manager not ready.\n"; return; }
    uint32_t frames_needed = (memory + config.mem_per_frame - 1) / config.mem_per_frame;
    if (frames_needed > mm->get_free_frame_count()) {
        std::cerr << "Error: Not enough memory for process '" << process_name << "'.\n";
        return;
    }

    // --- Step 2: The CRITICAL Pre-processing Logic ---
    std::string final_program_string_for_parser;
    std::string initial_string = instructions_str;

    // Remove outer quotes
    if (initial_string.length() > 1 && initial_string.front() == '"' && initial_string.back() == '"') {
        initial_string = initial_string.substr(1, initial_string.length() - 2);
    }

    // Unescape the string to handle \"
    std::string unescaped_string = unescape_string(initial_string);

    std::stringstream ss(unescaped_string);
    std::string instruction_line;

    // Split by semicolons
    while (std::getline(ss, instruction_line, ';')) {
        // Trim whitespace
        size_t first = instruction_line.find_first_not_of(" \t\n\r");
        if (std::string::npos == first) continue;
        size_t last = instruction_line.find_last_not_of(" \t\n\r");
        instruction_line = instruction_line.substr(first, (last - first + 1));

        // Find the command keyword
        size_t cmd_end = instruction_line.find_first_of(" \t(");
        std::string command = instruction_line.substr(0, cmd_end);

        if (command == "DECLARE" || command == "ADD" || command == "SUB" || command == "READ" || command == "WRITE" || command == "SLEEP") {
            // It's a simple command that needs reformatting
            std::string args_part = instruction_line.substr(command.length());
            args_part = args_part.substr(args_part.find_first_not_of(" \t"));
            // Replace spaces in the argument list with commas
            std::replace(args_part.begin(), args_part.end(), ' ', ',');
            final_program_string_for_parser += command + "(" + args_part + ")\n";
        } else if (command == "PRINT") {
            // PRINT is already in the correct format, pass it through
            final_program_string_for_parser += instruction_line + "\n";
        }
    }

    // --- Step 3: Parsing ---
    std::vector<Expr> program;
    // Use the transformed string
    ParseResult result = InstructionParser::parse_program(final_program_string_for_parser, program);

    if (!result.success) {
        std::cerr << "Parse error in custom command: " << result.error_msg << std::endl;
        std::cerr << "On instruction part: " << result.remaining << std::endl;
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
      << "  screen -s <name> <memory_size>   Start a new process with the given name and memory size.\n"
      << "  screen -r <name>     View the real-time log of a running process.\n"
      << "  screen -ls           List all active processes.\n"
      << "  screen -f <file> <name>  Load process from .opesy file.\n"
      << "  screen -c <name> <process_memory_size> \"<instructions>\" Make a custom process with instructions\n" ;
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
  // parse command returns args after "screen" so args[0] = -s, -r, -c
  // maps to a ScreenCommand
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
        std::cerr << "Error: Invalid memory size '" << args[2] << "'." << std::endl;
      }
      break;
    }
    case ScreenCommand::Resume:
      if (args.size() != 2) {
        display_usage();
        return;
      }
      view_process_screen(args[1], scheduler);
      break;

    case ScreenCommand::List:
      scheduler.print_status();
      break;

    case ScreenCommand::File:
      if (args.size() != 3) {
        display_usage();
        return;
      }
    {
      size_t mem_size  = static_cast<size_t>(std::stoull(args[2]));
      create_process_from_file(args[1], args[2], mem_size, scheduler);
    }
      break;

    case ScreenCommand::Custom:
      size_t num;

      //error handling
      if(args.size() != 4){
        display_usage();
        return;
      } 

      //more error handling
      try {
        num = std::stoul(args[2]);
      } catch(...) {
        std::cout << "Please input a valid number for process_memory_size" << std::endl;
      }

      create_process_from_string(args[1], num, args[3], config,scheduler);

    break;
    case ScreenCommand::Unknown:
    default:
      std::cout << "Unknown screen command: " << args[0] << "\n";
      display_usage();
      break;
  }
}

}
