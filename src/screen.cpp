
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

#include "console.hpp"  
#include "instruction_generator.hpp"
#include "process_control_block.hpp"
#include "instruction_parser.hpp"
#include "scheduler.hpp"

namespace osemu {
namespace {





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
      throw std::runtime_error("Process " + process_name + " not found.");
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





bool create_process(const std::string& process_name, Scheduler& scheduler, Config& config, size_t memory_size = 512) {
  //check for existing processname
  if (scheduler.find_process_by_name(process_name) != nullptr) {
    std::cerr << "Error: Process '" << process_name << "' already exists. Please choose a unique name." << std::endl;
    return false; // Abort the creation
  }
  if (memory_size > config.max_overall_mem) {
    std::cerr << "Error: Process exceeds overall memory limit.\n";
    return false;
  }

  InstructionGenerator generator;

  auto instructions = generator.generateRandomProgram(config.minInstructions, config.maxInstructions, process_name, config.min_mem_per_proc, config.max_mem_per_proc);
  auto pcb = std::make_shared<PCB>(process_name, instructions, memory_size);
  
  std::cout << "Created process '" << process_name << "' with " 
            << instructions.size() << " instructions and memory size of "
            << memory_size << " bytes." << std::endl;
  
  scheduler.submit_process(pcb);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true;
}


void create_process_from_file(const std::string& filename, const std::string& process_name, Scheduler& scheduler) {
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
  
  auto pcb = std::make_shared<PCB>(process_name, program);
  
  std::cout << "Created process '" << process_name << "' from file '" << filename 
            << "' with " << program.size() << " instructions." << std::endl;
  
  scheduler.submit_process(pcb);
}


std::string unescapeQuotes(const std::string& str) {
    std::string result;
    bool inEscape = false;
    
    for (size_t i = 0; i < str.length(); ++i) {
        if (str[i] == '\\' && i + 1 < str.length() && str[i + 1] == '"') {
            // Skip the escape sequence for a quote
            result += '"';
            ++i; // Skip the next character (the quote)
        } else {
            result += str[i];
        }
    }

    return result;
}

std::string transformInstructions(const std::string& input) {
    std::string result;
    std::string tempInput = input;
    
    // Remove the starting and ending double quotes
    if (!tempInput.empty() && tempInput[0] == '"') {
        tempInput = tempInput.substr(1, tempInput.size() - 2);  // Remove quotes
    }
    
    // Unescape all double quotes
    tempInput = unescapeQuotes(tempInput);
    
    // Split the input by semicolons (convert to new lines)
    std::istringstream stream(tempInput);
    std::string line;
    
    while (std::getline(stream, line, ';')) {
        if (line.empty()) continue; // Skip empty lines
        if (line.starts_with(" PRINT")){
          result += line.substr(1);
          break;
        }; //dont clean print
        // Now process each instruction
        std::istringstream instrStream(line);
        std::string instruction;
        instrStream >> instruction; // Get the instruction (e.g., DECLARE, ADD)

        result += instruction + "(";

        // Process the arguments
        std::string arg;
        bool first = true;
        
        while (instrStream >> arg) {
            if (!first) result += ", ";
            result += arg;
            first = false;
        }

        result += ")\n";
    }

    return result;
}


void create_process_from_string(const std::string& process_name, size_t memory, const std::string& instructions, Config& config, Scheduler& scheduler){
  //remove starting and ending "
  //unescape all \"
  if (memory > config.max_overall_mem) {
    std::cerr << "Error: Process exceeds overall memory limit.\n";
    return;
  }

  std::string input = transformInstructions(instructions);
  std::vector<Expr> program;
  ParseResult result = InstructionParser::parse_program(input, program);

  if (!result.success) {
    std::cerr << "Parse error: " << result.error_msg << std::endl;
    std::cerr << "Remaining input: " << result.remaining << std::endl;
    return;
  }

  auto pcb = std::make_shared<PCB>(process_name, program, memory);
  
  std::cout << "Created process '" << process_name << " with " << memory << "bytes of memory." << std::endl;
  
  scheduler.submit_process(pcb);

}

enum class ScreenCommand { Start, Resume, List, File, Custom, Unknown };

void display_usage() {
  std::cout
      << "Usage:\n"
      << "  screen -s <name>     Start a new process with the given name.\n"
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

  const ScreenCommand cmd = parse_command(args[0]);

  switch (cmd) {
    case ScreenCommand::Start: {
      if(args.size() == 3) {
        try {
          size_t mem_siz  = static_cast<size_t>(std::stoull(args[2]));
          bool created_success = create_process(args[1], scheduler, config, mem_siz);
          if (created_success) {
            view_process_screen(args[1],scheduler);
          }
          break;
        } catch (const std::invalid_argument& e) {
            std::cout << "Invalid argument: " << e.what() << std::endl;
        } catch (const std::out_of_range& e) {
            std::cout << "Out of range: " << e.what() << std::endl;
        }
      } else if (args.size() == 2) {
        bool created_success = create_process(args[1], scheduler, config);
        if (created_success) {
          view_process_screen(args[1],scheduler);
        }
        break;
      } else {
        display_usage();
        return;
      }
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
      create_process_from_file(args[1], args[2], scheduler);
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
