#include "config.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <cmath>

namespace osemu {

namespace {
bool is_power_of_2(uint32_t value) {
  return value > 0 && (value & (value - 1)) == 0;
}

uint32_t validate_memory_value(uint32_t value, const std::string& param_name) {
  // Different validation rules for different parameters
  if (param_name == "mem-per-frame") {
    // Frame sizes can be smaller, just need to be power of 2
    if (value < 2 || value > 65536) {
      throw std::runtime_error("Invalid memory allocation for " + param_name + ": " + std::to_string(value) + 
                               ". Must be between 2 and 65536 bytes.");
    }
  } else {
    // Process memory allocations must be at least 64 bytes
    if (value < 64 || value > 65536) {
      throw std::runtime_error("Invalid memory allocation for " + param_name + ": " + std::to_string(value) + 
                               ". Must be between 64 and 65536 bytes.");
    }
  }
  
  if (!is_power_of_2(value)) {
    throw std::runtime_error("Invalid memory allocation for " + param_name + ": " + std::to_string(value) + 
                             ". Must be a power of 2.");
  }
  return value;
}
}

Config::Config(uint32_t cpu, SchedulingAlgorithm sched, uint32_t quantum,
               uint32_t freq, uint32_t minIns, uint32_t maxIns, uint32_t delay, uint32_t mem_per_frame, uint32_t min_mem_per_proc, uint32_t max_mem_per_proc, uint32_t max_overall_mem)
    : cpuCount{std::clamp(cpu, 1u, 128u)},
      scheduler{sched},
      quantumCycles{
          std::clamp(quantum, 1u, std::numeric_limits<uint32_t>::max())},
      processGenFrequency{
          std::clamp(freq, 1u, std::numeric_limits<uint32_t>::max())},
      minInstructions{
          std::clamp(minIns, 1u, std::numeric_limits<uint32_t>::max())},
      maxInstructions{std::clamp(maxIns, minInstructions,
                                 std::numeric_limits<uint32_t>::max())},
      delayCyclesPerInstruction{delay},
      max_overall_mem{validate_memory_value(max_overall_mem, "max-overall-mem")},
      mem_per_frame{validate_memory_value(mem_per_frame, "mem-per-frame")},
      min_mem_per_proc{validate_memory_value(min_mem_per_proc, "min-mem-per-proc")},
      max_mem_per_proc{validate_memory_value(max_mem_per_proc, "max-mem-per-proc")} {
  if (scheduler != SchedulingAlgorithm::RoundRobin) {
    quantumCycles = 1;
  }
}

Config Config::fromFile(const std::filesystem::path& file) {
  std::ifstream in(file);
  if (!in) {
    throw std::runtime_error("Cannot open file: " + file.string());
  }

  Config cfg;
  std::string key, value;
  while (in >> key >> value) {
    if (key == "num-cpu") {
      cfg.cpuCount = std::stoul(value);
    } else if (key == "scheduler") {
      cfg.scheduler = (value == "fcfs") ? SchedulingAlgorithm::FCFS
                                        : SchedulingAlgorithm::RoundRobin;
    } else if (key == "quantum-cycles") {
      cfg.quantumCycles = std::stoul(value);
    } else if (key == "batch-process-freq") {
      cfg.processGenFrequency = std::stoul(value);
    } else if (key == "min-ins") {
      cfg.minInstructions = std::stoul(value);
    } else if (key == "max-ins") {
      cfg.maxInstructions = std::stoul(value);
    } else if (key == "delay-per-exec") {
      cfg.delayCyclesPerInstruction = std::stoul(value);
    } else if (key == "max-overall-mem"){
      cfg.max_overall_mem = validate_memory_value(std::stoul(value), "max-overall-mem");
    } else if (key == "mem-per-frame"){
      cfg.mem_per_frame = validate_memory_value(std::stoul(value), "mem-per-frame");
    } else if (key == "min-mem-per-proc"){
      cfg.min_mem_per_proc = validate_memory_value(std::stoul(value), "min-mem-per-proc");
    } else if (key == "max-mem-per-proc"){
      cfg.max_mem_per_proc = validate_memory_value(std::stoul(value), "max-mem-per-proc");
    }
  }
  return cfg;
}

}  // namespace osemu