#ifndef OSEMU_CONFIG_H_
#define OSEMU_CONFIG_H_

#include <cstdint>
#include <filesystem>

namespace osemu {

enum class SchedulingAlgorithm { FCFS, RoundRobin };

struct Config {
  uint32_t cpuCount{4};
  SchedulingAlgorithm scheduler{SchedulingAlgorithm::RoundRobin};
  uint32_t quantumCycles{5};
  uint32_t processGenFrequency{1};
  uint32_t minInstructions{1000};
  uint32_t maxInstructions{2000};
  uint32_t delayCyclesPerInstruction{0};
  uint32_t maxOverallMemory{1024};
  uint32_t memPerFrame{64};
 

  uint32_t max_overall_mem{16384};
  uint32_t mem_per_frame{16};
  uint32_t min_mem_per_proc{4096};
  uint32_t max_mem_per_proc{4096};

  explicit Config(uint32_t cpu = 4,
                  SchedulingAlgorithm sched = SchedulingAlgorithm::RoundRobin,
                  uint32_t quantum = 5, uint32_t freq = 1,
                  uint32_t minIns = 1000, uint32_t maxIns = 2000,
                  uint32_t delay = 0, uint32_t memPerFrame = 64, uint32_t minMemPerProc = 512, uint32_t maxMemPerProc = 1024, uint32_t maxOverallMemory = 1024);

  static Config fromFile(const std::filesystem::path& file);
};

}

#endif
