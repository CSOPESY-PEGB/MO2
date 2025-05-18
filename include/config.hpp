#pragma once
#include <cstdint>
#include <filesystem>

enum class Scheduler { FCFS, RoundRobin };

struct Config {
  uint32_t cpuCount = 4;
  Scheduler scheduler = Scheduler::RoundRobin;
  uint32_t quantumCycles = 5;
  uint32_t processGenFrequency = 1;
  uint32_t minInstructions = 1000;
  uint32_t maxInstructions = 2000;
  uint32_t delayCyclesPerInstruction = 0;

  explicit Config(uint32_t cpu = 4, Scheduler sched = Scheduler::RoundRobin,
                  uint32_t quantum = 5, uint32_t freq = 1,
                  uint32_t minIns = 1000, uint32_t maxIns = 2000,
                  uint32_t delay = 0);

  static Config fromFile(const std::filesystem::path& file);
};