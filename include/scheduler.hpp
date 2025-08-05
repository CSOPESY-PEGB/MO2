#ifndef OSEMU_SCHEDULER_H_
#define OSEMU_SCHEDULER_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>

#include "process_control_block.hpp"
#include "thread_safe_queue.hpp"
#include "instruction_generator.hpp"
#include "config.hpp"
#include "memory_manager.hpp"
#include "cpu_worker.h"

namespace osemu {

class Config;

class Scheduler {
 public:
  Scheduler();
  ~Scheduler();

  void dispatch();
  void global_clock();
  void start(const Config& config);
  void stop();

  void submit_process(std::shared_ptr<PCB> pcb);
  void print_status() const; // This is the main user-facing status command

  void start_batch_generation();
  void stop_batch_generation();

  void move_to_running(std::shared_ptr<PCB> pcb);
  void move_to_finished(std::shared_ptr<PCB> pcb);
  void move_to_ready(std::shared_ptr<PCB> pcb);

  std::shared_ptr<PCB> find_process_by_name(const std::string& name) const;

  // Getters for child classes and external components
  size_t get_ticks() const { return ticks_.load(); }
  MemoryManager* get_memory_manager() { return memory_manager_.get(); }
  std::mutex& GetClockMutex() { return clock_mutex_; }
  std::condition_variable& GetClockCondition() { return clock_cv_; }
  bool IsRunning() const { return running_.load(); }
  size_t GetDelayPerExecution() const { return config_.delayCyclesPerInstruction; }
  const std::unordered_map<std::string, std::shared_ptr<PCB>>& get_all_processes_map() const {
    return all_processes_map_;
  }
  std::atomic<bool> batch_generating_;
  bool is_generating() const { return batch_generating_.load(); }
  void generate_full_report(const std::string& filename = "csopesy-log.txt") const;

 private:
  friend class CpuWorker;

  // Helper for reporting
  void calculate_cpu_utilization(size_t& total_cores, size_t& cores_used,
                                 double& cpu_utilization) const;

  // Core components
  Config config_;
  std::unique_ptr<MemoryManager> memory_manager_;
  std::vector<std::unique_ptr<CpuWorker>> cpu_workers_;

  // State
  std::atomic<bool> running_;
  ThreadSafeQueue<std::shared_ptr<PCB>> ready_queue_;

  // Process tracking lists and their mutexes
  mutable std::mutex running_mutex_;
  std::vector<std::shared_ptr<PCB>> running_processes_;
  mutable std::mutex finished_mutex_;
  std::vector<std::shared_ptr<PCB>> finished_processes_;
  mutable std::mutex map_mutex_;
  std::unordered_map<std::string, std::shared_ptr<PCB>> all_processes_map_;

  // Threads
  std::thread dispatch_thread_;
  std::thread global_clock_thread_;

  // Batch generation
  std::unique_ptr<std::thread> batch_generator_thread_;
  InstructionGenerator instruction_generator_;
  int process_counter_;
  mutable std::mutex process_counter_mutex_;

  // Clock
  std::atomic<size_t> ticks_{0};
  mutable std::mutex clock_mutex_;
  std::condition_variable clock_cv_;
};

} // namespace osemu

#endif