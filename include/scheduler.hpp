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
#include "memory_manager.hpp" // NEW: Include the memory manager header

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
  void print_status() const;

  void start_batch_generation(const Config& config);
  void stop_batch_generation();
  void calculate_cpu_utilization(size_t& total_cores, size_t& cores_used,
                                 double& cpu_utilization) const;
  bool is_generating() const { return batch_generating_; }
  std::shared_ptr<PCB> find_process_by_name(const std::string& name) const;
  
  void generate_report(const std::string& filename = "csopesy-log.txt") const;

  size_t get_ticks(){ return ticks_.load(); } 

  std::unique_ptr<MemoryManager> memory_manager_;

 private:
  friend class CPUWorker;
  class CPUWorker;
  void move_to_running(std::shared_ptr<PCB> pcb);
  void move_to_finished(std::shared_ptr<PCB> pcb);
  void move_to_ready(std::shared_ptr<PCB> pcb);
  
  std::atomic<int> cores_ready_for_next_tick_{0};
  int total_cores_{0};

  std::atomic<bool> running_;
  std::vector<std::unique_ptr<CPUWorker>> cpu_workers_;

  ThreadSafeQueue<std::shared_ptr<PCB>> ready_queue_;

  mutable std::mutex running_mutex_;
  mutable std::mutex finished_mutex_;

  mutable std::mutex map_mutex_;
  std::unordered_map<std::string, std::shared_ptr<PCB>> all_processes_map_;

  std::vector<std::shared_ptr<PCB>> running_processes_;
  std::vector<std::shared_ptr<PCB>> finished_processes_;

  std::thread dispatch_thread_;

  std::atomic<bool> batch_generating_;
  std::unique_ptr<std::thread> batch_generator_thread_;
  InstructionGenerator instruction_generator_;
  int process_counter_;
  mutable std::mutex process_counter_mutex_;
  
  std::atomic<size_t> ticks_{0}; 
  mutable std::mutex clock_mutex_; 
  std::condition_variable clock_cv_; 
  std::thread global_clock_thread_;
  
  // --- NEW and MODIFIED MEMBERS for Memory Management ---
  uint32_t mem_per_proc_{4096};
  std::atomic<size_t> quantum_report_counter_{0};
  // ---------------------------------------------------

  size_t batch_process_freq_{1};
  size_t delay_per_exec_{0};
  size_t quantum_cycles_{5};
  SchedulingAlgorithm algorithm_{SchedulingAlgorithm::FCFS};

};

}  

#endif
