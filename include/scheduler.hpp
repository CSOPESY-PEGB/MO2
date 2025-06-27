#ifndef OSEMU_SCHEDULER_H_
#define OSEMU_SCHEDULER_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "process_control_block.hpp"
#include "thread_safe_queue.hpp"
#include "instruction_generator.hpp"

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

  // Process generation
  void start_batch_generation(const Config& config);
  void stop_batch_generation();
  void calculate_cpu_utilization(size_t& total_cores, size_t& cores_used,
                                 double& cpu_utilization) const;
  bool is_generating() const { return batch_generating_; }
  std::shared_ptr<PCB> find_process_by_name(const std::string& name) const;
  // Report utilities
  void generate_report(const std::string& filename = "csopesy-log.txt") const;

  size_t get_ticks(){ return ticks_.load(); } //getter for ticks

 private:
  friend class CPUWorker;
  class CPUWorker;

  
  void move_to_running(std::shared_ptr<PCB> pcb);
  void move_to_finished(std::shared_ptr<PCB> pcb);
  void move_to_ready(std::shared_ptr<PCB> pcb);

  std::atomic<bool> running_;
  std::vector<std::unique_ptr<CPUWorker>> cpu_workers_;

  ThreadSafeQueue<std::shared_ptr<PCB>> ready_queue_;

  mutable std::mutex running_mutex_;
  mutable std::mutex finished_mutex_;

  std::vector<std::shared_ptr<PCB>> running_processes_;
  std::vector<std::shared_ptr<PCB>> finished_processes_;

  std::thread dispatch_thread_;

  // Batch process generation
  std::atomic<bool> batch_generating_;
  std::unique_ptr<std::thread> batch_generator_thread_;
  InstructionGenerator instruction_generator_;
  int process_counter_;

  //global synchronization
  std::atomic<size_t> ticks_{0}; //global counter.
  mutable std::mutex clock_mutex_; //part 1 of notifying all
  std::condition_variable clock_cv_; //part 2 of notifying all
  std::thread global_clock_thread_;


};

}  // namespace osemu

#endif  // OSEMU_SCHEDULER_H_
