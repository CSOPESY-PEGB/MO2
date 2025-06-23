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

  void start(const Config& config);
  void stop();

  void submit_process(std::shared_ptr<PCB> pcb);
  void print_status() const;
  
  // Process generation
  void start_batch_generation(const Config& config);
  void stop_batch_generation();
  bool is_generating() const { return batch_generating_; }
  
  // Report utilities
  void generate_report(const std::string& filename = "csopesy-log.txt") const;

 private:
  friend class CPUWorker;
  class CPUWorker;

  void move_to_running(std::shared_ptr<PCB> pcb);
  void move_to_finished(std::shared_ptr<PCB> pcb);

  std::atomic<bool> running_;
  std::vector<std::unique_ptr<CPUWorker>> cpu_workers_;

  ThreadSafeQueue<std::shared_ptr<PCB>> ready_queue_;

  mutable std::mutex running_mutex_;
  mutable std::mutex finished_mutex_;

  std::vector<std::shared_ptr<PCB>> running_processes_;
  std::vector<std::shared_ptr<PCB>> finished_processes_;
  
  // Batch process generation
  std::atomic<bool> batch_generating_;
  std::unique_ptr<std::thread> batch_generator_thread_;
  InstructionGenerator instruction_generator_;
  int process_counter_;
};

}  // namespace osemu

#endif  // OSEMU_SCHEDULER_H_
