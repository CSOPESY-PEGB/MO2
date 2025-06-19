#ifndef OSEMU_SCHEDULER_H_
#define OSEMU_SCHEDULER_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "process_control_block.hpp"
#include "thread_safe_queue.hpp"

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
};

}  // namespace osemu

#endif  // OSEMU_SCHEDULER_H_
