#ifndef OSEMU_CPU_WORKER_H_
#define OSEMU_CPU_WORKER_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace osemu {

class PCB; // Forward declaration is sufficient
class Scheduler;

class CpuWorker {
public:
  CpuWorker(int core_id, Scheduler& scheduler);
  ~CpuWorker();

  CpuWorker(const CpuWorker&) = delete;
  CpuWorker& operator=(const CpuWorker&) = delete;

  void Start();
  void Stop();
  void Join();
  void AssignTask(std::shared_ptr<PCB> pcb, int time_quantum);
  bool IsIdle() const { return is_idle_.load(); }

private:
  void Run();
  void ExecuteProcess(std::shared_ptr<PCB> pcb, int time_quantum);

  const int core_id_;
  Scheduler& scheduler_;
  std::thread thread_;

  std::atomic<bool> is_idle_{true};
  std::atomic<bool> shutdown_requested_{false};

  std::shared_ptr<PCB> current_task_{nullptr};
  int time_quantum_{0};

  size_t last_fault_instruction_ptr_ = 0;
  int consecutive_fault_count_ = 0;
  size_t instruction_ptr_at_window_start_ = 0;
  size_t tick_at_window_start_ = 0;
  mutable std::mutex mutex_;
  std::condition_variable condition_variable_;
};

}

#endif