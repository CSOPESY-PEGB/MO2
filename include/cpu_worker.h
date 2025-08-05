#ifndef OSEMU_CPU_WORKER_H_
#define OSEMU_CPU_WORKER_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "process_control_block.hpp"

namespace osemu {

// Forward declaration
class Scheduler;

// CpuWorker handles execution of processes on individual CPU cores.
// Each CpuWorker runs in its own thread and can execute one process at a time.
class CpuWorker {
 public:
  // Constructs a CpuWorker for the specified core ID and scheduler.
  // Args:
  //   core_id: The ID of the CPU core this worker represents
  //   scheduler: Reference to the scheduler that manages this worker
  CpuWorker(int core_id, Scheduler& scheduler);
  
  // Destructor ensures proper cleanup of resources
  ~CpuWorker() = default;
  
  // Non-copyable and non-movable
  CpuWorker(const CpuWorker&) = delete;
  CpuWorker& operator=(const CpuWorker&) = delete;
  CpuWorker(CpuWorker&&) = delete;
  CpuWorker& operator=(CpuWorker&&) = delete;

  // Starts the worker thread for this CPU core
  void Start();
  
  // Stops the worker thread and signals shutdown
  void Stop();
  
  // Waits for the worker thread to complete
  void Join();
  
  // Assigns a task to this worker with the specified time quantum
  // Args:
  //   pcb: Process to execute
  //   time_quantum: Maximum time slices to run (-1 for unlimited)
  void AssignTask(std::shared_ptr<PCB> pcb, int time_quantum);
  
  // Returns true if this worker is currently idle
  bool IsIdle() const { return is_idle_.load(); }
  
  // Returns the core ID this worker represents
  int GetCoreId() const { return core_id_; }

 private:
  // Main execution loop for the worker thread
  void Run();
  
  // Executes a single process for the specified time quantum
  // Args:
  //   pcb: Process to execute
  //   time_quantum: Maximum time slices to run
  void ExecuteProcess(std::shared_ptr<PCB> pcb, int time_quantum);

  const int core_id_;
  Scheduler& scheduler_;
  std::thread thread_;
  
  std::atomic<bool> is_idle_{true};
  std::atomic<bool> shutdown_requested_{false};
  
  std::shared_ptr<PCB> current_task_;
  int time_quantum_;
  int ticks_with_current_task_;
  
  mutable std::mutex mutex_;
  std::condition_variable condition_variable_;
};

}  // namespace osemu

#endif  // OSEMU_CPU_WORKER_H_