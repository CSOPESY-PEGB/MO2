#include "cpu_worker.h"

#include <chrono>
#include <memory>
#include <optional>

#include "scheduler.hpp" // Needs full scheduler definition

namespace osemu {

CpuWorker::CpuWorker(int core_id, Scheduler& scheduler)
    : core_id_(core_id), scheduler_(scheduler) {}

void CpuWorker::Start() {
    thread_ = std::thread(&CpuWorker::Run, this);
}

void CpuWorker::Stop() {
    shutdown_requested_ = true;
    condition_variable_.notify_one();
}

void CpuWorker::Join() {
    if (thread_.joinable()) {
        thread_.join();
    }
}

void CpuWorker::AssignTask(std::shared_ptr<PCB> pcb, int time_quantum) {
    std::lock_guard<std::mutex> lock(mutex_);
    time_quantum_ = time_quantum;
    current_task_ = std::move(pcb);
    is_idle_ = false;
    condition_variable_.notify_one();
}

void CpuWorker::Run() {
    // The loop condition should depend on both the scheduler's state AND its own shutdown request.
    while (scheduler_.IsRunning() && !shutdown_requested_.load()) {
        std::shared_ptr<PCB> task_to_run;
        int quantum_to_run;

        { // Lock scope for accessing shared task data
            std::unique_lock<std::mutex> lock(mutex_);

            // Wait until there's a task or shutdown is requested
            condition_variable_.wait(lock, [this] {
                return shutdown_requested_.load() || !is_idle_.load();
            });

            if (shutdown_requested_.load()) {
                break; // Exit loop on shutdown
            }

            if (current_task_) {
                task_to_run = current_task_;
                quantum_to_run = time_quantum_;
            } else {
                // Spurious wakeup, no task, go back to waiting
                is_idle_ = true;
                continue;
            }
        } // Lock is released here

        // Execute the process outside the lock
        ExecuteProcess(task_to_run, quantum_to_run);

        // After execution, update state under lock
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_task_ = nullptr;
            is_idle_ = true;
        }
    }
}

void CpuWorker::ExecuteProcess(std::shared_ptr<PCB> pcb, int time_quantum) {
    pcb->assignedCore = core_id_;
    scheduler_.move_to_running(pcb);

    size_t last_tick = scheduler_.get_ticks();
    std::optional<size_t> deadline_tick;
    if (time_quantum != -1) {
        deadline_tick = last_tick + time_quantum;
    }

    while (!pcb->isComplete() && !pcb->isTerminated()) {
        if (deadline_tick.has_value() && scheduler_.get_ticks() >= *deadline_tick) {
            break; // Time slice expired
        }

        if (!scheduler_.IsRunning() || shutdown_requested_.load()) {
            break;
        }
// wait for next tick
        {
            std::unique_lock<std::mutex> lock(scheduler_.GetClockMutex());
            scheduler_.GetClockCondition().wait(lock, [&]() {
                return scheduler_.get_ticks() > last_tick ||
                       !scheduler_.IsRunning() || shutdown_requested_.load();
            });
        }

        last_tick = scheduler_.get_ticks();

        // Pass the memory manager to the step function
      if (scheduler_.IsRunning() && !shutdown_requested_.load()) {
        pcb->step(*scheduler_.get_memory_manager());
      }
    }

    // After loop, check status and move PCB to the correct queue
    pcb->assignedCore = std::nullopt; // Unassign core
  if (pcb->isComplete() || pcb->isTerminated()) {
    scheduler_.move_to_finished(pcb);
  } else {
    // Not done, so it goes back to the ready queue.
    scheduler_.move_to_ready(pcb);
  }
}

} // namespace osemu