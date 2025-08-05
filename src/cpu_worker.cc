#include "cpu_worker.h"

#include <chrono>
#include <memory>
#include <optional>

#include "scheduler.hpp"

namespace osemu {

CpuWorker::CpuWorker(int core_id, Scheduler& scheduler)
    : core_id_(core_id), scheduler_(scheduler) {}

CpuWorker::~CpuWorker() {
    // Ensure thread is stopped and joined on destruction
    Stop();
    Join();
}

void CpuWorker::Start() {
    thread_ = std::thread(&CpuWorker::Run, this);
}

void CpuWorker::Stop() {
    shutdown_requested_ = true;
    condition_variable_.notify_all();
}

void CpuWorker::Join() {
    if (thread_.joinable()) {
        thread_.join();
    }
}

void CpuWorker::AssignTask(std::shared_ptr<PCB> pcb, int time_quantum) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(shutdown_requested_.load()) return;
        time_quantum_ = time_quantum;
        current_task_ = std::move(pcb);
        is_idle_ = false;
    }
    condition_variable_.notify_one();
}

void CpuWorker::Run() {
    while (!shutdown_requested_.load()) {
        std::shared_ptr<PCB> task_to_run;
        int quantum_to_run;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_variable_.wait(lock, [this] {
                return shutdown_requested_.load() || !is_idle_.load();
            });

            if (shutdown_requested_.load()) break;

            task_to_run = std::move(current_task_);
            quantum_to_run = time_quantum_;
        }

        if(task_to_run) {
            ExecuteProcess(task_to_run, quantum_to_run);
        }

        is_idle_ = true;
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

    // --- SIMPLE PROGRESS DETECTOR ---
    size_t instruction_at_start_of_turn = pcb->currentInstruction;
    int ticks_without_progress = 0;
    const int PROGRESS_TIMEOUT = 200; // If no progress in 200 ticks, it's deadlocked.

    while (!pcb->isComplete() && !pcb->isTerminated() && !shutdown_requested_.load()) {
        if (deadline_tick.has_value() && scheduler_.get_ticks() >= *deadline_tick) {
            break; // Time slice expired
        }

        {
            std::unique_lock<std::mutex> lock(scheduler_.GetClockMutex());
            scheduler_.GetClockCondition().wait(lock, [&]() {
                return scheduler_.get_ticks() > last_tick || shutdown_requested_.load() || !scheduler_.IsRunning();
            });
        }

        if (shutdown_requested_.load() || !scheduler_.IsRunning()) break;

        last_tick = scheduler_.get_ticks();

        size_t instruction_before_step = pcb->currentInstruction;

        pcb->step(*scheduler_.get_memory_manager());

        if (pcb->currentInstruction > instruction_before_step) {
            // Progress was made! Reset the timeout counter.
            ticks_without_progress = 0;
            instruction_at_start_of_turn = pcb->currentInstruction;
        } else if (!pcb->isSleeping() && !pcb->isTerminated()) {
            // No progress was made (likely a fault), increment timeout.
            ticks_without_progress++;
        }

        if (ticks_without_progress > PROGRESS_TIMEOUT) {
            pcb->terminate("Deadlocked (no progress)");
            break;
        }
    }

    pcb->assignedCore = std::nullopt;
    if (!scheduler_.IsRunning()) return;

    if (pcb->isComplete() || pcb->isTerminated()) {
        scheduler_.move_to_finished(std::move(pcb));
    } else {
        scheduler_.move_to_ready(std::move(pcb));
    }
}

}