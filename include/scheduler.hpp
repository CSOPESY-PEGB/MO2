#pragma once


#include "config.hpp"
#include "process_control_block.hpp"
#include "thread_safe_queue.h"
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>

namespace osemu {
    class Scheduler {
    public:
        Scheduler();
        ~Scheduler();

        void start(const Config& config);
        void stop();

        void submit_process(std::shared_ptr<PCB> pcb);
        void print_status();

    private:
        // Forward declare the CPUWorker class
        class CPUWorker;

        // State
        std::atomic<bool> m_running;
        std::vector<std::unique_ptr<CPUWorker>> m_cpu_workers;

        // Queues and Lists
        ThreadSafeQueue<std::shared_ptr<PCB>> m_ready_queue;

        std::vector<std::shared_ptr<PCB>> m_running_processes;
        std::mutex m_running_mutex;

        std::vector<std::shared_ptr<PCB>> m_finished_processes;
        std::mutex m_finished_mutex;

        // Internal methods for workers to use
        void move_to_running(std::shared_ptr<PCB> pcb);
        void move_to_finished(std::shared_ptr<PCB> pcb);
    };
}
