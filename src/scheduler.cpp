#include "scheduler.hpp"

#include "config.hpp"
#include <thread>
#include <iostream>
#include <fstream>
#include <chrono> // You were missing this for std::chrono
#include <format>
#include <algorithm>

namespace osemu {

// --- CPUWorker Implementation ---
// Define the CPUWorker class that was forward-declared in scheduler.hpp
class Scheduler::CPUWorker {
public:
    CPUWorker(int core_id, Scheduler& scheduler, std::atomic<bool>& running)
        : m_core_id(core_id), m_scheduler(scheduler), m_system_running(running) {}

    void start() {
        m_thread = std::thread(&CPUWorker::run, this);
    }

    void join() {
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }

private:
    void run() {
        while (m_system_running) {
            std::shared_ptr<PCB> pcb;
            // The worker needs to access the scheduler's private queue.
            // This is a bit tricky. We'll make CPUWorker a friend of Scheduler.
            m_scheduler.m_ready_queue.wait_and_pop(pcb);

            if (!m_system_running || !pcb) {
                break;
            }

            execute_process(pcb);
        }
    }


    void execute_process(std::shared_ptr<PCB> pcb) {
        pcb->assignedCore = m_core_id;
        m_scheduler.move_to_running(pcb);

        std::ofstream log_file(pcb->processName + ".txt");

        // FCFS: Run to completion
        while (!pcb->isComplete()) {
            pcb->step();

            auto now = std::chrono::system_clock::now();
            log_file << std::format("({:%m/%d/%Y %I:%M:%S%p}) Core:{} \"Hello world from {}!\"\n",
                                   now, m_core_id, pcb->processName);

            // to slow down simulation
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for 10ms
        }

        pcb->finishTime = std::chrono::system_clock::now();
        m_scheduler.move_to_finished(pcb);
    }

    int m_core_id;
    std::thread m_thread;
    Scheduler& m_scheduler;
    std::atomic<bool>& m_system_running;
};



Scheduler::Scheduler() : m_running(false) {}

Scheduler::~Scheduler() {
    if (m_running.load()) {
        stop();
    }
}

void Scheduler::start(const Config& config) {
    m_running = true;
    for (uint32_t i = 0; i < config.cpuCount; ++i) {
        m_cpu_workers.push_back(std::make_unique<CPUWorker>(i, *this, m_running));
        m_cpu_workers.back()->start();
    }
    std::cout << "Scheduler started with " << config.cpuCount << " cores." << std::endl;
}

void Scheduler::stop() {
    if (!m_running.exchange(false)) { // Prevent double-stopping
        return;
    }
    
    for (size_t i = 0; i < m_cpu_workers.size(); ++i) {
        m_ready_queue.push(nullptr);
    }

    for (auto& worker : m_cpu_workers) {
        worker->join();
    }
    m_cpu_workers.clear();
    std::cout << "Scheduler stopped." << std::endl;
}

void Scheduler::submit_process(std::shared_ptr<PCB> pcb) {
    std::cout << "Process " << pcb->processName << " submitted to scheduler." << std::endl;
    m_ready_queue.push(std::move(pcb));
}

void Scheduler::print_status() const{
    std::cout << "----------------------------------------------------------------\n";
    std::cout << "Running processes:\n";
    {
        std::lock_guard<std::mutex> lock(m_running_mutex);
        for (const auto& pcb : m_running_processes) {
            std::cout << pcb->status() << std::endl;
        }
    }
    
    std::cout << "\nFinished processes:\n";
    {
        std::lock_guard<std::mutex> lock(m_finished_mutex);
        for (const auto& pcb : m_finished_processes) {
            std::cout << pcb->status() << std::endl;
        }
    }
    std::cout << "----------------------------------------------------------------\n";
}

void Scheduler::move_to_running(std::shared_ptr<PCB> pcb) {
    std::lock_guard<std::mutex> lock(m_running_mutex);
    m_running_processes.push_back(std::move(pcb));
}

void Scheduler::move_to_finished(std::shared_ptr<PCB> pcb) {
    {
        std::lock_guard<std::mutex> lock(m_running_mutex);
        std::erase_if(m_running_processes, 
                      [&](const auto& p){ return p->processName == pcb->processName; });
    }
    {
        std::lock_guard<std::mutex> lock(m_finished_mutex);
        m_finished_processes.push_back(std::move(pcb));
    }
}

}