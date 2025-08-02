#include "memory_manager.hpp"

#include <chrono>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace osemu {

// --- Constructor, allocate, free, coalesce_free_blocks are unchanged ---

MemoryManager::MemoryManager(uint32_t total_size)
    : total_memory_size_(total_size) {
    memory_map_.push_back({0, total_memory_size_, true, 0});
    std::cout << "Memory Manager initialized with " << total_size << " bytes." << std::endl;
}

bool MemoryManager::allocate(uint32_t pcb_id, uint32_t size,
                             std::shared_ptr<PCB> pcb) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    for (auto it = memory_map_.begin(); it != memory_map_.end(); ++it) {
        if (it->is_free && it->size >= size) {
            if (it->size > size) {
                uint32_t remaining_size = it->size - size;
                uint32_t new_block_start = it->start_address + size;
                MemoryBlock new_free_block = {new_block_start, remaining_size, true, 0, pcb};
                it->size = size;
                it->is_free = false;
                it->pcb_id = pcb_id;
                memory_map_.insert(std::next(it), new_free_block);
            } else {
                it->is_free = false;
                it->pcb_id = pcb_id;
                it->pcb_block = pcb;
            }
            // std::cout << "Allocated " << size << " bytes for PID " << pcb_id << " at address " << it->start_address << std::endl;
            return true;
        }
    }
    // std::cout << "Failed to allocate " << size << " bytes for PID " << pcb_id << ". No sufficient memory." << std::endl;
    return false;
}

void MemoryManager::free(uint32_t pcb_id) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    for (auto it = memory_map_.begin(); it != memory_map_.end(); ++it) {
        if (!it->is_free && it->pcb_id == pcb_id) {
            // std::cout << "Freeing memory for PID " << pcb_id << " at address " << it->start_address << std::endl;
            it->is_free = true;
            it->pcb_id = 0;
            it->pcb_block = nullptr;
            //coalesce_free_blocks(it);
            return;
        }
    }
}

void MemoryManager::coalesce_free_blocks(std::list<MemoryBlock>::iterator newly_freed_block) {
    auto next_block = std::next(newly_freed_block);
    if (next_block != memory_map_.end() && next_block->is_free) {
        newly_freed_block->size += next_block->size;
        memory_map_.erase(next_block);
    }
    if (newly_freed_block != memory_map_.begin()) {
        auto prev_block = std::prev(newly_freed_block);
        if (prev_block->is_free) {
            prev_block->size += newly_freed_block->size;
            memory_map_.erase(newly_freed_block);
        }
    }
}

// --- NEW METHOD IMPLEMENTATION ---
bool MemoryManager::is_allocated(uint32_t pcb_id) const {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    for (const auto& block : memory_map_) {
        if (!block.is_free && block.pcb_id == pcb_id) {
            return true; // Found the process in memory.
        }
    }
    return false; // Process not found in memory.
}


void MemoryManager::write_memory_report(std::ostream& out) const {
    std::lock_guard<std::mutex> lock(memory_mutex_);

    auto now = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now());
    out << "Timestamp: " << std::format("{:%m/%d/%Y %I:%M:%S %p}", now) << "\n";

    size_t procs_in_mem = 0;
    uint32_t external_frag_bytes = 0;
    for (const auto& block : memory_map_) {
        if (!block.is_free) {
            procs_in_mem++;
        } else {
            external_frag_bytes += block.size;
        }
    }

    out << "Number of processes in memory: " << procs_in_mem << "\n";
    out << "Total external fragmentation in KB: " << std::fixed << std::setprecision(2)
        << (static_cast<double>(external_frag_bytes) / 1024.0) << "\n\n";

    out << std::format("----end---- = {}\n", total_memory_size_);

    for (auto it = memory_map_.rbegin(); it != memory_map_.rend(); ++it) {
        const auto& block = *it;
        if (!block.is_free) {
            out << block.start_address + block.size << "\n";

            out << std::format("P{:02d} | {}\n", block.pcb_id, block.pcb_block->processName);
            out << block.start_address << "\n\n";

        }
    }

    out << std::format("----start---- = {}\n", 0);
}

void MemoryManager::generate_memory_report(const std::string& filename) const {
    std::ofstream report_file(filename);
    if (!report_file) {
        std::cerr << "Error: Could not open report file " << filename << std::endl;
        return;
    }
    write_memory_report(report_file);
}

void MemoryManager::generate_memory_report(std::ostream& out) const {
    write_memory_report(out);
}

}