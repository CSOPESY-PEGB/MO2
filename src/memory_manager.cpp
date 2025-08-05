#include "memory_manager.hpp"

#include <chrono>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace osemu {

// --- Constructor, allocate, free, coalesce_free_blocks are unchanged ---

MemoryManager::MemoryManager(uint32_t total_size, uint32_t frame_size)
    : total_memory_size_(total_size), frame_size_(frame_size), 
      num_frames_(total_size / frame_size), backing_store_filename_("csopesy-backing-store.txt") {
    memory_map_.push_back({0, total_memory_size_, true, 0});
    
    // Initialize frames
    frames_.reserve(num_frames_);
    for (uint32_t i = 0; i < num_frames_; ++i) {
        frames_.emplace_back(i, frame_size_);
    }
    
    std::cout << "Memory Manager initialized with " << total_size << " bytes (" 
              << num_frames_ << " frames of " << frame_size_ << " bytes each)." << std::endl;
}

bool MemoryManager::allocate(uint32_t pcb_id, uint32_t size,
                             std::shared_ptr<PCB> pcb) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    for (auto it = memory_map_.begin(); it != memory_map_.end(); ++it) {
        if (it->is_free && it->size >= size) {
            if (it->size > size) {
                uint32_t remaining_size = it->size - size;
                uint32_t new_block_start = it->start_address + size;
                MemoryBlock new_free_block = {new_block_start, remaining_size, true, 0, nullptr};
                it->size = size;
                it->is_free = false;
                it->pcb_id = pcb_id;
                it->pcb_block = pcb;
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
  uint32_t total_used_bytes = 0;

  for (const auto& block : memory_map_) {
    if (!block.is_free) {
      procs_in_mem++;
      total_used_bytes += block.size;
    } else {
      external_frag_bytes += block.size;
    }
  }

  double utilization = 100.0 * static_cast<double>(total_used_bytes) / static_cast<double>(total_memory_size_);

  out << "Number of processes in memory: " << procs_in_mem << "\n";
  out << "Total external fragmentation in KB: " << std::fixed << std::setprecision(2)
      << (static_cast<double>(external_frag_bytes) / 1024.0) << "\n";
  out << "Memory used (bytes): " << total_used_bytes << "MiB / " << total_memory_size_ << "MiB\n";
  out << "Memory utilization: " << std::fixed << std::setprecision(2) << utilization << "%\n\n";

  out << std::format("----end---- = {}\n", total_memory_size_);

  for (auto it = memory_map_.rbegin(); it != memory_map_.rend(); ++it) {
    const auto& block = *it;
    if (!block.is_free) {
      out << block.start_address + block.size << "\n";
      if (block.pcb_block) {
        out << std::format("P{:02d} | {}\n", block.pcb_id, block.pcb_block->processName);
      } else {
        out << std::format("P{:02d} | [null PCB]\n", block.pcb_id);
      }
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

void MemoryManager::initialize_page_table(uint32_t pcb_id, uint32_t num_pages) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    page_tables_[pcb_id] = std::vector<PageTableEntry>(num_pages);
}

bool MemoryManager::handle_page_fault(uint32_t pcb_id, uint32_t virtual_address) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    
    uint32_t page_id = virtual_address / frame_size_;
    
    // Check if page table exists
    if (page_tables_.find(pcb_id) == page_tables_.end()) {
        return false;
    }
    
    auto& page_table = page_tables_[pcb_id];
    if (page_id >= page_table.size()) {
        return false; // Invalid page access
    }
    
    // Allocate a frame
    uint32_t frame_id = allocate_frame();
    if (frame_id == UINT32_MAX) {
        // No free frames, need to evict
        frame_id = find_victim_frame();
        evict_page(frame_id);
    }
    
    // Load page from backing store
    std::vector<uint8_t> page_data(frame_size_, 0);
    if (load_page_from_backing_store(pcb_id, page_id, page_data)) {
        pages_paged_in_++;
    }
    
    // Update frame and page table
    frames_[frame_id].is_free = false;
    frames_[frame_id].pcb_id = pcb_id;
    frames_[frame_id].page_id = page_id;
    frames_[frame_id].data = page_data;
    
    page_table[page_id].is_valid = true;
    page_table[page_id].frame_id = frame_id;
    page_table[page_id].is_referenced = true;
    
    return true;
}

bool MemoryManager::read_from_memory(uint32_t pcb_id, uint32_t virtual_address, uint16_t& value) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    
    uint32_t page_id = virtual_address / frame_size_;
    uint32_t offset = virtual_address % frame_size_;
    
    if (page_tables_.find(pcb_id) == page_tables_.end()) {
        return false;
    }
    
    auto& page_table = page_tables_[pcb_id];
    if (page_id >= page_table.size() || !page_table[page_id].is_valid) {
        return false; // Page fault should be handled externally
    }
    
    uint32_t frame_id = page_table[page_id].frame_id;
    page_table[page_id].is_referenced = true;
    
    if (offset + 1 >= frame_size_) {
        return false; // Invalid access
    }
    
    // Read uint16 from frame data
    value = static_cast<uint16_t>(frames_[frame_id].data[offset]) |
            (static_cast<uint16_t>(frames_[frame_id].data[offset + 1]) << 8);
    
    return true;
}

bool MemoryManager::write_to_memory(uint32_t pcb_id, uint32_t virtual_address, uint16_t value) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    
    uint32_t page_id = virtual_address / frame_size_;
    uint32_t offset = virtual_address % frame_size_;
    
    if (page_tables_.find(pcb_id) == page_tables_.end()) {
        return false;
    }
    
    auto& page_table = page_tables_[pcb_id];
    if (page_id >= page_table.size() || !page_table[page_id].is_valid) {
        return false; // Page fault should be handled externally
    }
    
    uint32_t frame_id = page_table[page_id].frame_id;
    page_table[page_id].is_referenced = true;
    page_table[page_id].is_dirty = true;
    
    if (offset + 1 >= frame_size_) {
        return false; // Invalid access
    }
    
    // Write uint16 to frame data
    frames_[frame_id].data[offset] = static_cast<uint8_t>(value & 0xFF);
    frames_[frame_id].data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    
    return true;
}

uint32_t MemoryManager::allocate_frame() {
    for (uint32_t i = 0; i < num_frames_; ++i) {
        if (frames_[i].is_free) {
            return i;
        }
    }
    return UINT32_MAX; // No free frame
}

uint32_t MemoryManager::find_victim_frame() {
    // Simple LRU approximation: find least recently referenced frame
    for (uint32_t i = 0; i < num_frames_; ++i) {
        if (!frames_[i].is_free) {
            uint32_t pcb_id = frames_[i].pcb_id;
            uint32_t page_id = frames_[i].page_id;
            
            if (page_tables_.find(pcb_id) != page_tables_.end()) {
                auto& page_table = page_tables_[pcb_id];
                if (page_id < page_table.size() && !page_table[page_id].is_referenced) {
                    return i;
                }
            }
        }
    }
    
    // If all pages are referenced, clear reference bits and return first frame
    for (uint32_t i = 0; i < num_frames_; ++i) {
        if (!frames_[i].is_free) {
            uint32_t pcb_id = frames_[i].pcb_id;
            uint32_t page_id = frames_[i].page_id;
            
            if (page_tables_.find(pcb_id) != page_tables_.end()) {
                auto& page_table = page_tables_[pcb_id];
                if (page_id < page_table.size()) {
                    page_table[page_id].is_referenced = false;
                }
            }
        }
    }
    
    return 0; // Return first frame as victim
}

void MemoryManager::evict_page(uint32_t frame_id) {
    if (frame_id >= num_frames_ || frames_[frame_id].is_free) {
        return;
    }
    
    uint32_t pcb_id = frames_[frame_id].pcb_id;
    uint32_t page_id = frames_[frame_id].page_id;
    
    // Save to backing store if dirty
    if (page_tables_.find(pcb_id) != page_tables_.end()) {
        auto& page_table = page_tables_[pcb_id];
        if (page_id < page_table.size() && page_table[page_id].is_dirty) {
            save_page_to_backing_store(pcb_id, page_id, frames_[frame_id].data);
            pages_paged_out_++;
        }
        
        // Invalidate page table entry
        page_table[page_id].is_valid = false;
        page_table[page_id].is_dirty = false;
        page_table[page_id].is_referenced = false;
    }
    
    // Mark frame as free
    frames_[frame_id].is_free = true;
    frames_[frame_id].pcb_id = 0;
    frames_[frame_id].page_id = 0;
}

void MemoryManager::save_page_to_backing_store(uint32_t pcb_id, uint32_t page_id, const std::vector<uint8_t>& data) {
    std::ofstream file(backing_store_filename_, std::ios::binary | std::ios::app);
    if (file) {
        file.write(reinterpret_cast<const char*>(&pcb_id), sizeof(pcb_id));
        file.write(reinterpret_cast<const char*>(&page_id), sizeof(page_id));
        file.write(reinterpret_cast<const char*>(data.data()), data.size());
    }
}

bool MemoryManager::load_page_from_backing_store(uint32_t pcb_id, uint32_t page_id, std::vector<uint8_t>& data) {
    std::ifstream file(backing_store_filename_, std::ios::binary);
    if (!file) {
        return false;
    }
    
    uint32_t stored_pcb_id, stored_page_id;
    while (file.read(reinterpret_cast<char*>(&stored_pcb_id), sizeof(stored_pcb_id)) &&
           file.read(reinterpret_cast<char*>(&stored_page_id), sizeof(stored_page_id))) {
        
        if (stored_pcb_id == pcb_id && stored_page_id == page_id) {
            file.read(reinterpret_cast<char*>(data.data()), data.size());
            return true;
        } else {
            file.seekg(frame_size_, std::ios::cur); // Skip this page
        }
    }
    
    return false;
}

uint32_t MemoryManager::get_used_memory() const {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    uint32_t used = 0;
    // Check memory_map_ for traditional memory allocation
    for (const auto& block : memory_map_) {
        if (!block.is_free) {
            used += block.size;
        }
    }
    return used;
}

uint32_t MemoryManager::get_free_memory() const {
    return total_memory_size_ - get_used_memory();
}

}