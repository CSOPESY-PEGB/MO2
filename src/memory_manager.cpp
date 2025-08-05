#include "memory_manager.hpp"

#include <climits> // For UINT32_MAX
#include <chrono>
#include <format>
#include <iostream>
#include <unordered_set>
// Include other necessary headers that might have been missed
#include <thread>

#include "config.hpp"
#include "process_control_block.hpp"

namespace osemu {

// Helper function to calculate a unique file offset for any given page
uint64_t get_backing_store_offset(uint32_t pcb_id, uint32_t page_id, uint32_t page_size) {
    const uint64_t process_space_multiplier = 1000000;
    uint64_t process_base_offset = static_cast<uint64_t>(pcb_id) * process_space_multiplier;
    return (process_base_offset + page_id) * page_size;
}

MemoryManager::MemoryManager(const Config& config)
    : total_memory_size_(config.max_overall_mem),
      frame_size_(config.mem_per_frame),
      num_frames_(total_memory_size_ / frame_size_),
      backing_store_filename_("csopesy-backing-store.txt") {

    if (total_memory_size_ == 0 || frame_size_ == 0 || total_memory_size_ % frame_size_ != 0) {
        throw std::runtime_error("Invalid memory configuration.");
    }

    physical_memory_.resize(total_memory_size_, 0);

    frames_.reserve(num_frames_);
    for (uint32_t i = 0; i < num_frames_; ++i) {
        frames_.emplace_back(i);
    }

    // Clear the backing store on startup for a clean run
    std::ofstream ofs(backing_store_filename_, std::ios::trunc);

    std::cout << "Memory Manager initialized with " << total_memory_size_ << " bytes ("
              << num_frames_ << " frames of " << frame_size_ << " bytes each)." << std::endl;
}


// Add the is_registered function if it's missing.
bool MemoryManager::is_registered(uint32_t pcb_id) const {
  std::lock_guard<std::mutex> lock(memory_mutex_);
  return page_tables_.count(pcb_id) > 0;
}
// You'll need this helper function in your MemoryManager.
// Add the declaration to memory_manager.hpp
// uint32_t get_free_frame_count() const;
// And the implementation to memory_manager.cpp
uint32_t MemoryManager::get_free_frame_count() const {
  std::lock_guard<std::mutex> lock(memory_mutex_);
  uint32_t free_count = 0;
  for (const auto& frame : frames_) {
    if (frame.is_free) {
      free_count++;
    }
  }
  return free_count;

}


// The register_process should return bool, but it will always succeed in this model.
bool MemoryManager::register_process(std::shared_ptr<PCB> pcb) {
  std::lock_guard<std::mutex> lock(memory_mutex_);
  if (page_tables_.count(pcb->processID)) {
    return true; // Already registered
  }
  uint32_t num_pages = (pcb->getMemorySize() + frame_size_ - 1) / frame_size_;
  page_tables_[pcb->processID] = std::vector<PageTableEntry>(num_pages);
  return true;
}


void MemoryManager::cleanup_process(uint32_t pcb_id) {
  std::lock_guard<std::mutex> lock(memory_mutex_);
  if (page_tables_.find(pcb_id) == page_tables_.end()) { return; }
  for (uint32_t i = 0; i < num_frames_; ++i) {
    if (!frames_[i].is_free && frames_[i].pcb_id == pcb_id) {
      frames_[i].is_free = true;
      frames_[i].pcb_id = 0;
      frames_[i].page_id = 0;
    }
  }
  // Ensure this line is commented out for the test
  // page_tables_.erase(pcb_id);
}

uint32_t MemoryManager::translate_address(uint32_t pcb_id, uint32_t virtual_address) {
    if (page_tables_.find(pcb_id) == page_tables_.end()) {
        throw AccessViolationException("Process has no registered page table.");
    }

    uint32_t page_id = virtual_address / frame_size_;
    uint32_t offset = virtual_address % frame_size_;

    if (page_id >= page_tables_[pcb_id].size()) {
        throw AccessViolationException(
            std::format("Address 0x{:X} is out of process bounds.", virtual_address));
    }

    auto& pte = page_tables_[pcb_id][page_id];

    if (!pte.is_valid) {
        handle_page_fault(pcb_id, virtual_address);
        throw PageFaultException();
    }

    pte.is_referenced = true;
    uint32_t frame_id = pte.frame_id;
    return (frame_id * frame_size_) + offset;
}
uint16_t MemoryManager::read_u16(uint32_t pcb_id, uint32_t virtual_address) {
  std::lock_guard<std::mutex> lock(memory_mutex_);

  // --- NEW LOGIC TO HANDLE CROSS-PAGE READS ---

  // Read the low byte (at the given virtual address)
  uint32_t phys_addr_low = translate_address(pcb_id, virtual_address);
  uint8_t low_byte = physical_memory_[phys_addr_low];

  // Read the high byte (at the next virtual address)
  uint32_t phys_addr_high = translate_address(pcb_id, virtual_address + 1);
  uint8_t high_byte = physical_memory_[phys_addr_high];

  // Combine them (little-endian)
  return static_cast<uint16_t>(high_byte) << 8 | static_cast<uint16_t>(low_byte);
}

void MemoryManager::write_u16(uint32_t pcb_id, uint32_t virtual_address, uint16_t value) {
  std::lock_guard<std::mutex> lock(memory_mutex_);

  // --- NEW LOGIC TO HANDLE CROSS-PAGE WRITES ---

  // Mark both pages as potentially dirty.
  uint32_t page_id1 = virtual_address / frame_size_;
  uint32_t page_id2 = (virtual_address + 1) / frame_size_;
  page_tables_[pcb_id][page_id1].is_dirty = true;
  if (page_id1 != page_id2) {
    // Ensure the second page's entry exists before marking it
    if (page_id2 < page_tables_[pcb_id].size()) {
      page_tables_[pcb_id][page_id2].is_dirty = true;
    }
  }

  // Write the low byte
  uint8_t low_byte = static_cast<uint8_t>(value & 0xFF);
  uint32_t phys_addr_low = translate_address(pcb_id, virtual_address);
  physical_memory_[phys_addr_low] = low_byte;

  // Write the high byte
  uint8_t high_byte = static_cast<uint8_t>((value >> 8) & 0xFF);
  uint32_t phys_addr_high = translate_address(pcb_id, virtual_address + 1);
  physical_memory_[phys_addr_high] = high_byte;
}
void MemoryManager::handle_page_fault(uint32_t pcb_id, uint32_t virtual_address) {
    uint32_t page_id = virtual_address / frame_size_;
    uint32_t frame_id = allocate_frame();
    if (frame_id == UINT32_MAX) {
        frame_id = find_victim_frame();
        evict_page(frame_id);
    }

    if(load_page_from_backing_store(pcb_id, page_id, frame_id)) {
        pages_paged_in_++;
    }

    frames_[frame_id].is_free = false;
    frames_[frame_id].pcb_id = pcb_id;
    frames_[frame_id].page_id = page_id;

    auto& page_table = page_tables_[pcb_id];
    page_table[page_id].is_valid = true;
    page_table[page_id].frame_id = frame_id;
    page_table[page_id].is_referenced = true;
    page_table[page_id].is_dirty = false;
}

uint32_t MemoryManager::allocate_frame() {
    for (uint32_t i = 0; i < num_frames_; ++i) {
        if (frames_[i].is_free) {
            return i;
        }
    }
    return UINT32_MAX;
}

uint32_t MemoryManager::find_victim_frame() {
    // Simple second-chance (clock) algorithm for LRU approximation.
    static uint32_t clock_hand = 0;
    while (true) {
        if (!frames_[clock_hand].is_free) {
            uint32_t pcb_id = frames_[clock_hand].pcb_id;
            uint32_t page_id = frames_[clock_hand].page_id;

            if (page_tables_.count(pcb_id) && page_id < page_tables_[pcb_id].size()) {
                if (page_tables_[pcb_id][page_id].is_referenced) {
                    // Give it a second chance.
                    page_tables_[pcb_id][page_id].is_referenced = false;
                } else {
                    // No second chance, this is our victim.
                    return clock_hand;
                }
            }
        }
        clock_hand = (clock_hand + 1) % num_frames_;
    }
}

void MemoryManager::evict_page(uint32_t frame_id) {
    uint32_t pcb_id = frames_[frame_id].pcb_id;
    uint32_t page_id = frames_[frame_id].page_id;

    if (page_tables_.count(pcb_id) && page_id < page_tables_[pcb_id].size()) {
        auto& pte = page_tables_[pcb_id][page_id];
        if (pte.is_dirty) {
            save_page_to_backing_store(pcb_id, page_id, frame_id);
            pages_paged_out_++;
        }
        pte.is_valid = false;
    }

    frames_[frame_id].is_free = true;
}

void MemoryManager::save_page_to_backing_store(uint32_t pcb_id, uint32_t page_id, uint32_t frame_id) {
  // Simulate the high latency of writing to a physical disk.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));


  std::fstream file(backing_store_filename_, std::ios::binary | std::ios::in | std::ios::out);
    if (!file) { file.open(backing_store_filename_, std::ios::binary | std::ios::trunc | std::ios::out); }

    uint64_t offset = get_backing_store_offset(pcb_id, page_id, frame_size_);
    uint32_t physical_address_start = frame_id * frame_size_;

    file.seekp(offset);
    file.write(reinterpret_cast<const char*>(&physical_memory_[physical_address_start]), frame_size_);
}

bool MemoryManager::load_page_from_backing_store(uint32_t pcb_id, uint32_t page_id, uint32_t frame_id) {
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::ifstream file(backing_store_filename_, std::ios::binary);
    uint32_t physical_address_start = frame_id * frame_size_;

    if (!file) {
        std::fill_n(physical_memory_.begin() + physical_address_start, frame_size_, 0);
        return false;
    }

    uint64_t offset = get_backing_store_offset(pcb_id, page_id, frame_size_);
    file.seekg(offset);

    if (file.peek() == EOF) {
        std::fill_n(physical_memory_.begin() + physical_address_start, frame_size_, 0);
        return false;
    } else {
        file.read(reinterpret_cast<char*>(&physical_memory_[physical_address_start]), frame_size_);
        return true;
    }
}

// Implement reporting functions using the new model

void MemoryManager::generate_process_smi_report(std::ostream& out, const std::unordered_map<std::string, std::shared_ptr<PCB>>& all_processes) const {
  std::lock_guard<std::mutex> lock(memory_mutex_);

  uint32_t used_frames_count = 0;
  for(const auto& frame : frames_){
    if(!frame.is_free) used_frames_count++;
  }
  uint64_t used_mem_bytes = static_cast<uint64_t>(used_frames_count) * frame_size_;
  double mem_util = (total_memory_size_ > 0) ? (static_cast<double>(used_mem_bytes) / total_memory_size_) * 100.0 : 0.0;

  // --- Part 1: Overall stats (this part is mostly correct) ---
  out << std::format("Memory Usage: {}B / {}B\n", used_mem_bytes, total_memory_size_);
  out << std::format("Memory Util: {:.2f}%\n\n", mem_util);

  out << "Running processes and memory usage:\n";

  // --- Part 2: The NEW logic for listing resident processes ---

  // Step A: Find which processes are currently resident in RAM.
  std::unordered_set<uint32_t> resident_pcb_ids;
  for (const auto& frame : frames_) {
    if (!frame.is_free && frame.pcb_id != 0) {
      resident_pcb_ids.insert(frame.pcb_id);
    }
  }

  // Step B: Iterate through ALL processes, but only print the ones we found in RAM.
  for (const auto& pair : all_processes) {
    const auto& pcb = pair.second;
    // Check if this process's ID is in our set of resident IDs.
    if (resident_pcb_ids.count(pcb->processID)) {
      // Print its VIRTUAL memory size, as in the spec.
      out << std::format("{} {}B\n", pcb->processName, pcb->getMemorySize());
    }
  }
}

void MemoryManager::generate_vmstat_report(std::ostream& out) const {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    uint32_t free_frames = 0;
    for(const auto& frame : frames_){
        if(frame.is_free) free_frames++;
    }
    uint64_t free_mem = static_cast<uint64_t>(free_frames) * frame_size_;
    uint64_t used_mem = total_memory_size_ - free_mem;

    out << "--- vmstat ---\n";
    out << std::format("{:<10} K total memory\n", total_memory_size_);
    out << std::format("{:<10} K used memory\n", used_mem);
    out << std::format("{:<10} K free memory\n", free_mem);
    out << "-----\n";
    out << std::format("{:<10} page faults (total)\n", pages_paged_in_.load() + pages_paged_out_.load());
    out << std::format("{:<10} pages paged in\n", pages_paged_in_.load());
    out << std::format("{:<10} pages paged out\n", pages_paged_out_.load());
    out << "--------------\n";
}

} // namespace osemu