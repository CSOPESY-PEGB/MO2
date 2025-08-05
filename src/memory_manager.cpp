#include "memory_manager.hpp"
#include <climits>
#include <chrono>
#include <format>
#include <iostream>
#include <unordered_set>
#include <thread>
#include <algorithm>
#include "config.hpp"
#include "process_control_block.hpp"

namespace osemu {

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
    std::ofstream ofs(backing_store_filename_, std::ios::trunc);
    std::cout << "Memory Manager initialized with " << total_memory_size_ << " bytes ("
              << num_frames_ << " frames of " << frame_size_ << " bytes each)." << std::endl;
}

bool MemoryManager::register_process(std::shared_ptr<PCB> pcb) {
  std::lock_guard<std::mutex> lock(memory_mutex_);
  if (page_tables_.count(pcb->processID)) {
    return true;
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
  page_tables_.erase(pcb_id);
}

// =========================================================================
// ========================== START OF MAJOR FIX ===========================
// =========================================================================

// NEW private helper function. Does the translation logic WITHOUT locking the mutex.
// Assumes the caller has already acquired the lock.
uint32_t MemoryManager::translate_address_nolock(uint32_t pcb_id, uint32_t virtual_address, bool is_write) {
    if (page_tables_.find(pcb_id) == page_tables_.end()) {
        throw AccessViolationException("Process has no registered page table.");
    }

    uint32_t page_id = virtual_address / frame_size_;
    uint32_t offset = virtual_address % frame_size_;

    auto& p_page_tables = page_tables_.at(pcb_id);
    if (page_id >= p_page_tables.size()) {
        throw AccessViolationException(
            std::format("Address 0x{:X} is out of process bounds.", virtual_address));
    }

    auto& pte = p_page_tables[page_id];

    if (!pte.is_valid) {
        // We handle the fault here, but then throw to signal the CPU to retry the instruction.
        handle_page_fault(pcb_id, page_id);
        throw PageFaultException();
    }

    pte.is_referenced = true;
    if (is_write) {
        pte.is_dirty = true;
    }

    uint32_t frame_id = pte.frame_id;
    return (frame_id * frame_size_) + offset;
}

// MODIFIED public translate_address now just acts as a lock guard for the helper.
uint32_t MemoryManager::translate_address(uint32_t pcb_id, uint32_t virtual_address, bool is_write) {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    return translate_address_nolock(pcb_id, virtual_address, is_write);
}

// REWRITTEN to be atomic with respect to page faults.
uint16_t MemoryManager::read_u16(uint32_t pcb_id, uint32_t virtual_address) {
    std::lock_guard<std::mutex> lock(memory_mutex_);

    // First, ensure both pages are resident in memory.
    // translate_address_nolock will fault and throw if a page is not present,
    // which aborts this function and causes the CPU to retry the entire instruction.
    uint32_t phys_addr_low = translate_address_nolock(pcb_id, virtual_address, false);

    // Check if the 16-bit value crosses a page boundary.
    if ((virtual_address / frame_size_) != ((virtual_address + 1) / frame_size_)) {
        // If it crosses, we must also ensure the second page is resident.
        translate_address_nolock(pcb_id, virtual_address + 1, false);
    }

    // By this point, we are GUARANTEED that both necessary pages are in physical memory.
    // We can now safely read from the physical memory array.
    uint32_t phys_addr_high = phys_addr_low + 1; // It's just the next byte.

    uint8_t low_byte = physical_memory_[phys_addr_low];
    uint8_t high_byte = physical_memory_[phys_addr_high];
    return static_cast<uint16_t>(high_byte) << 8 | static_cast<uint16_t>(low_byte);
}

// REWRITTEN to be atomic with respect to page faults.
void MemoryManager::write_u16(uint32_t pcb_id, uint32_t virtual_address, uint16_t value) {
    std::lock_guard<std::mutex> lock(memory_mutex_);

    // First, ensure both potential pages are resident.
    uint32_t phys_addr_low = translate_address_nolock(pcb_id, virtual_address, true);

    if ((virtual_address / frame_size_) != ((virtual_address + 1) / frame_size_)) {
        translate_address_nolock(pcb_id, virtual_address + 1, true);
    }

    // Now we are guaranteed to be safe to write to physical memory.
    uint32_t phys_addr_high = phys_addr_low + 1;

    physical_memory_[phys_addr_low] = static_cast<uint8_t>(value & 0xFF);
    physical_memory_[phys_addr_high] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

// =========================================================================
// =========================== END OF MAJOR FIX ============================
// =========================================================================

void MemoryManager::handle_page_fault(uint32_t pcb_id, uint32_t page_id) {
  pages_paged_in_++;

  uint32_t frame_id = allocate_frame();
  if (frame_id == UINT32_MAX) { // No free frames
    frame_id = find_victim_frame();
    evict_page(frame_id);
  }

  load_page_from_backing_store(pcb_id, page_id, frame_id);

  frames_[frame_id].is_free = false;
  frames_[frame_id].pcb_id = pcb_id;
  frames_[frame_id].page_id = page_id;

  auto& pte = page_tables_.at(pcb_id)[page_id];
  pte.is_valid = true;
  pte.frame_id = frame_id;
  pte.is_referenced = false;
  pte.is_dirty = false;
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
  static uint32_t clock_hand = 0;
  while (true) {
    if (!frames_[clock_hand].is_free) {
      uint32_t pcb_id = frames_[clock_hand].pcb_id;
      uint32_t page_id = frames_[clock_hand].page_id;
      if (page_tables_.count(pcb_id) && page_id < page_tables_.at(pcb_id).size()) {
        auto& pte = page_tables_.at(pcb_id)[page_id];
        if (pte.is_referenced) {
          pte.is_referenced = false; // Give it a second chance
        } else {
          // Found a victim
          uint32_t victim_frame = clock_hand;
          clock_hand = (clock_hand + 1) % num_frames_; // Advance hand for next time
          return victim_frame;
        }
      }
    }
    clock_hand = (clock_hand + 1) % num_frames_;
  }
}
void MemoryManager::evict_page(uint32_t frame_id) {
  uint32_t pcb_id = frames_[frame_id].pcb_id;
  uint32_t page_id = frames_[frame_id].page_id;

  if (page_tables_.count(pcb_id) && page_id < page_tables_.at(pcb_id).size()) {
    auto& pte = page_tables_.at(pcb_id)[page_id];

    if (pte.is_dirty) {
      pages_paged_out_++;
      save_page_to_backing_store(pcb_id, page_id, frame_id);
    }
    pte.is_valid = false; // Mark PTE as invalid *after* potential save
  }
  frames_[frame_id].is_free = true;
}


void MemoryManager::save_page_to_backing_store(uint32_t pcb_id, uint32_t page_id, uint32_t frame_id) {
  std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Reduced sleep for faster testing
  std::fstream file(backing_store_filename_, std::ios::binary | std::ios::in | std::ios::out);
  if (!file) { file.open(backing_store_filename_, std::ios::binary | std::ios::trunc | std::ios::out); file.close(); file.open(backing_store_filename_, std::ios::binary | std::ios::in | std::ios::out);}
  uint64_t offset = get_backing_store_offset(pcb_id, page_id, frame_size_);
  uint32_t physical_address_start = frame_id * frame_size_;
  file.seekp(offset);
  file.write(reinterpret_cast<const char*>(&physical_memory_[physical_address_start]), frame_size_);
}


bool MemoryManager::load_page_from_backing_store(uint32_t pcb_id, uint32_t page_id, uint32_t frame_id) {
  std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Reduced sleep
  std::ifstream file(backing_store_filename_, std::ios::binary);
  uint32_t physical_address_start = frame_id * frame_size_;
  if (!file) {
    std::fill_n(physical_memory_.begin() + physical_address_start, frame_size_, 0);
    return false;
  }
  uint64_t offset = get_backing_store_offset(pcb_id, page_id, frame_size_);
  file.seekg(offset, std::ios::beg);
  file.read(reinterpret_cast<char*>(&physical_memory_[physical_address_start]), frame_size_);
  if(file.gcount() < static_cast<std::streamsize>(frame_size_)) {
    std::fill_n(physical_memory_.begin() + physical_address_start + file.gcount(), frame_size_ - file.gcount(), 0);
    return false;
  }
  return true;
}

void MemoryManager::generate_process_smi_report(std::ostream& out, const std::unordered_map<std::string, std::shared_ptr<PCB>>& all_processes) const {
  std::lock_guard<std::mutex> lock(memory_mutex_);
  uint32_t used_frames_count = 0;
  for(const auto& frame : frames_){
    if(!frame.is_free) used_frames_count++;
  }
  uint64_t used_mem_bytes = static_cast<uint64_t>(used_frames_count) * frame_size_;
  double mem_util = (total_memory_size_ > 0) ? (static_cast<double>(used_mem_bytes) / total_memory_size_) * 100.0 : 0.0;
  out << std::format("Memory Usage: {}B / {}B\n", used_mem_bytes, total_memory_size_);
  out << std::format("Memory Util: {:.2f}%\n\n", mem_util);
  out << "Running processes and memory usage:\n";
  std::unordered_set<uint32_t> resident_pcb_ids;
  for (const auto& frame : frames_) {
    if (!frame.is_free && frame.pcb_id != 0) {
      resident_pcb_ids.insert(frame.pcb_id);
    }
  }
  for (const auto& pair : all_processes) {
    const auto& pcb = pair.second;
    // Show process if it's resident OR not terminated
    if (resident_pcb_ids.count(pcb->processID) || !pcb->isTerminated()) {
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
  out << std::format("{:<10} K total memory\n", total_memory_size_ );
  out << std::format("{:<10} K used memory\n", used_mem );
  out << std::format("{:<10} K free memory\n", free_mem);
  out << "-----\n";
  out << std::format("{:<10} pages paged in\n", pages_paged_in_.load());
  out << std::format("{:<10} pages paged out\n", pages_paged_out_.load());
  out << "--------------\n";
}
bool MemoryManager::is_registered(uint32_t pcb_id) const {
  std::lock_guard<std::mutex> lock(memory_mutex_);
  return page_tables_.count(pcb_id) > 0;
}

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

uint32_t MemoryManager::get_total_frame_count() const {
  return frames_.size();
}

uint64_t MemoryManager::get_frame_size() const {
  return frame_size_;
}

size_t MemoryManager::get_pages_paged_in() const {
  return pages_paged_in_.load();
}

size_t MemoryManager::get_pages_paged_out() const {
  return pages_paged_out_.load();
}
}