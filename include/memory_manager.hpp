#ifndef OSEMU_MEMORY_MANAGER_H_
#define OSEMU_MEMORY_MANAGER_H_

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

// --- Custom Exception Classes ---
class PageFaultException : public std::exception {
public:
    const char* what() const noexcept override {
        return "A recoverable page fault occurred.";
    }
};

class ResourceLimitException : public std::runtime_error {
public:
  ResourceLimitException(const std::string& msg) : std::runtime_error(msg) {}
};

class AccessViolationException : public std::runtime_error {
public:
    AccessViolationException(const std::string& msg) : std::runtime_error(msg) {}
};

namespace osemu {

class PCB;
class Config;

struct PageTableEntry {
    bool is_valid = false;
    bool is_dirty = false;
    bool is_referenced = false;
    uint32_t frame_id = 0;
};

struct Frame {
    uint32_t id;
    uint32_t pcb_id = 0;
    uint32_t page_id = 0;
    bool is_free = true;

    Frame(uint32_t frame_id) : id(frame_id) {}
};

class MemoryManager {
public:
    explicit MemoryManager(const Config& config);

    bool register_process(std::shared_ptr<PCB> pcb);
    void cleanup_process(uint32_t pcb_id);

    uint16_t read_u16(uint32_t pcb_id, uint32_t virtual_address);
    void write_u16(uint32_t pcb_id, uint32_t virtual_address, uint16_t value);

    void generate_process_smi_report(std::ostream& out, const std::unordered_map<std::string, std::shared_ptr<PCB>>& all_processes) const;
    void generate_vmstat_report(std::ostream& out) const;

    bool is_registered(uint32_t pcb_id) const;
    uint32_t get_free_frame_count() const;

private:
    // --- UPDATED DECLARATIONS ---
    uint32_t translate_address(uint32_t pcb_id, uint32_t virtual_address, bool is_write);
    void handle_page_fault(uint32_t pcb_id, uint32_t page_id);
    // ----------------------------
  uint32_t translate_address_nolock(uint32_t pcb_id, uint32_t virtual_address, bool is_write) ;

    uint32_t find_victim_frame();
    uint32_t allocate_frame();
    void evict_page(uint32_t frame_id);

    void save_page_to_backing_store(uint32_t pcb_id, uint32_t page_id, uint32_t frame_id);
    bool load_page_from_backing_store(uint32_t pcb_id, uint32_t page_id, uint32_t frame_id);

    uint32_t total_memory_size_;
    uint32_t frame_size_;
    uint32_t num_frames_;
    std::string backing_store_filename_;

    std::vector<uint8_t> physical_memory_;
    std::vector<Frame> frames_;
    std::unordered_map<uint32_t, std::vector<PageTableEntry>> page_tables_;

    mutable std::mutex memory_mutex_;

    mutable std::atomic<size_t> pages_paged_in_{0};
    mutable std::atomic<size_t> pages_paged_out_{0};
};

}

#endif