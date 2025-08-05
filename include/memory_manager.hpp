#ifndef OSEMU_MEMORY_MANAGER_H_
#define OSEMU_MEMORY_MANAGER_H_

#include <atomic>
#include <cstdint>
#include <fstream>
#include <list>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

// --- Custom Exception Classes (should be defined once, here is a good place) ---
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

// Forward declarations
class PCB;
class Config;

// Data structures for paging
struct PageTableEntry {
    bool is_valid = false;      // Is the page in a frame? (the "present" bit)
    bool is_dirty = false;      // Has the page been written to since being loaded?
    bool is_referenced = false; // Has the page been accessed recently? (for LRU)
    uint32_t frame_id = 0;      // Which physical frame holds this page?
};

struct Frame {
    uint32_t id;
    uint32_t pcb_id = 0;   // Which process owns the page in this frame?
    uint32_t page_id = 0;  // Which virtual page is in this frame?
    bool is_free = true;

    Frame(uint32_t id) : id(id) {} // Simple constructor
};

class MemoryManager {
public:
    explicit MemoryManager(const Config& config); // Takes Config for all memory params

    // Process lifecycle management
    void cleanup_process(uint32_t pcb_id);

    // The single, core function for memory access
    uint32_t translate_address(uint32_t pcb_id, uint32_t virtual_address);

    // Convenience wrappers around translate_address
    uint16_t read_u16(uint32_t pcb_id, uint32_t virtual_address);
    void write_u16(uint32_t pcb_id, uint32_t virtual_address, uint16_t value);

    // Reporting functions
    void generate_process_smi_report(std::ostream& out, const std::unordered_map<std::string, std::shared_ptr<PCB>>& all_processes) const;
    void generate_vmstat_report(std::ostream& out) const;
    bool register_process(std::shared_ptr<PCB> pcb);
    bool is_registered(uint32_t pcb_id) const; // New function
     uint32_t get_free_frame_count() const; // <-- Add this new public method

private:
    // Paging mechanism helpers
    void handle_page_fault(uint32_t pcb_id, uint32_t virtual_address);
    uint32_t find_victim_frame();
    uint32_t allocate_frame();
    void evict_page(uint32_t frame_id);

    // Backing store I/O
    void save_page_to_backing_store(uint32_t pcb_id, uint32_t page_id, uint32_t frame_id);
    bool load_page_from_backing_store(uint32_t pcb_id, uint32_t page_id, uint32_t frame_id);

    // Configuration
    uint32_t total_memory_size_;
    uint32_t frame_size_;
    uint32_t num_frames_;

    // Main data structures
    std::vector<uint8_t> physical_memory_; // Represents the "physical RAM"
    std::vector<Frame> frames_;            // Management data for each frame of RAM
    std::unordered_map<uint32_t, std::vector<PageTableEntry>> page_tables_; // pcb_id -> page table

    // Backing Store
    std::string backing_store_filename_;

    // Synchronization
    mutable std::mutex memory_mutex_;

    // Statistics for vmstat
    mutable std::atomic<size_t> pages_paged_in_{0};
    mutable std::atomic<size_t> pages_paged_out_{0};
};

} // namespace osemu

#endif