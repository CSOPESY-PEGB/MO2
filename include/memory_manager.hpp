#ifndef OSEMU_MEMORY_MANAGER_H_
#define OSEMU_MEMORY_MANAGER_H_

#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>

#include "process_control_block.hpp"

namespace osemu {

struct MemoryBlock {
    uint32_t start_address;
    uint32_t size;
    bool is_free;
    uint32_t pcb_id;
    std::shared_ptr<PCB> pcb_block;
};

struct Frame {
    uint32_t frame_id;
    bool is_free;
    uint32_t pcb_id;
    uint32_t page_id;
    std::vector<uint8_t> data;
    
    Frame(uint32_t id, uint32_t frame_size) : frame_id(id), is_free(true), pcb_id(0), page_id(0), data(frame_size, 0) {}
};

struct PageTableEntry {
    bool is_valid;
    uint32_t frame_id; 
    bool is_dirty;
    bool is_referenced;
    
    PageTableEntry() : is_valid(false), frame_id(0), is_dirty(false), is_referenced(false) {}
};

class MemoryManager {
public:
    explicit MemoryManager(uint32_t total_size, uint32_t frame_size = 16);
    bool allocate(uint32_t pcb_id, uint32_t size, std::shared_ptr<PCB> pcb_block);
    void free(uint32_t pcb_id);

    // NEW: Method to check if a process is already in memory.
    // This is a const method because it only reads the memory state.
    bool is_allocated(uint32_t pcb_id) const;
    void generate_memory_report(const std::string& filename) const;
    void generate_memory_report(std::ostream& out) const;
    void write_memory_report(std::ostream& out) const; // Internal reusable helper
    
    // Demand paging methods
    bool handle_page_fault(uint32_t pcb_id, uint32_t virtual_address);
    bool read_from_memory(uint32_t pcb_id, uint32_t virtual_address, uint16_t& value);
    bool write_to_memory(uint32_t pcb_id, uint32_t virtual_address, uint16_t value);
    void initialize_page_table(uint32_t pcb_id, uint32_t num_pages);
    
    // Statistics for vmstat
    size_t get_pages_paged_in() const { return pages_paged_in_; }
    size_t get_pages_paged_out() const { return pages_paged_out_; }
    uint32_t get_total_memory() const { return total_memory_size_; }
    uint32_t get_used_memory() const;
    uint32_t get_free_memory() const;
    
private:
    void save_page_to_backing_store(uint32_t pcb_id, uint32_t page_id, const std::vector<uint8_t>& data);
    bool load_page_from_backing_store(uint32_t pcb_id, uint32_t page_id, std::vector<uint8_t>& data);
    uint32_t find_victim_frame(); // LRU page replacement
    uint32_t allocate_frame();
    void evict_page(uint32_t frame_id);

private:
    void coalesce_free_blocks(std::list<MemoryBlock>::iterator newly_freed_block);
    std::list<MemoryBlock> memory_map_;
    uint32_t total_memory_size_;
    mutable std::mutex memory_mutex_;
    
    // Demand paging structures
    std::vector<Frame> frames_;
    std::unordered_map<uint32_t, std::vector<PageTableEntry>> page_tables_; // pcb_id -> page table
    uint32_t frame_size_;
    uint32_t num_frames_;
    std::string backing_store_filename_;
    
    // Statistics
    mutable std::atomic<size_t> pages_paged_in_{0};
    mutable std::atomic<size_t> pages_paged_out_{0};
};

}
#endif
