#ifndef OSEMU_MEMORY_MANAGER_H_
#define OSEMU_MEMORY_MANAGER_H_

#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include "process_control_block.hpp"
#include <stack>
#include <queue>

namespace osemu {

//we can keep this
struct MemoryBlock {
    uint32_t start_address;
    uint32_t size;
    bool is_free;
    uint32_t pcb_id; //id of process corresponding to frame
    uint32_t page_number; //page of the process corresponding to the frame. 
    
    std::vector<uint8_t> bits; //actual representation
}; 

struct PageFaultRequest{
    uint32_t pcb_id;
    uint32_t page_number;
    std::shared_ptr<PCB> process;
};

class MemoryManager {
public:
    explicit MemoryManager(uint32_t total_size);
    bool allocate(uint32_t pcb_id, uint32_t size); //allocates the upper n frames into main memory
    void free(uint32_t pcb_id); //should also delete from the backing store

    // NEW: Method to check if a process is already in memory.
    // This is a const method because it only reads the memory state.
    bool is_allocated(uint32_t pcb_id) const;
    void generate_memory_report(std::ostream& out) const;
    
    
    //demand paging api
    void handle_page_faults();
    void submit(std::shared_ptr<PCB> process); //load a process into the backing store upon process generation
    void request_page_fault(uint32_t pcb_id, uint32_t page_num, std::shared_ptr<PCB> process); //enqueues a page fault request

private:
    void handle_page_fault(PageFaultRequest request);
    void coalesce_free_blocks(std::list<MemoryBlock>::iterator newly_freed_block);
    void write_memory_report(std::ostream& out) const; // Internal reusable helper
    void generate_memory_report(const std::string& filename) const;

    std::shared_ptr<PCB> find_process_by_pid(uint32_t id);

    std::stack<PageFaultRequest> page_fault_reqs;
    std::queue<MemoryBlock> free_frame_list;
    std::unordered_map<uint64_t, uint32_t> swap_map; //offset : pcb_id
    //std::queue<MemoryBlock> victim_list; //add to da pcb

    std::unordered_map<std::string, std::shared_ptr<PCB>> all_processes_map_; //this is shared between the memory manager and scheduler.
    std::string bs = "csopesy-backing-store.txt"; //file name to open

    std::list<MemoryBlock> memory_map_;
    uint32_t total_memory_size_;
    mutable std::mutex memory_mutex_;
};

}
#endif
