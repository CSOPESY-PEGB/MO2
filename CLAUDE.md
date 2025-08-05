# CSOPESY OS Emulator - Complete System Documentation

## Overview

This is a comprehensive C++ operating system emulator (CSOPESY) that simulates process scheduling, memory management, and CPU execution. The system implements advanced OS concepts including virtual memory with demand paging, multi-core processing, and various scheduling algorithms.

**Project Structure:**
- **Language:** C++20
- **Build System:** CMake
- **Architecture:** Multi-threaded with barrier synchronization
- **Entry Point:** `src/main.cpp`

## Recent Major Changes (vmstat Integration)

The latest commits (f732d63, 7da685a) introduced significant vmstat integration improvements:

- **Barrier Synchronization:** Added sophisticated CPU worker barrier synchronization for coordinated execution
- **External CpuWorker Integration:** Enhanced CpuWorker architecture with vmstat monitoring capabilities
- **InstructionExecutionInfo:** New execution result tracking for detailed vmstat reporting
- **Enhanced Memory Manager:** Added statistics tracking for pages paged in/out for vmstat
- **Hybrid Architecture:** Combines vmstat dispatch mechanisms with enhanced CpuWorker features

## System Architecture

### Core Components

#### 1. Main Entry Point (`src/main.cpp`)
- Initializes Config, Scheduler, and console prompt
- Implements main command loop with token parsing
- Command dispatch through Commands enum and dispatcher
- Error handling and graceful shutdown

#### 2. Console System (`src/console.cpp`, `include/console.hpp`)
- ASCII art welcome message and branding
- Simple interface displaying OS Emulator v0.1 by PEGP team
- Terminal formatting with color codes

#### 3. Command System
**Files:** `src/commands.cpp`, `include/commands.hpp`

**Supported Commands:**
- `initialize` - Load system configuration
- `screen` - Process management interface  
- `scheduler-start`/`scheduler-stop` - Batch process generation control
- `report-util` - Generate system utilization report
- `process-smi` - Display running processes and memory usage
- `vmstat` - Virtual memory statistics display
- `clear` - Clear screen and show prompt
- `exit` - Shutdown emulator

**Command Processing:**
- Hash map-based command lookup for O(1) performance
- String-to-enum conversion with error handling
- Modular dispatch system through dispatcher component

#### 4. Dispatcher (`src/dispatcher.cpp`, `include/dispatcher.hpp`)
- Central command routing and execution
- Initialization state management (prevents commands before `initialize`)
- Configuration loading and scheduler management
- Screen command delegation
- Process generation control
- Report generation coordination

### Process Management

#### 5. Process Control Block (PCB)
**Files:** `src/process_control_block.cpp`, `include/process_control_block.hpp`

**Core Functionality:**
- **Process Representation:** Contains all process metadata and execution state
- **Instruction Execution:** Step-by-step instruction processing with execution info tracking
- **Memory Integration:** Heap memory management with symbol table for variables
- **State Management:** Sleep cycles, completion status, core assignment tracking
- **Logging:** Output log generation with timestamps for debugging

**Key Data Structures:**
```cpp
class PCB {
    std::atomic<uint32_t> processID;           // Unique process identifier
    std::string processName;                   // Process name
    std::atomic<size_t> currentInstruction;   // Instruction pointer
    size_t totalInstructions;                 // Total instruction count
    std::optional<uint32_t> assignedCore;     // CPU core assignment
    std::vector<Expr> instructions;           // Parsed instruction sequence
    std::vector<uint8_t> heap_memory;         // Process memory space
    std::unordered_map<std::string, size_t> symbol_table; // Variable mappings
    std::vector<std::string> output_log;      // Execution output
    std::unique_ptr<InstructionEvaluator> evaluator; // Instruction processor
};
```

**Important Methods:**
- `step()` - Execute one instruction, return execution info for barrier sync
- `executeCurrentInstruction()` - Handle specific instruction types
- `isComplete()` - Check process completion status
- `status()` - Generate formatted status string for reports

#### 6. Scheduler (`src/scheduler.cpp`, `include/scheduler.hpp`)
**Core Responsibility:** Multi-threaded process scheduling with barrier synchronization

**Architecture Features:**
- **Multi-Core Support:** External CpuWorker integration with vmstat capabilities
- **Barrier Synchronization:** Coordinated execution across all CPU cores
- **Scheduling Algorithms:** FCFS and Round-Robin with configurable quantum
- **Memory Integration:** Full integration with MemoryManager for demand paging
- **Process Generation:** Automatic batch process creation with configurable frequency

**Key Components:**
```cpp
class Scheduler {
    std::vector<std::unique_ptr<CpuWorker>> cpu_workers_;     // CPU cores
    ThreadSafeQueue<std::shared_ptr<PCB>> ready_queue_;       // Process queue
    std::unique_ptr<MemoryManager> memory_manager_;           // Memory system
    
    // Barrier Synchronization
    std::atomic<int> workers_completed_step_count_;           // Barrier counter
    std::condition_variable dispatch_go_cv_;                  // Start signal
    std::condition_variable workers_done_cv_;                 // Completion signal
    std::mutex barrier_mutex_;                                // Barrier protection
};
```

**Scheduling Process:**
1. **Process Generation:** Automatic process creation based on batch frequency
2. **CPU Assignment:** Idle CPU detection and process assignment
3. **Barrier Execution:** Coordinated execution across all cores
4. **Quantum Management:** Time slice enforcement for round-robin scheduling
5. **Memory Management:** Page fault handling and memory allocation

**Vmstat Integration:**
- CPU utilization tracking (idle vs active ticks)
- Memory statistics integration
- Page fault counting and reporting
- Process memory usage monitoring

#### 7. CPU Worker (`src/cpu_worker.cpp`, `include/cpu_worker.h`)
**Purpose:** Individual CPU core simulation with thread-based execution

**Architecture:**
- **Thread-Based:** Each core runs in separate thread
- **Barrier Synchronization:** Waits for scheduler signal before each execution cycle
- **Time Quantum Support:** Configurable time slicing for round-robin scheduling
- **Memory Integration:** Handles page faults during instruction execution

**Key Features:**
```cpp
class CpuWorker {
    uint32_t core_id_;                    // Core identifier
    Scheduler& scheduler_;                // Parent scheduler reference
    std::thread thread_;                  // Worker execution thread
    std::shared_ptr<PCB> current_task_;   // Assigned process
    std::atomic<bool> is_idle_;          // Idle state flag
};
```

**Execution Cycle:**
1. Wait for scheduler barrier signal
2. Execute assigned process for time quantum
3. Handle page faults and memory operations
4. Report completion to scheduler
5. Update process state (running/finished/ready)

### Memory Management

#### 8. Memory Manager (`src/memory_manager.cpp`, `include/memory_manager.hpp`)
**Dual Memory Architecture:** Traditional allocation + Virtual memory with demand paging

**Core Features:**
- **Physical Memory:** Frame-based physical memory simulation
- **Virtual Memory:** Process-specific page tables with demand paging
- **Page Replacement:** LRU (Least Recently Used) algorithm
- **Backing Store:** File-based page swapping for virtual memory
- **Statistics Tracking:** Page fault counting, memory utilization monitoring

**Data Structures:**
```cpp
class MemoryManager {
    struct MemoryBlock {
        size_t start_address;
        size_t size;
        std::shared_ptr<PCB> pcb;
    };
    
    struct Frame {
        uint32_t frame_id;
        std::vector<uint8_t> data;
        std::shared_ptr<PCB> pcb;
        size_t last_access_time;
    };
    
    struct PageTableEntry {
        bool valid;
        uint32_t frame_id;
        bool dirty;
        bool referenced;
    };
};
```

**Memory Operations:**
- `allocate()/free()` - Traditional memory allocation
- `handle_page_fault()` - Demand paging implementation
- `read_from_memory()/write_to_memory()` - Virtual memory access
- `find_victim_frame()` - LRU page replacement

**Vmstat Features:**
- Page fault statistics (`get_pages_paged_in()`, `get_pages_paged_out()`)
- Memory utilization tracking
- Frame usage monitoring
- Backing store management

### Instruction Processing Pipeline

#### 9. Instruction Parser (`src/instruction_parser.cpp`, `include/instruction_parser.hpp`)
**Purpose:** Parse text-based instruction language into executable expression trees

**Supported Instructions:**
- `DECLARE(variable, value)` - Variable declaration
- `PRINT(message)` - Output generation
- `ADD(result, operand1, operand2)` - Addition operation
- `SUB(result, operand1, operand2)` - Subtraction operation
- `FOR(instructions, count)` - Loop structures
- `READ(address)` - Memory read operations
- `WRITE(address, value)` - Memory write operations

**Parser Architecture:**
- **Recursive Descent:** Hierarchical parsing structure
- **Expression Trees:** AST generation for evaluation
- **Error Reporting:** Detailed parse error information
- **Hex Support:** Memory address parsing in hexadecimal

#### 10. Instruction Evaluator (`src/instruction_evaluator.cpp`, `include/instruction_evaluator.hpp`)
**Purpose:** Execute parsed instruction expressions with memory integration

**Key Features:**
- **Variable Management:** Symbol table integration with memory addresses
- **Memory Operations:** Integration with demand paging system
- **Arithmetic Operations:** Add/subtract with variable support
- **Control Flow:** For loop execution with nesting support
- **Output Generation:** Timestamped log creation

**Memory Integration:**
- Virtual memory operations through MemoryManager
- Page fault handling during memory access
- Variable storage in process heap memory
- Memory protection and boundary checking

#### 11. Instruction Generator (`src/instruction_generator.cpp`, `include/instruction_generator.hpp`)
**Purpose:** Generate random instruction sequences for process simulation

**Generation Features:**
- **Balanced Distribution:** Even spread across instruction types
- **Memory-Aware:** Address generation within memory bounds
- **Variable Management:** Unique variable name generation
- **Nested Loops:** Support for complex control structures
- **Configurable:** Instruction count and complexity parameters

### System Configuration

#### 12. Configuration System (`src/config.cpp`, `include/config.hpp`)
**Purpose:** Centralized parameter management for all system components

**Configuration Parameters:**
```cpp
struct Config {
    uint32_t cpuCount;                    // Number of CPU cores (1-128)
    SchedulingAlgorithm scheduler;        // FCFS or RoundRobin
    uint32_t quantumCycles;              // Time quantum for RR
    uint32_t processGenFrequency;        // Process generation frequency
    uint32_t minInstructions;            // Min instructions per process
    uint32_t maxInstructions;            // Max instructions per process
    uint32_t delayCyclesPerInstruction;  // Execution delay
    uint32_t max_overall_mem;            // Total system memory
    uint32_t mem_per_frame;              // Memory frame size
    uint32_t min_mem_per_proc;           // Min process memory
    uint32_t max_mem_per_proc;           // Max process memory
};
```

**Configuration Loading:**
- File-based configuration (`config.txt`)
- Parameter validation and range clamping
- Default value provision for missing parameters
- Error handling for invalid configurations

### User Interface

#### 13. Screen System (`src/screen.cpp`, `include/screen.hpp`)
**Purpose:** Interactive process management and monitoring interface

**Screen Commands:**
- `screen -s <name>` - Create new process
- `screen -r <name>` - Resume/view existing process
- `screen -ls` - List all processes with status
- `screen -f <file>` - Create process from instruction file
- `screen -c <instructions>` - Create custom process

**Process Monitoring:**
- Real-time process log viewing
- Process status and execution information
- Interactive terminal-based interface
- Process creation with multiple input methods

**Integration Features:**
- Memory utilization display
- CPU assignment information
- Process completion tracking
- Log timestamp formatting

### Utility Components

#### 14. Thread-Safe Queue (`include/thread_safe_queue.hpp`)
**Purpose:** Inter-thread communication for process scheduling

**Features:**
- `push()` - Thread-safe element addition with notification
- `try_pop()` - Non-blocking optional pop operation
- `wait_and_pop()` - Blocking pop with condition variable
- `shutdown()` - Graceful termination signaling
- Template-based for type flexibility

#### 15. Parser Utility (`src/parser.cpp`, `include/parser.hpp`)
**Purpose:** Command-line argument parsing with quote support

**Features:**
- Token splitting with quoted string support
- Escape sequence handling
- Integration with screen command processing
- Lightweight design for CLI parsing

## System Data Flow

### Process Lifecycle
1. **Creation:** Process generated by scheduler or screen command
2. **Parsing:** Instructions parsed into expression trees
3. **Memory Allocation:** Virtual memory space allocated
4. **Scheduling:** Process added to ready queue
5. **Execution:** CPU worker executes instructions with barrier sync
6. **Memory Operations:** Page faults handled during execution
7. **Completion:** Process moved to finished state
8. **Cleanup:** Memory deallocated and resources freed

### Barrier Synchronization Flow
1. **Scheduler Dispatch:** Signal all CPU workers to start execution
2. **Worker Execution:** Each worker executes assigned process
3. **Barrier Wait:** All workers report completion to scheduler
4. **Synchronization:** Scheduler waits for all workers to complete
5. **Next Cycle:** Process repeats for next execution cycle

### Memory Management Flow
1. **Virtual Access:** Process accesses virtual memory address
2. **Page Table Lookup:** Check page table for physical mapping
3. **Page Fault:** Handle missing pages with demand paging
4. **Frame Allocation:** Allocate physical frame or find victim
5. **Page Loading:** Load page from backing store if needed
6. **Memory Access:** Complete original memory operation
7. **Statistics Update:** Update vmstat counters

## Build and Execution

### Prerequisites
- C++20 compatible compiler (GCC 10+, Clang 12+, MSVC v19.29+)
- CMake version 3.20 or newer
- Standard C++ threading library support

### Build Process
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### Configuration File (`config.txt`)
```
num-cpu 2
scheduler "fcfs"
quantum-cycles 0
batch-process-freq 1
min-ins 4000
max-ins 4000
delay-per-exec 0
max-overall-mem 512
mem-per-frame 256
min-mem-per-proc 512
max-mem-per-proc 512
```

### Usage
```bash
./sim
~ initialize              # Load configuration
~ scheduler-start         # Begin process generation
~ screen -ls             # View process status
~ screen -s <name>       # Create specific process
~ process-smi            # Show memory usage
~ vmstat                 # Virtual memory statistics
~ report-util            # Generate system report
~ exit                   # Shutdown emulator
```

## Advanced Features

### Virtual Memory System
- **Demand Paging:** Pages loaded only when accessed
- **LRU Replacement:** Intelligent page eviction algorithm
- **Backing Store:** Persistent storage for swapped pages
- **Page Fault Handling:** Transparent fault resolution
- **Memory Protection:** Process isolation and boundary checking

### Multi-Core Processing
- **True Parallelism:** Each core runs in separate thread
- **Barrier Synchronization:** Coordinated execution cycles
- **Load Balancing:** Automatic process distribution
- **Core Affinity:** Process-to-core assignment tracking
- **Scalable Architecture:** Support for up to 128 cores

### Process Scheduling
- **Multiple Algorithms:** FCFS and Round-Robin support
- **Configurable Quantum:** Adjustable time slicing
- **Dynamic Generation:** Automatic process creation
- **Priority Handling:** Process state management
- **Statistics Tracking:** Comprehensive utilization monitoring

### Memory Statistics and Monitoring
- **Real-time Tracking:** Live memory utilization monitoring
- **Page Fault Statistics:** Detailed paging operation counts
- **Process Memory Maps:** Individual process memory usage
- **System Reports:** Comprehensive memory utilization reports
- **Vmstat Integration:** Unix-like virtual memory statistics

This OS emulator provides a comprehensive simulation of modern operating system features with particular strength in memory management, multi-core processing, and process scheduling. The recent vmstat integration enhances monitoring capabilities and provides detailed system performance insights.