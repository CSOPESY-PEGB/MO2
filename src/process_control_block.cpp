#include "process_control_block.hpp"

#include <format>
#include <sstream>

namespace osemu {

PCB::PCB(std::string procName, size_t totalLines)
    : processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(totalLines),
      creationTime(std::chrono::system_clock::now()) {}

void PCB::step() {
    if (currentInstruction < totalInstructions) {
        ++currentInstruction;
    }
}

bool PCB::isComplete() const { 
    return currentInstruction >= totalInstructions; 
}

std::string PCB::status() const {
    std::ostringstream oss;
    oss << processName << "  " << currentInstruction << "/" << totalInstructions
        << " Created at: "
        << std::format("{:%m/%d/%Y, %I:%M:%S %p}", creationTime);
    return oss.str();
}

}
