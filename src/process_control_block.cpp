#include "process_control_block.hpp"

#include <format>
#include <sstream>

namespace osemu {

PCB::PCB(std::string procName, size_t totalLines)
    : processName(std::move(procName)),
      currentInstruction(0),
      totalInstructions(totalLines),
      creationTime(std::chrono::system_clock::now()),
      assignedCore(
          std::nullopt)  // Good practice to initialize optional to nullopt
{
  // The finishTime is not initialized here, only when the process actually
  // finishes.
}

// Definition for step()
void PCB::step() {
  if (currentInstruction < totalInstructions) {
    ++currentInstruction;
  }
}

// Definition for isComplete()
bool PCB::isComplete() const { return currentInstruction >= totalInstructions; }

// Definition for status() - This is the key one you need to implement as per
// the new requirements
std::string PCB::status() const {
  // Get the creation time as a formatted string
  auto creation_time_str = std::format("{:%m/%d/%Y %I:%M:%S%p}", creationTime);

  std::ostringstream oss;
  oss << processName << " (" << creation_time_str << ")  ";

  // Logic to display different status based on the process state
  if (isComplete()) {
    // If the process is done, show "Finished" and the final instruction count
    oss << "Finished           " << totalInstructions << " / "
        << totalInstructions;

  } else if (assignedCore.has_value()) {
    // If it's running on a core, show the core number and progress
    oss << "Core: " << *assignedCore << "            " << currentInstruction
        << " / " << totalInstructions;
  } else {
    // If it's not complete and not on a core, it's in the "Ready" state
    // You can customize this, but for now, we'll just show progress.
    // The UI screenshot doesn't show a "Ready" state, only "Running" or
    // "Finished". This part of the status string is for the "Running processes"
    // list.
    oss << "Ready (in queue)   " << currentInstruction << " / "
        << totalInstructions;
  }

  return oss.str();
}

}  // namespace osemu
