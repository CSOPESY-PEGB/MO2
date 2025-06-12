#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <optional>
#include <memory>

namespace osemu {

    class PCB : public std::enable_shared_from_this<PCB> {
    public:
        PCB(std::string procName, size_t totalLines);

        void step();
        bool isComplete() const;
        std::string status() const;

        std::string processName;
        size_t currentInstruction;
        size_t totalInstructions;
        std::chrono::system_clock::time_point creationTime;

        // NEW: State tracking
        std::optional<int> assignedCore;
        std::chrono::system_clock::time_point finishTime;
    };
}
