#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <optional>
#include <memory>

namespace osemu {

    // MODIFIED: Inherit from std::enable_shared_from_this
    class PCB : public std::enable_shared_from_this<PCB> {
    public:
        // This is needed for std::unordered_set. We will stop using it soon.
        struct Hasher {
            std::size_t operator()(const PCB& p) const {
                return std::hash<std::string>()(p.processName);
            }
        };
        bool operator==(const PCB& other) const {
            return processName == other.processName;
        }


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
