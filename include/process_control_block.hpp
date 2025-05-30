#pragma once

#include <chrono>
#include <functional>
#include <string>

namespace osemu {

struct PCB {
    explicit PCB(std::string procName, size_t totalLines);

    void step();
    bool isComplete() const;
    std::string status() const;

    std::string processName;
    size_t currentInstruction{0};
    size_t totalInstructions;
    std::chrono::system_clock::time_point creationTime;
};

inline bool operator==(const PCB& a, const PCB& b) noexcept {
    return a.processName == b.processName;
}

}

namespace std {
template<>
struct hash<osemu::PCB> {
    size_t operator()(const osemu::PCB& pcb) const noexcept {
        return std::hash<std::string>()(pcb.processName);
    }
};
}
