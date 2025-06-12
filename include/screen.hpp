#pragma once

#include <vector>
#include <string>


namespace osemu {
    class Scheduler;
    void screen(std::vector<std::string>& args, Scheduler& scheduler);
}