#pragma once
#include "commands.hpp"
#include "config.hpp"
#include <string>
#include <vector>

void dispatch(Commands cmd,
              const std::vector<std::string>& args,
              Config& cfg);