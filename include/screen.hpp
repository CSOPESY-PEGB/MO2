#ifndef OSEMU_SCREEN_H_
#define OSEMU_SCREEN_H_

#include <string>
#include <vector>

namespace osemu {

class Scheduler;

void screen(std::vector<std::string>& args, Scheduler& scheduler);

}  // namespace osemu

#endif  // OSEMU_SCREEN_H_
