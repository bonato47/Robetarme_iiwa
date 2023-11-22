#include <maplanner_sml.hpp>

int main() {
  using namespace sml;
  sm<MAPlanner::states> sm;

//   assert(sm.is("idle"_s));

//   sm.process_event(MAPlanner::cad_received{});
//   assert(sm.is("loading"_s));

//   sm.process_event(MAPlanner::cad_loaded{});
//   assert(sm.is("extracting"_s));

  std::cout << "done" << std::endl;
}