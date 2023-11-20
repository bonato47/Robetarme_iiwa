#include "maplanner.h"

int main() {
  MAPlanner::test();
  return 0;
}

void MAPlanner::test() {
  planner p;
  p.start();
  p.process_event(start());
  pstate(p);
  p.process_event(cad_received());
  pstate(p);
  p.process_event(cad_loaded());
  pstate(p);
  p.process_event(waypoints_extracted());
  pstate(p);
  p.process_event(waypoints_sent());
  pstate(p);
  p.process_event(start());
  pstate(p);
}

void MAPlanner::pstate(planner const& p) {
  static char const* const state_names[] = {
      "Stopped", "Open", "Empty", "Playing", "Paused"
  };

  std::cout << " -> " << state_names[p.current_state()[0]] << std::endl;
}