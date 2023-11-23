#include "maplanner_class.hpp"

void test(msm::back::state_machine<Planner> &p);

int main() {
  msm::back::state_machine<Planner> planner;
  test(planner);
  return 0;
}

void test(msm::back::state_machine<Planner> &p) {
  p.start();
  p.process_event(Start());
  p.process_event(CadReceived());
  p.process_event(CadLoaded());
  p.process_event(WaypointsExtracted());
  p.process_event(WaypointsSent());
  p.process_event(Start());
}
