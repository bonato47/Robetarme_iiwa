#include "maplanner.hpp"

void test(msm::back::state_machine<Planner> &p);

int main() {
  std::string context = "FSM - Planner";
  msm::back::state_machine<Planner> planner(context);

  test(planner);
  return 0;
}

void test(msm::back::state_machine<Planner> &p) {
  p.start();
  p.process_event(Start());
  p.process_event(WPReceived());
  p.process_event(WPInterpolated());
  p.process_event(BadFeasibility());
  p.process_event(TrajPlanned());
  p.process_event(TrajSent());
}
