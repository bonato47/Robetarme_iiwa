#include "maplanner.hpp"

void test(msm::back::state_machine<MAPlanner::planner_> &p);

int main() {
  msm::back::state_machine<MAPlanner::planner_> planner;
  msm::back::state_machine<MAPlanner::planner_> planner2;
  planner.start();

  std::cout << "Initial state: " << planner.current_state()[0] << std::endl;
  // std::cout << "Initial state: " << planner.current_state()[0] == MAPlanner::planner_::Stopped << std::endl;

  test(planner);

  std::cout << "Initial state: " << planner2.current_state()[0] << std::endl;
  // std::cout << "Initial state: " << planner2.get_current_state() << std::endl;
  test(planner2);
  return 0;
}

void test(msm::back::state_machine<MAPlanner::planner_> &p) {
  p.start();

  // std::cout << "Initial state: " << p.current_state()[0] << std::endl;
  // std::cout << "Initial state: " << MAPlanner::get_current_state() << std::endl;
  p.process_event(MAPlanner::start());
  p.process_event(MAPlanner::cad_received());
  p.process_event(MAPlanner::cad_loaded());
  // std::cout << "Initial state: " << p.current_state()[0] << std::endl;
  // std::cout << "Initial state: " << MAPlanner::get_current_state() << std::endl;
  p.process_event(MAPlanner::waypoints_extracted());
  p.process_event(MAPlanner::waypoints_sent());
  // std::cout << "Initial state: " << p.current_state()[0] << std::endl;
  // std::cout << "Initial state: " << MAPlanner::get_current_state() << std::endl;
  p.process_event(MAPlanner::start());
}
