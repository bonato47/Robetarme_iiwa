#include "waypoints_extraction.hpp"

void test(msm::back::state_machine<WaypointsExtraction> &p);

int main() {
  std::string context = "FSM - WaypointsExtraction";
  msm::back::state_machine<WaypointsExtraction> wp_extraction(context);

  test(wp_extraction);
  return 0;
}

void test(msm::back::state_machine<WaypointsExtraction> &wp_extraction) {
  wp_extraction.start();
  wp_extraction.process_event(Start());
  wp_extraction.process_event(CadReceived());
  wp_extraction.process_event(CadLoaded());
  wp_extraction.process_event(WPExtracted());
  wp_extraction.process_event(WPSent());
  wp_extraction.process_event(Start());
}
