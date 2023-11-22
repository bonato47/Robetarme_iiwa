#define BOOST_TEST_MODULE TestMAPlanner
#include <boost/test/included/unit_test.hpp>

#include "maplanner.hpp"

BOOST_AUTO_TEST_CASE(TestInitialState) {
  msm::back::state_machine<MAPlanner::planner_> planner;
  planner.start();

  // Check initial state
//   BOOST_CHECK_EQUAL(
//       planner.current_state()[0],
//       msm::back::state_machine<MAPlanner::planner_>::Waiting
//   );
}