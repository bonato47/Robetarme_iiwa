#include <boost/sml.hpp>
#include <assert.h>
#include <iostream>

namespace sml = boost::sml;

namespace MAPlanner {
  // events
  struct start {};
  struct stop {};
  struct cad_received {};
  struct cad_loaded {};
  struct waypoints_extracted {};
  struct waypoints_sent {};

  // guards

  // actions
  const auto enter_waiting = [] { std::cout << "entering: Waiting" << std::endl; };
  const auto send_ack = [] {};

  // Define states as structs
  struct idle {};
  struct loading {};
  struct extracting {};

  // states
  struct states {
    auto operator()() const noexcept {
      using namespace sml;
      // Use the state structs
      return make_transition_table(
        *state<idle> + event<cad_received> = state<loading>,
        state<loading> + sml::on_entry<_> / enter_waiting,
        state<loading> + sml::on_exit<_> / [] { std::cout << "exiting: Waiting" << std::endl; },
        state<loading> + event<cad_loaded> = state<extracting>
      );
    }
  };
}  // namespace MAPlanner
