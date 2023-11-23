#pragma once

#include <fstream>
#include <iostream>

// back-end
#include <boost/msm/back/state_machine.hpp>

// front-end
#include <boost/mp11/mpl.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

namespace msm = boost::msm;
namespace mp11 = boost::mp11;

using namespace std;
using namespace msm::front;

// events
class Start {};
class Stop {};
class CadReceived {};
class CadLoaded {};
class WaypointsExtracted {};
class WaypointsSent {};

// The list of FSM states
class Stopped : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&) {
    std::cout << "entering: Stopped" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&) {
    std::cout << "leaving: Stopped" << std::endl;
  }
};

class Waiting : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&) {
    std::cout << "entering: Waiting" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&) {
    std::cout << "leaving: Waiting" << std::endl;
    test_function();
  }

 private:
  void test_function() { std::cout << "test_function" << std::endl; }
};

class Loading : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&) {
    std::cout << "entering: Loading" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&) {
    std::cout << "leaving: Loading" << std::endl;
  }
};

class ExtractingWaypoints : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "entering: ExtractingWaypoints" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "leaving: ExtractingWaypoints" << std::endl;
  }
};

class Sending : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&) {
    std::cout << "entering: Sending" << std::endl;
  }
  template <class Event, class FSM>
  void on_exit(Event const&, FSM&) {
    std::cout << "leaving: Sending" << std::endl;
  }
};

// state not defining any entry or exit
class Paused : public msm::front::state<> {};

// front-end: define the FSM structure
class Planner : public msm::front::state_machine_def<Planner> {
 public:
  // the initial state of the player SM. Must be defined
  typedef Waiting initial_state;

  // transition actions
  struct start_waiting_cad {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM&, SourceState&, TargetState&) {
      cout << "planner::start_waiting_cad" << endl;
    }
  };

  struct load_cad {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM&, SourceState&, TargetState&) {
      cout << "planner::load_cad" << endl;
    }
  };

  struct extract_waypoints {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM&, SourceState&, TargetState&) {
      cout << "planner::extract_waypoints" << endl;
    }
  };

  struct send_to_planner {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM&, SourceState&, TargetState&) {
      cout << "planner::send_to_planner" << endl;
    }
  };

  // clang-format off
  // Transition table for player
  using transition_table = mp11::mp_list<
      //    Start                 Event                 Next                  Action              Guard
      //  +---------------------+---------------------+---------------------+-------------------+------+
      Row < Stopped             , Start               , Waiting             , start_waiting_cad , none >,
      //  +---------------------+---------------------+---------------------+-------------------+------+
      Row < Waiting             , Start               , Waiting             , none              , none >,
      Row < Waiting             , CadReceived         , Loading             , load_cad          , none >,
      //  +---------------------+---------------------+---------------------+-------------------+------+
      Row < Loading             , CadLoaded           , ExtractingWaypoints , extract_waypoints , none >,
      //  +---------------------+---------------------+---------------------+-------------------+------+
      Row < ExtractingWaypoints , WaypointsExtracted  , Sending             , send_to_planner   , none >,
      //  +---------------------+---------------------+---------------------+-------------------+------+
      Row < Sending             , WaypointsSent       , Waiting             , start_waiting_cad , none >
      //  +---------------------+---------------------+---------------------+-------------------+------+
  >;
  // clang-format on
};
