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

  // clang-format off
  // Transition table for player
  using transition_table = mp11::mp_list<
      //    Start                 Event                 Next                  Action   Guard
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Stopped             , Start               , Waiting             , none   , none >,
      Row < Stopped             , Stop                , Stopped             , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Waiting             , Start               , Waiting             , none   , none >,
      Row < Waiting             , Stop                , Stopped             , none   , none >,
      Row < Waiting             , CadReceived         , Loading             , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Loading             , Stop                , Stopped             , none   , none >,
      Row < Loading             , CadLoaded           , ExtractingWaypoints , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < ExtractingWaypoints , Stop                , Stopped             , none   , none >,
      Row < ExtractingWaypoints , WaypointsExtracted  , Sending             , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Sending             , Stop                , Stopped             , none   , none >,
      Row < Sending             , WaypointsSent       , Waiting             , none   , none >
      //  +---------------------+---------------------+---------------------+--------+------+
  >;
  // clang-format on
};
