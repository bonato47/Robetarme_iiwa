#ifndef MAPLANNER_HPP
#define MAPLANNER_HPP

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

namespace MAPlanner {
  struct ExtractingWaypoints;

  // events
  struct start {};
  struct stop {};
  struct cad_received {};
  struct cad_loaded {};
  struct waypoints_extracted {};
  struct waypoints_sent {};

  // front-end: define the FSM structure
  struct planner_ : public msm::front::state_machine_def<planner_> {
    // The list of FSM states
    struct Stopped : public msm::front::state<> {
      template <class Event, class FSM>
      void on_entry(Event const&, FSM&) {
        std::cout << "entering: Stopped" << std::endl;
      }

      template <class Event, class FSM>
      void on_exit(Event const&, FSM&) {
        std::cout << "leaving: Stopped" << std::endl;
      }
    };

    struct Waiting : public msm::front::state<> {
      template <class Event, class FSM>
      void on_entry(Event const&, FSM&) {
        std::cout << "entering: Waiting" << std::endl;
      }

      template <class Event, class FSM>
      void on_exit(Event const&, FSM&) {
        std::cout << "leaving: Waiting" << std::endl;
      }
    };

    struct Loading : public msm::front::state<> {
      template <class Event, class FSM>
      void on_entry(Event const&, FSM&) {
        std::cout << "entering: Loading" << std::endl;
      }

      template <class Event, class FSM>
      void on_exit(Event const&, FSM&) {
        std::cout << "leaving: Loading" << std::endl;
      }
    };

    struct ExtractingWaypoints : public msm::front::state<> {};

    struct Sending : public msm::front::state<> {
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
    struct Paused : public msm::front::state<> {};

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
        Row < Stopped             , start               , Waiting             , start_waiting_cad , none >,
        //  +---------------------+---------------------+---------------------+-------------------+------+
        Row < Waiting             , cad_received        , Loading             , load_cad          , none >,
        //  +---------------------+---------------------+---------------------+-------------------+------+
        Row < Loading             , cad_loaded          , ExtractingWaypoints , extract_waypoints , none >,
        //  +---------------------+---------------------+---------------------+-------------------+------+
        Row < ExtractingWaypoints , waypoints_extracted , Sending             , send_to_planner   , none >,
        //  +---------------------+---------------------+---------------------+-------------------+------+
        Row < Sending             , waypoints_sent      , Waiting             , start_waiting_cad , none >
        //  +---------------------+---------------------+---------------------+-------------------+------+
    >;
    // clang-format on

    // Replaces the default no-transition response.
    template <class FSM, class Event>
    void no_transition(Event const& e, FSM&, int state) {
      std::cout << "no transition from state " << state << " on event "
                << typeid(e).name() << std::endl;
    }
  };
}  // namespace MAPlanner

#endif  // MAPLANNER_HPP