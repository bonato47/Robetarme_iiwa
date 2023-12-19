#pragma once

#include <fstream>
#include <iostream>

// back-end
#include <boost/msm/back/state_machine.hpp>

// front-end
#include <boost/mp11/mpl.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include "log.hpp"

#define BOOST_LOG_LEVEL trivial::info

namespace msm = boost::msm;
namespace mp11 = boost::mp11;

using namespace std;
using namespace msm::front;

// List of FSM events
class Start {};
class Stop {};
class WPReceived {};
class WPInterpolated {};
class BadFeasibility {};
class TrajPlanned {};
class TrajSent {};

// front-end: define the FSM structure
class Planner : public msm::front::state_machine_def<Planner> {
 private:
  std::shared_ptr<Logs> log_;

  // List of FSM states
  class Stopped;
  class Waiting;
  class Interpolating;
  class Verifying;
  class Sending;

 public:
  Planner(
      const std::string& context,
      trivial::severity_level log_level = BOOST_LOG_LEVEL,
      bool log_to_file = false, const std::string& filename = ""
  ) {
    log_ = std::make_shared<Logs>(context, log_level, log_to_file, filename);
  };

  typedef Waiting initial_state;

  // clang-format off
  using transition_table = mp11::mp_list<
      //    Start                 Event                 Next                  Action   Guard
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Stopped             , Start               , Waiting             , none   , none >,
      Row < Stopped             , Stop                , Stopped             , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Waiting             , Stop                , Stopped             , none   , none >,
      Row < Waiting             , WPReceived          , Interpolating       , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Interpolating       , Stop                , Stopped             , none   , none >,
      Row < Interpolating       , WPInterpolated      , Verifying           , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Verifying           , Stop                , Stopped             , none   , none >,
      Row < Verifying           , BadFeasibility      , Interpolating       , none   , none >,
      Row < Verifying           , TrajPlanned         , Sending             , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Sending             , Stop                , Stopped             , none   , none >,
      Row < Sending             , TrajSent            , Waiting             , none   , none >
      //  +---------------------+---------------------+---------------------+--------+------+
  >;
  // clang-format on
};

// The list of FSM states
class Planner::Stopped : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    fsm.log_->info("Entering: Stopped");
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    fsm.log_->info("Leaving: Stopped");
  }
};

class Planner::Waiting : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    fsm.log_->info("Entering: Waiting");
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    fsm.log_->info("Leaving: Waiting");
  }
};

class Planner::Interpolating : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    fsm.log_->info("Entering: Interpolating");
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    fsm.log_->info("Leaving: Interpolating");
  }
};

class Planner::Verifying : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    fsm.log_->info("Entering: Verifying");
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    fsm.log_->info("Leaving: Verifying");
  }
};

class Planner::Sending : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    fsm.log_->info("Entering: Sending");
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    fsm.log_->info("Leaving: Sending");
  }
};
