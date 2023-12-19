#pragma once

#include <fstream>
#include <iostream>

#include "stl_reader.h"

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

using namespace msm::front;

// List of FSM events
class Start {};
class Stop {};
class CadReceived {};
class CadLoaded {};
class WPExtracted {};
class WPSent {};

// front-end: define the FSM structure
class WaypointsExtraction
    : public msm::front::state_machine_def<WaypointsExtraction> {
 private:
  std::shared_ptr<Logs> log_;

  // List of FSM states
  class Stopped;
  class Waiting;
  class Loading;
  class Extracting;
  class Sending;

 public:
  WaypointsExtraction(
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
      Row < Waiting             , Start               , Waiting             , none   , none >,
      Row < Waiting             , Stop                , Stopped             , none   , none >,
      Row < Waiting             , CadReceived         , Loading             , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Loading             , Stop                , Stopped             , none   , none >,
      Row < Loading             , CadLoaded           , Extracting          , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Extracting          , Stop                , Stopped             , none   , none >,
      Row < Extracting          , WPExtracted         , Sending             , none   , none >,
      //  +---------------------+---------------------+---------------------+--------+------+
      Row < Sending             , Stop                , Stopped             , none   , none >,
      Row < Sending             , WPSent              , Waiting             , none   , none >
      //  +---------------------+---------------------+---------------------+--------+------+
  >;
  // clang-format on
};

// The list of FSM states
class WaypointsExtraction::Stopped : public msm::front::state<> {
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

class WaypointsExtraction::Waiting : public msm::front::state<> {
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

class WaypointsExtraction::Loading : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    fsm.log_->info("Entering: Loading");

    std::string file_path = "/home/ros/ros_ws/src/metal_additive_manufacturing/data/test_data.stl";
    read_cad_file(file_path);
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    fsm.log_->info("Leaving: Loading");
  }

 private:
  void read_cad_file(std::string stl_file) {
    try {
      stl_reader::StlMesh<float, unsigned int> mesh(stl_file);

      for (size_t itri = 0; itri < mesh.num_tris(); ++itri) {
        std::cout << "coordinates of triangle " << itri << ": ";
        for (size_t icorner = 0; icorner < 3; ++icorner) {
          const float* c = mesh.tri_corner_coords(itri, icorner);
          std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
        }
        std::cout << std::endl;

        const float* n = mesh.tri_normal(itri);
        std::cout << "normal of triangle " << itri << ": "
                  << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
      }
    } catch (std::exception& e) {
      std::cout << e.what() << std::endl;
    }
  }
};

class WaypointsExtraction::Extracting : public msm::front::state<> {
 public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    fsm.log_->info("Entering: Extracting");
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    fsm.log_->info("Leaving: Extracting");
  }
};

class WaypointsExtraction::Sending : public msm::front::state<> {
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
