#include <iostream>
#include <atomic>
#include <chrono>
#include <functional>
#include <vector>
#include <algorithm>
#include <mutex>
#include <thread>

#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>

#include <gtkmm.h>

#include "ai/driver.hpp"
#include "ai/model/world.hpp"
#include "ai/model/updater/world.hpp"
#include "ai/model/updater/refbox.hpp"
#include "ai/receiver/refbox.hpp"
#include "ai/receiver/vision.hpp"
#include "ai/sender/grsim.hpp"
#include "ai/game/action/move.hpp"
#include "ai/filter/va.hpp"
#include "ai/filter/observer/ball.hpp"
#include "ai/controller/feedback.hpp"
#include "ai/util/math/affine.hpp"

using namespace std::chrono_literals;
using namespace std::string_literals;

namespace observer   = ai::filter::observer;
namespace controller = ai::controller;
namespace filter     = ai::filter;
namespace game       = ai::game;
namespace model      = ai::model;
namespace receiver   = ai::receiver;
namespace sender     = ai::sender;
namespace util       = ai::util;

// Visionの設定
static constexpr char visionAddress[] = "224.5.23.2";
static constexpr short visionPort     = 10006;

// Refboxの設定
static constexpr char globalRefboxAddress[] = "224.5.23.1";
static constexpr short globalRefboxPort     = 10003;
static constexpr char localRefboxAddress[]  = "224.5.23.8";
static constexpr short localRefboxPort      = 10088;

// Senderの設定
static constexpr bool isGrsim           = true;
static constexpr char grsimAddress[]    = "127.0.0.1";
static constexpr short grsimCommandPort = 20011;

// 周期の設定
using fps60 = std::chrono::duration<util::TimePointType::rep, std::ratio<1, 60>>;
static constexpr auto cycle = std::chrono::duration_cast<util::DurationType>(fps60{1});

class gameRunner {
public:
  gameRunner(model::updater::world& _world, model::updater::refbox& _refbox,
             std::shared_ptr<sender::base>& _sender)
      : running_{false},
        gameThread_{},
        driverThread_{},
        teamColor_(model::teamColor::Yellow),
        updaterWorld_(_world),
        updaterRefbox_(_refbox),
        isGlobalRefbox_{true},
        sender_(_sender),
        driver_(driverIo_, cycle, updaterWorld_, teamColor_),
        activeRobots_({
            0u,
            1u,
            2u,
            3u,
            4u,
            5u,
            6u,
            7u,
        }) {
    driverThread_ = std::thread([this] {
      try {
        driverIo_.run();
      } catch (const std::exception& e) {
        std::cout << boost::format("exception at driver_thread\n\t%1%") % e.what() << std::endl;
      } catch (...) {
        std::cout << boost::format("unknownexception at driver thread") << std::endl;
      }
    });
    for (auto id : activeRobots_) {
      constexpr auto cycleCount = std::chrono::duration<double>(cycle).count();
      auto controller           = std::make_unique<controller::feedback>(cycleCount, world_);
      driver_.registerRobot(id, std::move(controller), sender_);
    }
  }

  ~gameRunner() {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    running_ = false;
    if (gameThread_.joinable()) gameThread_.join();
    driverIo_.stop();
    if (driverThread_.joinable()) driverThread_.join();
  }

  void start() {
    if (!gameThread_.joinable()) {
      running_    = true;
      gameThread_ = std::thread([this] { mainLoop(); });
    }
  }

  void stop() {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    running_ = false;
    gameThread_.join();
    std::cout << "game stopped!" << std::endl;
  }

  std::vector<uint32_t> activeRobots() const {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return activeRobots_;
  }

  void activeRobots(const std::vector<uint32_t>& _ids) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    bool changed = false;
    // register new robots
    for (auto id : _ids) {
      if (!driver_.registered(id)) {
        constexpr auto cycleCount = std::chrono::duration<double>(cycle).count();
        auto controller           = std::make_unique<controller::feedback>(cycleCount, world_);
        driver_.registerRobot(id, std::move(controller), sender_);
        changed = true;
      }
    }
    // unregister robots
    for (auto id : activeRobots_) {
      if (std::find(_ids.cbegin(), _ids.cend(), id) == _ids.cend()) {
        driver_.unregisterRobot(id);
        changed = true;
      }
    }
    if (changed) {
      std::stringstream ss{};
      ss << "activeRobots changed: { ";
      std::copy(activeRobots_.cbegin(), activeRobots_.cend(),
                std::ostream_iterator<uint32_t>(ss, ", "));
      ss << "} -> { ";
      std::copy(_ids.cbegin(), _ids.cend(), std::ostream_iterator<uint32_t>(ss, ", "));
      ss << "}";
                        std::cout << ss.str() << std::endl;

      activeRobots_ = _ids;
    }
  }

  void teamColor(model::teamColor _color) {
    if (teamColor_ != _color) {
      driver_.teamColor(_color);
      teamColor_ = _color;
    }
  }

  bool isGlobalRefbox() const {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return isGlobalRefbox_;
  }

  void useGlobalRefbox(bool _isGlobal) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    if (isGlobalRefbox_ != _isGlobal) {
      std::cout << "switched to "s + (_isGlobal ? "global"s : "local"s) + " refbox"
                << std::endl;
      isGlobalRefbox_ = _isGlobal;
    }
  }

  void transformationMatrix(double _x, double _y, double _theta) {
    const auto mat = util::math::makeTransformationMatrix(_x, _y, _theta);
    updaterWorld_.transformationMatrix(mat);
    updaterRefbox_.transformationMatrix(mat);
  }

private:
  void mainLoop() {
    std::cout << "game start!!"<< std::endl;
    ai::util::TimePointType prevTime{};

    while (running_) {
      try {
        const auto currentTime = util::ClockType::now();
        if (currentTime - prevTime < cycle) {
          std::this_thread::sleep_for(1ms);
        } else {
          std::unique_lock<std::shared_timed_mutex> lock(mutex_);
          const auto prevCmd = refbox_.command();
          world_              = updaterWorld_.value();
          refbox_             = updaterRefbox_.value();

          const auto currentCmd  = refbox_.command();
          const auto penaltyKick = teamColor_ == model::teamColor::Yellow
                                       ? model::refbox::gameCommand::PreparePenaltyYellow
                                       : model::refbox::gameCommand::PreparePenaltyBlue;

          // stopgameとpenalty_kickでは速度を落として安定制御
          if (currentCmd != prevCmd) {
            if (currentCmd == model::refbox::gameCommand::Stop ||
                /*current_cmd == model::refbox::game_command::ball_placement_yellow ||
                  current_cmd == model::refbox::game_command::ball_placement_blue ||*/
                (prevCmd == penaltyKick &&
                 currentCmd == model::refbox::gameCommand::NormalStart) ||
                currentCmd == model::refbox::gameCommand::PreparePenaltyYellow ||
                currentCmd == model::refbox::gameCommand::PreparePenaltyBlue) {
              driver_.velocityLimit(500);
            } else if (currentCmd == model::refbox::gameCommand::BallPlacementYellow ||
                       currentCmd == model::refbox::gameCommand::BallPlacementBlue) {
              driver_.velocityLimit(1000);
            } else {
              driver_.velocityLimit(std::numeric_limits<double>::max());
            }
          }

          const auto visibleRobots =
              static_cast<bool>(teamColor_) ? world_.robotsYellow() : world_.robotsBlue();
          prevTime = currentTime;
        }
      } catch (const std::exception& e) {
        std::cout << boost::format("exception at game_thread\n\t%1%") % e.what() << std::endl;
      } catch (...) {
        std::cout << "unknown exception at game_thread" << std::endl;
      }
    }
  }
  mutable std::shared_timed_mutex mutex_;
  std::atomic<bool> running_;
  std::thread gameThread_;
  std::thread driverThread_;
  model::teamColor teamColor_;
  model::updater::world& updaterWorld_;
  model::updater::refbox& updaterRefbox_;
  std::atomic<bool> isGlobalRefbox_;

  boost::asio::io_service driverIo_;
  std::shared_ptr<sender::base> sender_;
  ai::driver driver_;

  model::world world_;
  model::refbox refbox_;
  std::vector<uint32_t> activeRobots_;
};

// Gameを行うクラス
// --------------------------------
class gameWindow final : public Gtk::Window {
public:
  gameWindow(model::updater::world& _world, gameRunner& _runner)
      : updaterWorld_(_world), runner_(_runner) {
    set_border_width(10);
    set_default_size(800, 500);
    set_title("AI Client");

    {
      // left widgets
      color_.set_label("Team color");
      colorR1_.set_label("Yellow");
      colorR2_.set_label("Blue");
      colorR2_.join_group(colorR1_);
      colorBox_.set_border_width(4);
      colorBox_.pack_start(colorR1_);
      colorBox_.pack_start(colorR2_);
      color_.add(colorBox_);
      left_.pack_start(color_, Gtk::PACK_SHRINK, 4);

      dir_.set_label("Attack direction");
      dirR1_.set_label("Right");
      dirR2_.set_label("Left");
      dirR2_.join_group(dirR1_);
      dirBox_.set_border_width(4);
      dirBox_.pack_start(dirR1_);
      dirBox_.pack_start(dirR2_);
      dir_.add(dirBox_);
      left_.pack_start(dir_, Gtk::PACK_SHRINK, 4);

      robots_.set_label("Active robots");
      robotsIdBox_.set_border_width(4);
      robotsIdBox_.set_max_children_per_line(6);
      for (auto i = 0u; i < 12; ++i) {
        robotsCb_.emplace_back(std::to_string(i));
        robotsIdBox_.add(robotsCb_.back());
      }
      apply_.set_label("Apply");
      apply_.set_border_width(4);
      apply_.set_sensitive(false);
      apply_.signal_clicked().connect(
          sigc::mem_fun(*this, &gameWindow::handleChangeActiveRobots));
      robotsBox_.pack_start(robotsIdBox_);
      robotsBox_.pack_start(apply_);
      robots_.add(robotsBox_);
      left_.pack_start(robots_, Gtk::PACK_SHRINK, 4);

      camera_.set_label("Camera");
      cameraIdBox_.set_border_width(4);
      cameraIdBox_.set_max_children_per_line(6);
      for (auto i = 0u; i < 8; ++i) {
        cameraCb_.emplace_back(std::to_string(i));
        cameraIdBox_.add(cameraCb_.back());
      }
      camera_.add(cameraIdBox_);
      left_.pack_start(camera_, Gtk::PACK_SHRINK, 4);

      refbox_.set_label("Refbox");
      refboxR1_.set_label("Global");
      refboxR2_.set_label("Local");
      refboxR2_.join_group(refboxR1_);
      refboxBox_.set_border_width(4);
      refboxBox_.pack_start(refboxR1_);
      refboxBox_.pack_start(refboxR2_);
      refbox_.add(refboxBox_);
      left_.pack_start(refbox_, Gtk::PACK_SHRINK, 4);

      start_.set_label("start");
      start_.set_sensitive(false);
      start_.signal_clicked().connect(sigc::mem_fun(*this, &gameWindow::handleStartStop));
      left_.pack_end(start_, Gtk::PACK_SHRINK);

      left_.set_size_request(320, -1);
      center_.pack_start(left_, Gtk::PACK_SHRINK, 10);
    }

    {
      // right widgets
			state_.set_label("Game State");
			auto textArea = stateText_.get_buffer();
			textArea->set_text( "Stop Game" );
			stateText_.set_editable(false);
			stateBox_.set_border_width(4);
			stateBox_.pack_start(stateText_);
			state_.add(stateBox_);
			right_.pack_start(state_,  Gtk::PACK_SHRINK, 4);
      initTree();
			right_.pack_start(tree_, Gtk::PACK_EXPAND_WIDGET, 10);
      center_.pack_end(right_, Gtk::PACK_EXPAND_WIDGET, 10);
    }

    add(center_);
    show_all_children();

    initRadioButtons();
    initCheckButtons();
  }

  void ready() {
    start_.set_sensitive(true);
  }

private:
  void initTree() {
    treestore_ = Gtk::TreeStore::create(model_);
    tree_.set_model(treestore_);
    tree_.append_column_editable("Agent/Action", model_.name_);
    tree_.append_column("Robot ID", model_.id_);
		tree_.append_column("Battery[V]", model_.battery_);
		tree_.append_column("Kicker[V]", model_.kicker_);
		tree_.append_column("Ball Sensor", model_.ballSensor_);

    auto r1_1          = *(treestore_->append());
    r1_1[model_.name_] = "no_op";
    r1_1[model_.id_]   = 0;
		r1_1[model_.battery_] = 22.2;
		r1_1[model_.kicker_] = 200;
		r1_1[model_.ballSensor_] = "Inactive";
    auto r1_2          = *(treestore_->append());
    r1_2[model_.name_] = "no_op";
    r1_2[model_.id_]   = 1;
		r1_2[model_.battery_] = 22.2;
		r1_2[model_.kicker_] = 200;
		r1_2[model_.ballSensor_] = "Inactive";
    auto r1_3          = *(treestore_->append());
    r1_3[model_.name_] = "no_op";
    r1_3[model_.id_]   = 2;
		r1_3[model_.battery_] = 22.2;
		r1_3[model_.kicker_] = 200;
    r1_3[model_.ballSensor_] = "Inactive";
    
		tree_.expand_all();
  }

  void initRadioButtons() {
    dirR1_.signal_toggled().connect([this] {
      runner_.transformationMatrix(
          0.0, 0.0, (dirR1_.get_active() ? 0.0 : boost::math::double_constants::pi));
    });

    colorR1_.signal_toggled().connect([this] {
      if (colorR1_.get_active()) {
        runner_.teamColor(model::teamColor::Yellow);
      } else {
        runner_.teamColor(model::teamColor::Blue);
      }
    });

    if (runner_.isGlobalRefbox()) {
      refboxR1_.set_active(true);
    } else {
      refboxR2_.set_active(true);
    }
    refboxR1_.signal_toggled().connect([this] {
      if (refboxR1_.get_active()) {
        runner_.useGlobalRefbox(true);
      } else {
        runner_.useGlobalRefbox(false);
      }
    });
  }

  void initCheckButtons() {
    auto robots = runner_.activeRobots();
    for (auto i = 0u; i < 12; ++i) {
      auto& cb = robotsCb_.at(i);
      cb.set_active(std::find(robots.cbegin(), robots.cend(), i) != robots.cend());
      cb.signal_toggled().connect(sigc::mem_fun(*this, &gameWindow::handleActiveRobotsChanged));
    }

    for (auto i = 0u; i < 8; ++i) {
      auto& cb = cameraCb_.at(i);
      cb.set_active(updaterWorld_.isCameraEnabled(i));
      cb.signal_toggled().connect([id = i, this] {
        auto& cb = cameraCb_.at(id);
        if (cb.get_active()) {
          updaterWorld_.enableCamera(id);
          std::cout << boost::format("camera%1% enabled") % id << std::endl;
        } else {
          updaterWorld_.disableCamera(id);
          std::cout << boost::format("camera%1% disabled") % id << std::endl;
        }
      });
    }
  }

  void handleStartStop() {
    if (start_.get_label() == "start") {
      runner_.start();
      start_.set_label("stop");
    } else {
      runner_.stop();
      start_.set_label("start");
    }
  }

  void handleActiveRobotsChanged() {
    apply_.set_sensitive(true);

		
  }

  void handleChangeActiveRobots() {
    std::vector<uint32_t> robots{};
    for (auto i = 0u; i < robotsCb_.size(); ++i) {
      if (robotsCb_[i].get_active()) {
        robots.push_back(i);
      }
    }
    runner_.activeRobots(robots);
    apply_.set_sensitive(false);
  }

  class treeModel : public Gtk::TreeModel::ColumnRecord {
  public:
    treeModel() {
      add(name_);
      add(id_);
			add(battery_);
			add(kicker_);
			add(ballSensor_);
    }

    Gtk::TreeModelColumn<Glib::ustring> name_;
    Gtk::TreeModelColumn<uint32_t> id_;
		Gtk::TreeModelColumn<double> battery_;
		Gtk::TreeModelColumn<uint32_t> kicker_;
		Gtk::TreeModelColumn<std::string> ballSensor_;
  };

  treeModel model_;

  Gtk::Frame color_, dir_, robots_, refbox_, camera_, state_;
  Gtk::VBox left_, robotsBox_, right_;
  Gtk::HBox center_, colorBox_, dirBox_, refboxBox_, stateBox_;
  Gtk::RadioButton colorR1_, colorR2_, dirR1_, dirR2_, refboxR1_, refboxR2_;
  std::vector<Gtk::CheckButton> robotsCb_, cameraCb_;
  Gtk::FlowBox robotsIdBox_, cameraIdBox_;
  Gtk::Button apply_, start_;
  Gtk::TreeView tree_;
  Glib::RefPtr<Gtk::TreeStore> treestore_;
	Gtk::TextView stateText_;

  model::updater::world& updaterWorld_;
  gameRunner& runner_;
};

auto main(int argc, char** argv) -> int {
  boost::asio::io_service receiverIo{};

  try {
    std::cout << boost::format("cycle: %1%") % std::chrono::duration<double>(cycle).count()
              << std::endl;

    model::updater::world updaterWorld{};

    // 速度・加速度計算機
    updaterWorld.robotsBlueUpdater().setDefaultFilter<filter::va<model::robot>>();
    updaterWorld.robotsYellowUpdater().setDefaultFilter<filter::va<model::robot>>();

    // ボールの状態オブザーバを使う
    updaterWorld.ballUpdater().setFilter<filter::observer::ball>(model::ball{},
                                                                 util::ClockType::now());

    // Vision receiverの設定
    std::atomic<bool> visionReceived{false};
    receiver::vision vision{receiverIo, "0.0.0.0", visionAddress, visionPort};
    vision.onReceive([&updaterWorld, &visionReceived](auto&& p) {
      if (!visionReceived) {
        // 最初に受信したときにメッセージを表示する
        std::cout << "vision packet received!" << std::endl;
        visionReceived = true;
      }
      updaterWorld.update(std::forward<decltype(p)>(p));
    });

    // Refbox receiverの設定
    std::atomic<bool> refboxReceived{false};
    model::updater::refbox updaterRefbox{};
    receiver::refbox refboxReceiver{receiverIo, "0.0.0.0", globalRefboxAddress,
                                    globalRefboxPort};
    refboxReceiver.onReceive([&updaterRefbox, &refboxReceived](auto&& p) {
      if (!refboxReceived) {
        // 最初に受信したときにメッセージを表示する
        std::cout << "refbox (global) packet received!" << std::endl;
        refboxReceived = true;
      }

      updaterRefbox.update(std::forward<decltype(p)>(p));
    });

		// Receiver, Driverなどをぶんまわすスレッド
		std::thread ioThread{};

    // receiver_ioに登録されたタスクを別スレッドで開始
    ioThread = std::thread{[&receiverIo] {
      try {
        receiverIo.run();
      } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
      }
    }};

    // Senderの設定
    std::shared_ptr<sender::base> sender{};
    sender = std::make_shared<sender::grsim>(receiverIo, grsimAddress, grsimCommandPort);
    std::cout << boost::format("sender: grSim (%1%:%2%)") % grsimAddress % grsimCommandPort
              << std::endl;

    auto app = Gtk::Application::create(argc, argv, "org.gtkmm.example");
    gameRunner runner{updaterWorld, updaterRefbox, sender};
    gameWindow gw{updaterWorld, runner};
    gw.show();

    std::thread wait([&runner, &gw, &visionReceived] {
      // Visionから値を受信するまで待つ
      do {
        std::this_thread::sleep_for(500ms);
      } while (!visionReceived);

      // 状態オブザーバなどの値が収束するまで待つ
      std::this_thread::sleep_for(1s);
      gw.ready();
    });

    app->run(gw);

		wait.detach();
    receiverIo.stop();
    ioThread.join();
  } catch (std::exception& e) {
    std::cout << "exception" << std::endl << e.what() << std::endl;
  }
}
