// Build the real window offscreen (--no-rviz path: Ogre has no GL there),
// drive it through the gating states, click every button, count what came out
// on /operator_key. C++ port of the former test_gui_smoke.py — same phases.
// Run:  QT_QPA_PLATFORM=offscreen ./build/raon_operator_gui/test_gui_smoke
#include <QApplication>
#include <QImage>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "../src/main_window.hpp"
#include "../src/ros_link.hpp"
#include "../src/state.hpp"

using raon_gui::State;

static State full()
{
  State s;
  s.q = {0, 0, 0, 0, 0, 0};
  s.servo = s.ctrl = s.grav = s.home = s.lock = s.menu = true;
  return s;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<raon_gui::RosLink>(
    "/img", "/detections", "/operator_key", "/robot_state");

  QApplication app(argc, argv);
  qRegisterMetaType<QImage>();
  qRegisterMetaType<raon_gui::State>();

  raon_gui::MainWindow w(
    node.get(), {"/img", "/robot_state", "/operator_key"},
    std::weak_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>());

  std::vector<std::string> seen;
  auto echo = std::make_shared<rclcpp::Node>("echo");
  auto sub = echo->create_subscription<std_msgs::msg::String>(
    "/operator_key", 10,
    [&seen](std_msgs::msg::String::ConstSharedPtr m) {seen.push_back(m->data);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(echo);

  const auto & btns = w.buttons();
  assert(btns.size() == 22);   // 2 stop + 20 letters; digits replaced by
                               // class buttons (own gating, tested below)

  // 1) before any /robot_state: EVERYTHING disabled, E-STOP included
  for (const auto & [k, b] : btns) {assert(!b->isEnabled());}

  // frames drive the labels (two: FPS is an interval)
  w.onFrame(QImage(848, 480, QImage::Format_BGR888));
  assert(w.resLabel()->text() == "848 × 480");
  w.onFrame(QImage(848, 480, QImage::Format_BGR888));
  assert(w.fpsLabel()->text() != "NO SIGNAL");

  // 2) fully permissive: all enabled; click all, one spin burst per click
  // (/operator_key QoS is depth 1 on purpose — a burst legitimately drops)
  w.onState(full());
  for (const auto & [k, b] : btns) {assert(b->isEnabled());}
  for (const auto & [k, b] : btns) {
    b->click();
    for (int i = 0; i < 6; ++i) {exec.spin_some(std::chrono::milliseconds(20));}
  }
  assert(seen.size() == btns.size());
  const char * allowed = "rhjgfcitkbpvoxensmadlw0123456789";
  for (const auto & k : seen) {
    assert(k.size() == 1 && std::strchr(allowed, k[0]) != nullptr);
    assert(k != "q");
  }

  // 3) tracking: exactly DoInput's whitelist stays enabled
  State tr = full();
  tr.track = true;
  w.onState(tr);
  const char * track_ok = "ojexgp";
  for (const auto & [k, b] : btns) {
    assert(b->isEnabled() == (std::strchr(track_ok, k) != nullptr));
  }

  // 4) no lock → v/o die
  State s4 = full();
  s4.menu = false;
  s4.lock = false;
  w.onState(s4);
  assert(!btns.at('v')->isEnabled() && !btns.at('o')->isEnabled() &&
    btns.at('h')->isEnabled());

  // 4b) object-selection gating + digit mapping. Menu open with two classes
  // (menu order = bridge numbering): only those two light up; clicks send the
  // POSITION digit; AUTO = N+1; 취소 = 0; HOME 기록 never enables.
  State sm = full();
  sm.menu_items = {"apple", "tennis_ball"};
  w.onState(sm);
  assert(w.classButtons().at("apple")->isEnabled());
  assert(w.classButtons().at("tennis_ball")->isEnabled());
  assert(!w.classButtons().at("orange")->isEnabled());
  assert(!w.classButtons().at("banana")->isEnabled());
  assert(!w.classButtons().at("mustard_bottle")->isEnabled());
  assert(w.autoButton()->isEnabled() && w.cancelButton()->isEnabled());
  assert(!w.homeRecordButton()->isEnabled());

  seen.clear();
  auto click_spin = [&](QPushButton * b) {
      b->click();
      for (int i = 0; i < 6; ++i) {exec.spin_some(std::chrono::milliseconds(20));}
    };
  click_spin(w.classButtons().at("apple"));
  click_spin(w.classButtons().at("tennis_ball"));
  click_spin(w.autoButton());
  click_spin(w.cancelButton());
  assert((seen == std::vector<std::string>{"1", "2", "3", "0"}));

  // menu closed again → every selection control dies
  w.onState(s4);
  for (const auto & [cls, b] : w.classButtons()) {assert(!b->isEnabled());}
  assert(!w.autoButton()->isEnabled() && !w.cancelButton()->isEnabled());

  // 5) robot link dead: everything off again — but the QUIT button (outside
  // buttons_) must survive: closing a dead console is always legitimate.
  w.forceStateStale();
  w.checkLink();
  for (const auto & [k, b] : btns) {assert(!b->isEnabled());}
  {
    int quit_alive = 0;
    for (auto * b : w.findChildren<QPushButton *>()) {
      if (b->text() == "콘솔 종료") {quit_alive += b->isEnabled() ? 1 : 0;}
    }
    assert(quit_alive == 1);
  }

  // 6) /operator_msg lines land in the log pane, menu multi-line intact
  w.onRobotMsg("[PickBridge] ---- visible objects ----\n[PickBridge]   1) apple");
  assert(w.robotLog()->toPlainText().contains("1) apple"));

  printf("PASS: 22 key buttons + object selection; init/full/tracking/menu/"
    "dead gating match DoInput; digit mapping 1/2/AUTO/cancel exact; "
    "no 'q'; HOME-record dead; quit ungated; robot log live; res/fps live\n");
  rclcpp::shutdown();
  return 0;
}
