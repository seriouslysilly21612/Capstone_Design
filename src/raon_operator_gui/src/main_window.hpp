// Operator console — ONE window: video+bbox, embedded RViz, res/FPS, gated
// buttons. C++ port of the former main_window.py; the button set, the Korean
// labels, the gating rules and the link-dead behaviour are carried over 1:1
// (the gating itself lives in state.hpp so test_logic can hit it without Qt).
#ifndef RAON_GUI_MAIN_WINDOW_HPP
#define RAON_GUI_MAIN_WINDOW_HPP

#include <QImage>
#include <QMainWindow>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include "state.hpp"

class QLabel;
class QPushButton;
class QGroupBox;
class QPlainTextEdit;

namespace raon_gui
{

class RosLink;
class VideoPane;

struct Topics
{
  std::string image, state, key;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  // rviz_node empty (expired weak_ptr) => no RViz pane. That is the offscreen
  // test path — Ogre needs a GL context the offscreen platform cannot give —
  // and the --no-rviz escape hatch on machines with broken GL.
  MainWindow(
    RosLink * ros, const Topics & topics,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node);

  // exposed for the smoke test
  const std::map<char, QPushButton *> & buttons() const {return buttons_;}
  const std::map<std::string, QPushButton *> & classButtons() const {return class_btns_;}
  QPushButton * autoButton() {return btn_auto_;}
  QPushButton * cancelButton() {return btn_cancel_;}
  QPushButton * homeRecordButton() {return btn_home_rec_;}
  QLabel * resLabel() {return lbl_res_;}
  QLabel * fpsLabel() {return lbl_fps_;}
  QPlainTextEdit * robotLog() {return log_;}

public slots:
  void onFrame(QImage frame);
  void onState(raon_gui::State state);
  void onRobotMsg(QString line);
  void checkLink();     // watchdog: public so the test can force a timeout
  void forceStateStale() {last_state_ms_ = 0;}

private:
  QGroupBox * buildVideoBox();
  QGroupBox * buildRobotBox();
  QGroupBox * buildStopBox();
  QGroupBox * buildKeyBox(const QString & title,
    const std::vector<std::pair<char, QString>> & keys);
  QGroupBox * buildSelectBox();
  QPushButton * makeBtn(char key, const QString & text);
  void sendKey(char key);
  void sendMenuSelection(const std::string & cls);   // "" = AUTO
  void applyGating();
  void showRobotLive(const State & s);
  void showRobotDead();

  RosLink * ros_;
  VideoPane * video_;
  std::map<char, QPushButton *> buttons_;
  // menu-selection buttons — gated on the menu, not on a fixed key
  std::map<std::string, QPushButton *> class_btns_;   // class_name → button
  QPushButton * btn_auto_{nullptr};
  QPushButton * btn_cancel_{nullptr};
  QPushButton * btn_home_rec_{nullptr};   // permanently disabled, see README
  QLabel * lbl_res_{nullptr};
  QLabel * lbl_fps_{nullptr};
  QLabel * lbl_robot_{nullptr};
  QPlainTextEdit * log_{nullptr};

  std::optional<State> state_;
  qint64 last_state_ms_{0};
  qint64 last_frame_ms_{0};
  qint64 prev_frame_ms_{0};
  double fps_ema_{0.0};
  bool fps_valid_{false};
};

}  // namespace raon_gui

#endif  // RAON_GUI_MAIN_WINDOW_HPP
