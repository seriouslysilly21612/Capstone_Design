// Embedded RViz — a bare RenderPanel + VisualizationManager inside our Qt
// window, no VisualizationFrame (that class is the whole rviz2 UI with menus
// and toolbars; we only want the 3D view).
//
// This replaces the former QPainter robot renderer (robot_pane.py). The robot
// is now drawn by the stock rviz_default_plugins/RobotModel display from
// /robot_description + TF, i.e. by exactly the code path plain rviz2 uses —
// robot_state_publisher (started by operator.launch.py) turns the bridge's
// /joint_states into that TF.
//
// CAVEAT: embedding rviz_common this way is the community-established pattern,
// not a stability-guaranteed API. It compiles against Humble; a ROS upgrade
// may need this file revisited (and only this file — nothing else touches
// rviz).
#ifndef RAON_GUI_RVIZ_PANE_HPP
#define RAON_GUI_RVIZ_PANE_HPP

#include <QWidget>
#include <memory>

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace rviz_common
{
class RenderPanel;
class VisualizationManager;
}

namespace raon_gui
{

class RvizPane : public QWidget
{
  Q_OBJECT

public:
  // ros_node: a RosNodeAbstraction. Nobody else may spin its raw node —
  // VisualizationManager add_node()s it to its OWN executor and spin_some()s
  // from the 30 Hz update timer.
  RvizPane(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node,
    QWidget * parent = nullptr);
  ~RvizPane() override;

protected:
  // RViz is initialized HERE (deferred one event-loop turn), never in the
  // constructor: Ogre binds to the panel's native X window at initialize
  // time, and Qt re-creates native windows while the widget tree is being
  // shown — init before realization leaves Ogre holding a dead handle, and
  // the first resize then dies inside XFree (field backtrace 2026-08-04).
  void showEvent(QShowEvent * event) override;

private:
  void initNow();

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_;
  rviz_common::RenderPanel * render_panel_{nullptr};
  rviz_common::VisualizationManager * manager_{nullptr};
  bool init_scheduled_{false};
};

}  // namespace raon_gui

#endif  // RAON_GUI_RVIZ_PANE_HPP
