#include "rviz_pane.hpp"

#include <QShowEvent>
#include <QTimer>
#include <QVBoxLayout>

#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"

namespace raon_gui
{

RvizPane::RvizPane(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node,
  QWidget * parent)
: QWidget(parent), ros_node_(ros_node)
{
  setMinimumSize(280, 170);
}

void RvizPane::showEvent(QShowEvent * event)
{
  QWidget::showEvent(event);
  if (!init_scheduled_) {
    init_scheduled_ = true;
    // One event-loop turn later: the show cascade has finished and every
    // native window exists, so Ogre binds to the FINAL X window (see .hpp).
    QTimer::singleShot(0, this, [this]() {initNow();});
  }
}

void RvizPane::initNow()
{
  render_panel_ = new rviz_common::RenderPanel(this);
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(render_panel_);
  render_panel_->show();   // parent is already visible; new children are not

  // Init order is load-bearing (rviz_common's own visualizer_app does the
  // same dance): render window first, then the manager, then panel->manager,
  // then manager init + update timer.
  render_panel_->getRenderWindow()->initialize();
  auto clock = ros_node_.lock()->get_raw_node()->get_clock();
  manager_ = new rviz_common::VisualizationManager(
    render_panel_, ros_node_, nullptr /*window manager*/, clock);
  render_panel_->initialize(manager_);
  manager_->initialize();
  manager_->startUpdate();

  // The robot URDF roots at 'world' (world→link0 fixed identity); the
  // perception pipeline's frames hang under base_link, stitched to world by
  // the static TF in operator.launch.py.
  manager_->setFixedFrame("world");

  auto * grid = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
  grid->subProp("Cell Size")->setValue(0.25);
  grid->subProp("Plane Cell Count")->setValue(12);

  auto * robot =
    manager_->createDisplay("rviz_default_plugins/RobotModel", "RobotModel", true);
  robot->subProp("Description Source")->setValue("Topic");
  robot->subProp("Description Topic")->setValue("/robot_description");
  // robot_state_publisher latches robot_description as transient-local; a
  // volatile subscriber would only see it if we started first — this makes
  // start order not matter.
  robot->subProp("Description Topic")->subProp("Durability Policy")
  ->setValue("Transient Local");

  auto * views = manager_->getViewManager();
  views->setCurrentViewControllerType("rviz_default_plugins/Orbit");
  if (auto * v = views->getCurrent()) {
    v->subProp("Distance")->setValue(2.5);
    v->subProp("Pitch")->setValue(0.5);
    v->subProp("Yaw")->setValue(0.8);
    v->subProp("Focal Point")->subProp("Z")->setValue(0.4);
  }
}

// Destruction order is the constructor's reverse; the manager owns the
// displays and must die before the panel it draws into.
RvizPane::~RvizPane()
{
  delete manager_;
}

}  // namespace raon_gui
