#include "main_window.hpp"

#include <QDateTime>
#include <algorithm>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QStatusBar>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>

#include "ros_link.hpp"
#include "rviz_pane.hpp"
#include "video_pane.hpp"

namespace raon_gui
{

// No frame for this long => the link is presumed dead (~45 missed frames at
// 30 Hz). Same standard for /robot_state (20 Hz): silence disables every
// button — a clickable E-STOP nobody is listening to is the worst lie this
// window could tell.
static constexpr qint64 kLinkTimeoutMs = 1500;

static const char * kOkStyle = "color:#3ddc84; font-weight:bold;";
static const char * kBadStyle = "color:#ff5f56; font-weight:bold;";
static const char * kDimStyle = "color:#808088;";
static const char * kStopStyle =
  "background-color:#8c1d18; color:#ffffff; font-weight:bold;";

// Same characters the terminal takes — see CRobotIndy7::DoInput. 'q'/'z' are
// deliberately absent (state.hpp explains the ISO-cube trap).
struct Group
{
  QString title;
  std::vector<std::pair<char, QString>> keys;
};
static const std::vector<Group> kGroups = {
  {"준비", {{'r', "컨트롤러 토글"}, {'t', "드라이브→CST"},
    {'h', "서보 ON"}, {'j', "서보 OFF"}}},
  {"제어 모드", {{'g', "중력보상"}, {'f', "풀 다이내믹스"},
    {'c', "계산 토크"}, {'i', "자코비안 IK"}, {'k', "스티키 홀드"}}},
  {"피킹", {{'p', "객체 메뉴"}, {'v', "접근 (approach)"},
    {'o', "추종 토글"}, {'b', "홈 복귀"}}},
  {"기타", {{'m', "RBDL IK (6DOF)"}, {'a', "기준 자세"}, {'n', "사각 데모"},
    {'s', "TCP 자세 출력"}, {'w', "캘리브 캡처 arm"},
    {'l', "TCP 궤적 로깅"}, {'d', "거리 오차 로그"}}},
};

static QString tipFor(char k)
{
  switch (k) {
    case 'h': return "필요: 컨트롤러 ON('r') + 중력보상('g')";
    case 'b': return "필요: 컨트롤러+서보 ON, HOME/ready 자세 존재";
    case 'o': return "시작 조건: 컨트롤러+서보 ON, 타깃 LOCK, 유휴 / 추종 중엔 정지 버튼";
    case 'v': return "필요: 타깃 LOCK ('p' 메뉴에서 선택 후)";
    default:
      if (k >= '0' && k <= '9') {return "'p' 메뉴가 열려 있을 때만";}
      return "추종(tracking) 중에는 비활성";
  }
}

MainWindow::MainWindow(
  RosLink * ros, const Topics & topics,
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node)
: ros_(ros)
{
  setWindowTitle("RAON-RT operator console");
  resize(1360, 760);

  video_ = new VideoPane;
  auto * left = new QSplitter(Qt::Vertical);
  left->addWidget(video_);
  if (!rviz_node.expired()) {
    left->addWidget(new RvizPane(rviz_node));
  }
  // Robot terminal mirror (/operator_msg): the 'p' menu, gate verdicts,
  // TARGET LOCKED / GOAL READY — everything the operator used to have to
  // read on the robot's own terminal.
  log_ = new QPlainTextEdit;
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(500);   // scrollback cap, drops oldest
  log_->setStyleSheet(
    "background-color:#101014; color:#c8c8d0; font-family:monospace;");
  log_->setPlaceholderText("로봇 메시지 (/operator_msg) — 'p' 메뉴가 여기 표시됩니다");
  left->addWidget(log_);
  left->setStretchFactor(0, 3);
  left->setStretchFactor(1, 2);
  left->setStretchFactor(left->count() - 1, 1);

  auto * right = new QWidget;
  right->setFixedWidth(360);
  auto * rl = new QVBoxLayout(right);
  rl->addWidget(buildVideoBox());
  rl->addWidget(buildRobotBox());
  rl->addWidget(buildStopBox());
  for (const auto & g : kGroups) {
    rl->addWidget(buildKeyBox(g.title, g.keys));
  }
  rl->addWidget(buildSelectBox());
  rl->addStretch(1);
  {
    // Closes the window; launch shuts the whole station down on our exit
    // (on_exit=Shutdown() in operator.launch.py). Deliberately NOT in
    // buttons_: quitting the CONSOLE must work with the robot link dead.
    auto * quit = new QPushButton("콘솔 종료");
    quit->setStyleSheet(kDimStyle);
    connect(quit, &QPushButton::clicked, this, &QMainWindow::close);
    rl->addWidget(quit);
  }

  auto * central = new QWidget;
  auto * layout = new QHBoxLayout(central);
  layout->addWidget(left, 1);
  layout->addWidget(right, 0);
  setCentralWidget(central);
  statusBar()->showMessage(
    QString("image: %1    state: %2    keys → %3")
    .arg(topics.image.c_str(), topics.state.c_str(), topics.key.c_str()));

  applyGating();   // everything starts disabled

  connect(ros_->sig(), &RosSignals::frameReady,
    this, &MainWindow::onFrame, Qt::QueuedConnection);
  connect(ros_->sig(), &RosSignals::stateReady,
    this, &MainWindow::onState, Qt::QueuedConnection);
  connect(ros_->sig(), &RosSignals::robotMsg,
    this, &MainWindow::onRobotMsg, Qt::QueuedConnection);

  auto * watchdog = new QTimer(this);
  connect(watchdog, &QTimer::timeout, this, &MainWindow::checkLink);
  watchdog->start(500);
}

// ---------------- widgets ----------------

QGroupBox * MainWindow::buildVideoBox()
{
  auto * box = new QGroupBox("Video");
  auto * g = new QGridLayout(box);
  lbl_res_ = new QLabel("–");
  lbl_fps_ = new QLabel("NO SIGNAL");
  lbl_fps_->setStyleSheet(kBadStyle);
  const std::pair<QString, QLabel *> rows[] = {{"해상도", lbl_res_}, {"FPS", lbl_fps_}};
  int row = 0;
  for (const auto & [name, w] : rows) {
    auto * label = new QLabel(name);
    label->setStyleSheet(kDimStyle);
    g->addWidget(label, row, 0);
    g->addWidget(w, row, 1);
    ++row;
  }
  return box;
}

QGroupBox * MainWindow::buildRobotBox()
{
  auto * box = new QGroupBox("Robot");
  auto * v = new QVBoxLayout(box);
  lbl_robot_ = new QLabel;
  lbl_robot_->setWordWrap(true);
  lbl_robot_->setTextFormat(Qt::RichText);
  v->addWidget(lbl_robot_);
  showRobotDead();
  return box;
}

QPushButton * MainWindow::makeBtn(char key, const QString & text)
{
  auto * btn = new QPushButton(text);
  connect(btn, &QPushButton::clicked, this, [this, key]() {sendKey(key);});
  btn->setToolTip(tipFor(key));
  buttons_[key] = btn;
  return btn;
}

QGroupBox * MainWindow::buildStopBox()
{
  // Own row, own colour, always at the top: these two are the reason an
  // operator looks at this window in a hurry.
  auto * box = new QGroupBox("정지");
  auto * h = new QHBoxLayout(box);
  for (const auto & [key, text] :
    std::vector<std::pair<char, QString>>{
      {'e', "비상 정지 (서보 차단)"}, {'x', "동작 정지 (유지)"}})
  {
    auto * btn = makeBtn(key, text);
    btn->setStyleSheet(kStopStyle);
    h->addWidget(btn);
  }
  buttons_.at('e')->setToolTip(
    "모든 동작 취소 + 서보 OFF(브레이크). 재개: 'h'로 서보 재무장");
  buttons_.at('x')->setToolTip(
    "모든 동작 취소, 서보 유지 — 중력보상으로 그 자리에서 정지");
  return box;
}

QGroupBox * MainWindow::buildKeyBox(
  const QString & title, const std::vector<std::pair<char, QString>> & keys)
{
  auto * box = new QGroupBox(title);
  auto * g = new QGridLayout(box);
  int i = 0;
  for (const auto & [key, text] : keys) {
    g->addWidget(makeBtn(key, QString("%1  [%2]").arg(text).arg(key)), i / 2, i % 2);
    ++i;
  }
  return box;
}

// The 5 pickable classes (person is never selectable — the bridge excludes it
// from the menu too). Buttons light up only for classes in the OPEN menu.
static const std::vector<std::pair<std::string, QString>> kClasses = {
  {"apple", "사과"}, {"orange", "오렌지"}, {"banana", "바나나"},
  {"tennis_ball", "테니스공"}, {"mustard_bottle", "머스타드병"},
};

QGroupBox * MainWindow::buildSelectBox()
{
  // Replaces the raw digit row: 'p' opens the menu, then the operator clicks
  // the OBJECT, and the window translates it to the menu digit. The digit for
  // a class depends on the menu's numbering, so the mapping comes from
  // menu_items in /robot_state — never guessed locally (a wrong digit sends
  // the arm to the wrong object).
  auto * box = new QGroupBox("객체 선택 ('p' 메뉴 열림 시, 인식된 것만 활성)");
  auto * g = new QGridLayout(box);
  int i = 0;
  for (const auto & [cls, label] : kClasses) {
    auto * btn = new QPushButton(label);
    connect(btn, &QPushButton::clicked, this,
      [this, cls = cls]() {sendMenuSelection(cls);});
    btn->setToolTip("메뉴가 열려 있고 이 객체가 인식 목록에 있을 때만");
    btn->setEnabled(false);
    class_btns_[cls] = btn;
    g->addWidget(btn, i / 3, i % 3);
    ++i;
  }
  btn_auto_ = new QPushButton("AUTO");
  btn_auto_->setToolTip("desired_class 해제 — 최적 객체 자동 선택");
  connect(btn_auto_, &QPushButton::clicked, this,
    [this]() {sendMenuSelection("");});
  btn_auto_->setEnabled(false);
  g->addWidget(btn_auto_, i / 3, i % 3);
  ++i;

  btn_cancel_ = new QPushButton("취소");
  btn_cancel_->setToolTip("메뉴 닫기 (digit 0)");
  connect(btn_cancel_, &QPushButton::clicked, this, [this]() {sendKey('0');});
  btn_cancel_->setEnabled(false);
  g->addWidget(btn_cancel_, i / 3, i % 3);
  ++i;

  // Deliberately DEAD in the UI: the computed q_ready is the trusted home,
  // and re-recording HOME from a window (arm possibly in a bad pose) is an
  // easy way to lose it. The whole chain below stays functional — README
  // §HOME 기록 has the one-line re-enable.
  btn_home_rec_ = new QPushButton("HOME 기록");
  btn_home_rec_->setEnabled(false);
  btn_home_rec_->setToolTip("비활성 (q_ready 사용 중) — 해제 방법은 README");
  connect(btn_home_rec_, &QPushButton::clicked, this, [this]() {
      if (state_.has_value()) {
        sendKey(static_cast<char>('0' + state_->menu_items.size() + 2));
      }
    });
  g->addWidget(btn_home_rec_, i / 3, i % 3);
  return box;
}

void MainWindow::sendMenuSelection(const std::string & cls)
{
  // Position in menu_items + 1 == the digit ShowMenu printed for this class;
  // AUTO ("") is the entry right after the classes. Guarded: a click racing a
  // menu change just does nothing.
  if (!state_.has_value() || !state_->menu) {return;}
  const auto & items = state_->menu_items;
  if (cls.empty()) {
    sendKey(static_cast<char>('0' + items.size() + 1));
    return;
  }
  for (size_t i = 0; i < items.size(); ++i) {
    if (items[i] == cls) {
      sendKey(static_cast<char>('1' + i));
      return;
    }
  }
}

// ---------------- actions / gating ----------------

void MainWindow::sendKey(char key)
{
  ros_->sendKey(key);
  statusBar()->showMessage(QString("sent '%1' → /operator_key").arg(key), 2000);
}

void MainWindow::applyGating()
{
  const bool fresh = state_.has_value() &&
    (QDateTime::currentMSecsSinceEpoch() - last_state_ms_) < kLinkTimeoutMs;
  for (auto & [key, btn] : buttons_) {
    btn->setEnabled(fresh && keyEnabled(key, *state_));
  }
  // menu-selection buttons: class buttons only for classes in the OPEN menu;
  // AUTO/취소 whenever a menu is open. HOME 기록 stays dead (see README).
  const bool menu_open = fresh && state_->menu;
  for (auto & [cls, btn] : class_btns_) {
    btn->setEnabled(menu_open &&
      std::find(state_->menu_items.begin(), state_->menu_items.end(), cls) !=
      state_->menu_items.end());
  }
  btn_auto_->setEnabled(menu_open);
  btn_cancel_->setEnabled(menu_open);
}

void MainWindow::onState(State s)
{
  state_ = s;
  last_state_ms_ = QDateTime::currentMSecsSinceEpoch();
  showRobotLive(s);
  applyGating();
}

void MainWindow::showRobotLive(const State & s)
{
  auto dot = [](bool ok) {
      return ok ? "<span style='color:#3ddc84'>●</span>" :
             "<span style='color:#ff5f56'>●</span>";
    };
  QString line1 = QString("%1 servo&nbsp;&nbsp;%2 ctrl&nbsp;&nbsp;모드: %3")
    .arg(dot(s.servo), dot(s.ctrl), s.grav ? "중력보상" : "기타");
  QStringList badges;
  if (s.track) {badges << "<span style='color:#ffb454'>TRACKING</span>";}
  if (s.busy) {badges << "<span style='color:#ffb454'>BUSY</span>";}
  if (s.lock) {badges << "<span style='color:#3ddc84'>LOCK</span>";}
  if (s.menu) {badges << "MENU";}
  if (s.calib) {badges << "<span style='color:#ff5f56'>CALIB-ARM</span>";}
  lbl_robot_->setText(
    line1 + "<br>" +
    (badges.isEmpty() ? "<span style='color:#808088'>대기</span>" : badges.join(" · ")));
}

void MainWindow::showRobotDead()
{
  lbl_robot_->setText(
    "<span style='color:#ff5f56; font-weight:bold'>NO LINK</span>"
    "<span style='color:#808088'> — 로봇 앱(/robot_state) 응답 없음, "
    "전 버튼 비활성</span>");
}

// ---------------- slots ----------------

void MainWindow::onRobotMsg(QString line)
{
  log_->appendPlainText(
    QTime::currentTime().toString("HH:mm:ss ") + line.trimmed());
}

void MainWindow::onFrame(QImage frame)
{
  video_->setFrame(frame);
  lbl_res_->setText(QString("%1 × %2").arg(frame.width()).arg(frame.height()));
  // FPS is measured here, on the frames actually shown, so it reports what
  // the operator sees rather than what the network delivered.
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  if (prev_frame_ms_ > 0 && now > prev_frame_ms_) {
    const double inst = 1000.0 / static_cast<double>(now - prev_frame_ms_);
    fps_ema_ = fps_valid_ ? 0.2 * inst + 0.8 * fps_ema_ : inst;
    fps_valid_ = true;
    lbl_fps_->setText(QString::number(fps_ema_, 'f', 1));
    lbl_fps_->setStyleSheet(kOkStyle);
  }
  prev_frame_ms_ = now;
  last_frame_ms_ = now;
}

void MainWindow::checkLink()
{
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  // The FPS label doubles as the video link light; a dead stream keeping its
  // last FPS is exactly the lie this window exists to prevent.
  if (now - last_frame_ms_ >= kLinkTimeoutMs) {
    lbl_fps_->setText("NO SIGNAL");
    lbl_fps_->setStyleSheet(kBadStyle);
    fps_valid_ = false;
    prev_frame_ms_ = 0;
  }
  if (now - last_state_ms_ >= kLinkTimeoutMs) {
    if (state_.has_value()) {
      state_.reset();
      showRobotDead();
    }
    applyGating();
  }
}

}  // namespace raon_gui
