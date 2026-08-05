#include "ros_link.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <map>
#include <string>
#include <vector>

namespace raon_gui
{

// Keyed by class_name, never class_id: the id ordering comes from
// decode_meta.json and would silently mis-label if it ever changed.
// (Duplicated with detection_viewer_pkg on purpose — see README §8.)
static const std::map<std::string, cv::Scalar> kClassColors = {
  {"apple", {60, 60, 220}}, {"orange", {0, 150, 255}},
  {"banana", {60, 220, 220}}, {"tennis_ball", {60, 220, 60}},
  {"mustard_bottle", {200, 140, 60}}, {"person", {220, 120, 220}},
};
static const cv::Scalar kUnknownColor{200, 200, 200};

static int64_t stampNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

RosLink::RosLink(
  const std::string & image_topic, const std::string & detections_topic,
  const std::string & key_topic, const std::string & state_topic,
  const std::string & msg_topic)
: rclcpp::Node("raon_operator_gui")
{
  // BEST_EFFORT on the image: the publisher is RELIABLE (compatible), and
  // retransmitting a frame we are about to overwrite only adds latency.
  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  // RELIABLE for the small topics — and for the outgoing keys: a command that
  // moves an arm must not be silently dropped by the transport.
  auto small_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  sub_image_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    image_topic, image_qos,
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {onImage(m);});
  sub_dets_ = create_subscription<my_interfaces::msg::DetectionArray>(
    detections_topic, small_qos,
    [this](my_interfaces::msg::DetectionArray::ConstSharedPtr m) {onDetections(m);});
  sub_state_ = create_subscription<std_msgs::msg::String>(
    state_topic, small_qos,
    [this](std_msgs::msg::String::ConstSharedPtr m) {onState(m);});
  // depth 20 to match the publisher: menu/gate lines are events, keep them all
  sub_msg_ = create_subscription<std_msgs::msg::String>(
    msg_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) {
      emit signals_.robotMsg(QString::fromStdString(m->data));
    });
  pub_key_ = create_publisher<std_msgs::msg::String>(key_topic, small_qos);
}

void RosLink::sendKey(char key)
{
  std_msgs::msg::String msg;
  msg.data = std::string(1, key);
  pub_key_->publish(msg);
  RCLCPP_INFO(get_logger(), "key '%c' -> %s", key, pub_key_->get_topic_name());
}

void RosLink::onDetections(my_interfaces::msg::DetectionArray::ConstSharedPtr msg)
{
  last_dets_ = msg;
}

void RosLink::onState(std_msgs::msg::String::ConstSharedPtr msg)
{
  State s;
  if (parseState(msg->data, s)) {
    emit signals_.stateReady(s);
  } else {
    // a parse failure is version skew worth shouting about, not crashing over
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "bad /robot_state JSON (app/console version skew?)");
  }
}

void RosLink::onImage(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  if (frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "imdecode failed");
    return;
  }
  const int64_t key = stampNs(msg->header.stamp);
  // Monotonic guard: a reordered/duplicate image must not redraw an older
  // frame over a newer one, which reads as a stutter.
  if (have_rendered_ && key <= last_rendered_ns_) {return;}
  have_rendered_ = true;
  last_rendered_ns_ = key;

  // Boxes older than kMaxBoxAgeS relative to THIS frame are not drawn: a
  // stalled detector must go visibly blank rather than leave its last boxes
  // frozen on live video.
  const my_interfaces::msg::DetectionArray * dets = nullptr;
  if (last_dets_ &&
    (key - stampNs(last_dets_->header.stamp)) <= static_cast<int64_t>(kMaxBoxAgeS * 1e9))
  {
    dets = last_dets_.get();
  }

  if (dets) {
    for (const auto & det : dets->detections) {
      // Coords are already pixels in THIS frame — the worker undid the
      // 416x416 letterbox. Never rescale here.
      const int xmin = static_cast<int>(std::lround(det.center_x - det.width / 2.0));
      const int ymin = static_cast<int>(std::lround(det.center_y - det.height / 2.0));
      const int xmax = static_cast<int>(std::lround(det.center_x + det.width / 2.0));
      const int ymax = static_cast<int>(std::lround(det.center_y + det.height / 2.0));
      const auto it = kClassColors.find(det.class_name);
      const cv::Scalar color = (it != kClassColors.end()) ? it->second : kUnknownColor;
      cv::rectangle(frame, {xmin, ymin}, {xmax, ymax}, color, 2);
      char label[64];
      snprintf(label, sizeof(label), "%s %.2f", det.class_name.c_str(), det.confidence);
      int baseline = 0;
      const cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      const int ytop = std::max(0, ymin - ts.height - 4);
      cv::rectangle(frame, {xmin, ytop}, {xmin + ts.width + 4, ytop + ts.height + 4}, color, -1);
      cv::putText(frame, label, {xmin + 2, ytop + ts.height},
        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 0}, 1, cv::LINE_AA);
    }
  }

  // Format_BGR888 means NO channel swap — the JPEG payload really is BGR
  // (verified in the field; a cvtColor here turns the orange blue).
  // .copy() detaches the QImage from the cv::Mat buffer before it dies.
  emit signals_.frameReady(
    QImage(frame.data, frame.cols, frame.rows,
    static_cast<int>(frame.step), QImage::Format_BGR888).copy());
}

}  // namespace raon_gui
