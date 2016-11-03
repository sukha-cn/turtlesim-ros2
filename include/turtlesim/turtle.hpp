#ifndef TURTLESIM_TURTLE_H
#define TURTLESIM_TURTLE_H

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <rclcpp/rclcpp.hpp>
# include <boost/shared_ptr.hpp>

# include <turtlesim/msg/pose.hpp>
# include <geometry_msgs/msg/twist.hpp>
# include <turtlesim/srv/set_pen.hpp>
# include <turtlesim/srv/teleport_relative.hpp>
# include <turtlesim/srv/teleport_absolute.hpp>
# include <turtlesim/msg/color.hpp>
#endif

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <chrono>

#define PI 3.14159265

namespace turtlesim
{


class Turtle
{
public:
  Turtle(const std::shared_ptr<rclcpp::Node> nh, const std::string name, const QImage& turtle_image, const QPointF& pos, float orient);

  bool update(double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height);
  void paint(QPainter &painter);
private:
//   void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr vel);
//   bool setPenCallback(turtlesim::srv::SetPen::Request&, turtlesim::srv::SetPen::Response&);
//   bool teleportRelativeCallback(turtlesim::srv::TeleportRelative::Request&, turtlesim::srv::TeleportRelative::Response&);
//   bool teleportAbsoluteCallback(turtlesim::srv::TeleportAbsolute::Request&, turtlesim::srv::TeleportAbsolute::Response&);

  void rotateImage();

  std::shared_ptr<rclcpp::Node> nh_;
  std::string name_;

  QImage turtle_image_;
  QImage turtle_rotated_image_;

  QPointF pos_;
  qreal orient_;

  qreal lin_vel_;
  qreal ang_vel_;

  bool pen_on_;
  QPen pen_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<turtlesim::msg::Color>::SharedPtr color_pub_;

  // ros::WallTime last_command_time_;
  std::chrono::system_clock::time_point last_command_time_;

  float meter_;

  struct TeleportRequest
  {
    TeleportRequest(float x, float y, qreal _theta, qreal _linear, bool _relative)
    : pos(x, y)
    , theta(_theta)
    , linear(_linear)
    , relative(_relative)
    {}

    QPointF pos;
    qreal theta;
    qreal linear;
    bool relative;
  };
  typedef std::vector<TeleportRequest> V_TeleportRequest;
  V_TeleportRequest teleport_requests_;
};
typedef boost::shared_ptr<Turtle> TurtlePtr;

}


#endif