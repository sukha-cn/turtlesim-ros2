#ifndef TURTLE_FRAME_H
#define TURTLE_FRAME_H

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <rclcpp/rclcpp.hpp>

# include <turtlesim/srv/empty.hpp>
# include <turtlesim/srv/spawn.hpp>
# include <turtlesim/srv/kill.hpp>
# include <map>

# include "turtle.hpp"
#endif

namespace turtlesim
{

class TurtleFrame : public QFrame
{
  Q_OBJECT
public:
  TurtleFrame(std::shared_ptr<rclcpp::Node> nh, QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~TurtleFrame();

  std::string spawnTurtle(const std::string& name, float x, float y, float angle);
  std::string spawnTurtle(const std::string& name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent* event);

private slots:
  void onUpdate();

private:
  void updateTurtles();
  void clear();
  bool hasTurtle(const std::string& name);

//   bool clearCallback(std::shared_ptr<turtlesim::srv::Empty::Request>,
//                             std::shared_ptr<turtlesim::srv::Empty::Response>);
//   bool resetCallback(std::shared_ptr<turtlesim::srv::Empty::Request>, 
//                             std::shared_ptr<turtlesim::srv::Empty::Response>);
//   bool spawnCallback(std::shared_ptr<turtlesim::srv::Spawn::Request>, 
//                             std::shared_ptr<turtlesim::srv::Spawn::Response>);
//   bool killCallback(std::shared_ptr<turtlesim::srv::Kill::Request>, 
//                            std::shared_ptr<turtlesim::srv::Kill::Response>);

  std::shared_ptr<rclcpp::Node> nh_;
  QTimer* update_timer_;
  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;

  // ros::WallTime last_turtle_update_;

  // ros::ServiceServer clear_srv_;
  // ros::ServiceServer reset_srv_;
  // ros::ServiceServer spawn_srv_;
  // ros::ServiceServer kill_srv_;

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_;
  uint32_t id_counter_ = 0;

  QVector<QImage> turtle_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}


#endif