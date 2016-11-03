#include "turtlesim/turtle_frame.hpp"

#include <QPointF>

#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace turtlesim
{

TurtleFrame::TurtleFrame(std::shared_ptr<rclcpp::Node> nh, QWidget* parent, Qt::WindowFlags f)
: nh_(nh)
, QFrame(parent, f)
, path_image_(500, 500, QImage::Format_ARGB32)
, path_painter_(&path_image_)
, frame_count_(0)
{
  setFixedSize(500, 500);
  setWindowTitle("TurtleSim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

//   nh_->setParam("background_r", DEFAULT_BG_R);
//   nh_->setParam("background_g", DEFAULT_BG_G);
//   nh_->setParam("background_b", DEFAULT_BG_B);

  QVector<QString> turtles;
  turtles.append("box-turtle.png");
  turtles.append("robot-turtle.png");
  turtles.append("sea-turtle.png");
  turtles.append("diamondback.png");
  turtles.append("electric.png");
  turtles.append("fuerte.png");
  turtles.append("groovy.png");
  turtles.append("hydro.svg");
  turtles.append("indigo.svg");
  turtles.append("jade.png");

  QString images_path = "/home/micros/test_ws/src/turtlesim/images/";
  for (int i = 0; i < turtles.size(); ++i)
  {
    QImage img;
    img.load(images_path + turtles[i]);
    turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height();

  clear();

//   nh_->create_service<turtlesim::srv::Empty>("clear", TurtleFrame::clearCallback);
//   nh_->create_service<turtlesim::srv::Empty>("reset", TurtleFrame::resetCallback);
//   nh_->create_service<turtlesim::srv::Spawn>("spawn", TurtleFrame::spawnCallback);
//   nh_->create_service<turtlesim::srv::Kill>("kill", TurtleFrame::killCallback);

//  ROS_INFO("Starting turtlesim with node name %s", ros::this_node::getName().c_str()) ;

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);

  // spawn all available turtle types
  if(FALSE)
  {
    for(int index = 0; index < turtles.size(); ++index)
    {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
    }
  }
}


TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

// bool TurtleFrame::spawnCallback(std::shared_ptr<turtlesim::srv::Spawn::Request> req, 
//                                 std::shared_ptr<turtlesim::srv::Spawn::Response> res)
// {
//   std::string name = spawnTurtle(req->name, req->x, req->y, req->theta);
//   if (name.empty())
//   {
//     // ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
//     return false;
//   }

//   res->name = name;

//   return true;
// }

// bool TurtleFrame::killCallback(turtlesim::srv::Kill::Request& req, 
//                                turtlesim::srv::Kill::Response& res)
// {
//   M_Turtle::iterator it = turtles_.find(req.name);
//   if (it == turtles_.end())
//   {
//     // ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
//     return false;
//   }

//   turtles_.erase(it);
//   update();

//   return true;
// }

// bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
// {
//   ROS_INFO("Clearing turtlesim.");
//   clear();
//   return true;
// }

// bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
// {
//   ROS_INFO("Resetting turtlesim.");
//   turtles_.clear();
//   id_counter_ = 0;
//   spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
//   clear();
//   return true;
// }


bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

   TurtlePtr t(new Turtle(nh_, real_name, turtle_images_[index], QPointF(x, height_in_meters_ - y), angle));
   turtles_[real_name] = t;
   update();

//    ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

//   nh_.param("background_r", r, r);
//   nh_.param("background_g", g, g);
//   nh_.param("background_b", b, b);

  path_image_.fill(qRgb(r, g, b));
  update();
}

void TurtleFrame::onUpdate()
{
  rclcpp::spin_some(nh_);

  updateTurtles();

  if (!rclcpp::ok())
  {
    close();
  }
}

void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void TurtleFrame::updateTurtles()
{
//   if (last_turtle_update_.isZero())
//   {
//     last_turtle_update_ = ros::WallTime::now();
//     return;
//   }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}


}