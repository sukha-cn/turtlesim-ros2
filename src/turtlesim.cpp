#include <QApplication>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/turtle_frame.hpp"

class TurtleApp : public QApplication
{
public:
  std::shared_ptr<rclcpp::Node> nh_;

  TurtleApp(int& argc, char** argv)
    : QApplication(argc, argv)
  {
    rclcpp::init(argc, argv);
    nh_ = rclcpp::Node::make_shared("turtlesim");
  }

  int exec()
  {
    turtlesim::TurtleFrame *frame = new turtlesim::TurtleFrame(nh_);
    frame->show();

    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  TurtleApp app(argc, argv);
  return app.exec();
}
