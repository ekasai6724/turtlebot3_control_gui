#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow *w = new MainWindow();
    w->show();

#ifdef ROS
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create publisher node
    auto node = rclcpp::Node::make_shared("qt_publisher");

    // Publisher
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
    w->SetPublisherPtr(pub);

    // ROS and application loop
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok() && w->ui_alive())
    {
        a.processEvents();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
#else
    return a.exec();
#endif
}
