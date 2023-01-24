#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>
#include <QtWidgets>

#ifdef ROS
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
#ifdef ROS
    void    SetPublisherPtr(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub);
#endif

    bool    ui_alive() const;

protected:
    void    closeEvent(QCloseEvent *event);

private:
    Ui::MainWindow      *ui;
    class ControlPad    *m_cpad;
    class QTimer        *m_timer;
    int32_t             m_elaps;
    bool                m_ui_alive;

#ifdef ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
#endif

public slots:
    void onTimer(void);
};
#endif // MAINWINDOW_H
