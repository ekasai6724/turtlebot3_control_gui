#include "mainwindow.h"
#include "controlpad.h"
#include "ui_mainwindow.h"

/******************************************************************************
    コンストラクタ
******************************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // マウス操作パッドオブジェクト生成と初期化
    m_cpad = new ControlPad(this);
    m_cpad->setGeometry(5, 5, 300, 300);

    m_ui_alive = true;
    m_elaps = 0;
    m_timer = new QTimer();
    connect(m_timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    m_timer->start(100);
}

/******************************************************************************
    デストラクタ
******************************************************************************/
MainWindow::~MainWindow()
{
    delete ui;
}

/******************************************************************************
    ウィンドウが閉じられる際の処理
******************************************************************************/
void MainWindow::closeEvent(QCloseEvent *event)
{
    m_ui_alive = false;
    event->accept();
}

/******************************************************************************
    GUIが表示されているか(ウィンドウが閉じられたか)
******************************************************************************/
bool MainWindow::ui_alive() const
{
    return m_ui_alive;
}

#ifdef ROS
/******************************************************************************
    パブリッシャのポインタを登録
******************************************************************************/
void MainWindow::SetPublisherPtr(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
{
    m_publisher = pub;
}
#endif

/******************************************************************************
    周期タイマ処理
******************************************************************************/
void MainWindow::onTimer(void)
{
    m_elaps++;
    ui->lblTime->setText(QString("Time:%1").arg(m_elaps));

    m_cpad->CyclicEvent();      // 操作パッドの入力と描画

    // Linear/Angular速度指令値表示
    ui->lblLinear->setText(QString::number(m_cpad->m_cmd_linear, 'f', 2));
    ui->lblAngular->setText(QString::number(m_cpad->m_cmd_angular, 'f', 2));

#ifdef ROS
    auto msg = geometry_msgs::msg::Twist();
    //msg.linear.x = g_elaps;
    //msg.angular.z = -g_elaps;
    msg.linear.x    = m_cpad->m_cmd_linear;
    msg.angular.z   = m_cpad->m_cmd_angular;
    m_publisher->publish(msg);
#endif
    update();
}

