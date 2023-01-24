#include "controlpad.h"
#include "ui_controlpad.h"

/******************************************************************************
    コンストラクタ
******************************************************************************/
ControlPad::ControlPad(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControlPad)
{
    ui->setupUi(this);

    m_tgt_linear = 0; m_tgt_angular = 0;
    m_cmd_linear = 0; m_cmd_angular = 0;
}

/******************************************************************************
    デストラクタ
******************************************************************************/
ControlPad::~ControlPad()
{
    delete ui;
}

/******************************************************************************
    周期処理
******************************************************************************/
void ControlPad::CyclicEvent()
{
    // 速度指令値を目標値に近づけていく
    if(m_tgt_linear > m_cmd_linear)
    {
        m_cmd_linear += ACCEL_LINEAR;
        if(m_cmd_linear > m_tgt_linear)     m_cmd_linear = m_tgt_linear;

    }
    else if (m_tgt_linear < m_cmd_linear)
    {
        m_cmd_linear -= ACCEL_LINEAR;
        if(m_cmd_linear < m_tgt_linear)     m_cmd_linear = m_tgt_linear;
    }

    if(m_tgt_angular > m_cmd_angular)
    {
        m_cmd_angular += ACCEL_ANGULAR;
        if(m_cmd_angular > m_tgt_angular)   m_cmd_angular = m_tgt_angular;

    }
    else if (m_tgt_angular < m_cmd_angular)
    {
        m_cmd_angular -= ACCEL_ANGULAR;
        if(m_cmd_angular < m_tgt_angular)   m_cmd_angular = m_tgt_angular;
    }

    update();
}

/******************************************************************************
    画面描画イベント
******************************************************************************/
void ControlPad::paintEvent(QPaintEvent *event)
{
    QPainter    painter(this);
    QPoint      center;

    // 背景
    painter.setPen(Qt::white);
    painter.setBrush(Qt::white);
    painter.drawRect(0, 0, this->width(), this->height());

    // 座標基準線
    painter.setPen(Qt::black);
    painter.setBrush(Qt::black);
    QPoint p1 = QPoint(0,               this->height()/2);
    QPoint p2 = QPoint(this->width(),   this->height()/2);
    painter.drawLine(p1, p2);
    p1 = QPoint(this->width()/2, 0);
    p2 = QPoint(this->width()/2, this->height());
    painter.drawLine(p1, p2);

    // 速度指令を図示
    painter.setPen(Qt::black);
    painter.setBrush(Qt::red);
    center.setX((int)-(m_cmd_angular * this->width()/2  / MAXVEL_ANGULAR - this->width()/2 ));
    center.setY((int)-(m_cmd_linear  * this->height()/2 / MAXVEL_LINEAR  - this->height()/2));
    painter.drawEllipse(center, 5, 5);
}

/******************************************************************************
    マウスボタンダウンイベント
******************************************************************************/
void ControlPad::mousePressEvent(QMouseEvent *event)
{
    MouseEvent(event);
}

/******************************************************************************
    マウスボタンアップイベント
******************************************************************************/
void ControlPad::mouseReleaseEvent(QMouseEvent *event)
{
    MouseEvent(event);
}

/******************************************************************************
    マウスポインタ移動イベント
******************************************************************************/
void ControlPad::mouseMoveEvent(QMouseEvent *event)
{
    MouseEvent(event);
}

/*==============================================================================
    マウスカーソル移動とマウスボタン状態から速度指令目標値算出
==============================================================================*/
void ControlPad::MouseEvent(QMouseEvent *event)
{
    int     abf, lbf;
    double  target_l, target_a;

    // マウスカーソル位置から座標を算出
    // 左ボタン押下していない場合は0に固定
    if(event->buttons() & Qt::LeftButton)
    {
        // Widget座標系は左上原点、x→/y↓
        // Linear/Angular座標では中心原点、L↑/A←
        // (angularは左旋回で正の角度なのでパッド左側が正の値とする)
        lbf = -(event->y() - this->height()/2);
        abf = -(event->x() - this->width()/2);
    }
    else
    {
        lbf = 0;
        abf = 0;
    }

    // Linear/Augularの速度に変換、上下限の制限
    target_l = (double)lbf * MAXVEL_LINEAR  / (this->height()/2);
    if      (target_l >  MAXVEL_LINEAR)     target_l =  MAXVEL_LINEAR;
    else if (target_l < -MAXVEL_LINEAR)     target_l = -MAXVEL_LINEAR;
    target_a = (double)abf * MAXVEL_ANGULAR / (this->width() /2);
    if      (target_a >  MAXVEL_ANGULAR)    target_a =  MAXVEL_ANGULAR;
    else if (target_a < -MAXVEL_ANGULAR)    target_a = -MAXVEL_ANGULAR;

    m_tgt_linear = target_l;
    m_tgt_angular = target_a;
}
