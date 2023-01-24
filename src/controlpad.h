#ifndef CONTROLPAD_H
#define CONTROLPAD_H

#include <QtGui>
#include <QtWidgets>

#define MAXVEL_LINEAR   (0.2)
#define MAXVEL_ANGULAR  (2.0)
#define ACCEL_LINEAR    (MAXVEL_LINEAR/10)
#define ACCEL_ANGULAR   (MAXVEL_ANGULAR/10)

namespace Ui {
class ControlPad;
}

class ControlPad : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPad(QWidget *parent = nullptr);
    ~ControlPad();

    void    CyclicEvent(void);

    double  m_tgt_linear, m_tgt_angular,
            m_cmd_linear, m_cmd_angular;

protected:
    void    paintEvent(QPaintEvent *event);
    void    mousePressEvent(QMouseEvent *event);
    void    mouseReleaseEvent(QMouseEvent *event);
    void    mouseMoveEvent(QMouseEvent *event);

private:
    Ui::ControlPad *ui;

    void    MouseEvent(QMouseEvent *event);
};

#endif // CONTROLPAD_H
