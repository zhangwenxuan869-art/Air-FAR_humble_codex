#ifndef TELEOP_RVIZ_PLUGIN__DRIVE_WIDGET_H_
#define TELEOP_RVIZ_PLUGIN__DRIVE_WIDGET_H_

#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>

namespace teleop_rviz_plugin
{

class DriveWidget : public QWidget
{
  Q_OBJECT

public:
  explicit DriveWidget(QWidget * parent = nullptr);

  QSize sizeHint() const override {return QSize(150, 150);}

Q_SIGNALS:
  void outputVelocity(float linear, float angular, bool pressed, float z_vel);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void leaveEvent(QEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;

private Q_SLOTS:
  void setZVelocity(int value);

private:
  void sendVelocitiesFromMouse(int x, int y, int width, int height);
  void stop();

  float linear_velocity_;
  float angular_velocity_;
  float linear_scale_;
  float angular_scale_;
  float x_mouse_;
  float y_mouse_;
  float z_velocity_;
  bool mouse_pressed_;
  QSlider * z_velocity_slider_;
};

}  // namespace teleop_rviz_plugin

#endif  // TELEOP_RVIZ_PLUGIN__DRIVE_WIDGET_H_
