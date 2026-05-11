#include "drive_widget.h"

#include <algorithm>
#include <cmath>

namespace teleop_rviz_plugin
{

DriveWidget::DriveWidget(QWidget * parent)
: QWidget(parent),
  linear_velocity_(0.0F),
  angular_velocity_(0.0F),
  linear_scale_(1.0F),
  angular_scale_(1.0F),
  x_mouse_(0.0F),
  y_mouse_(0.0F),
  z_velocity_(0.0F),
  mouse_pressed_(false)
{
  z_velocity_slider_ = new QSlider(Qt::Vertical, this);
  z_velocity_slider_->setRange(-100, 100);
  z_velocity_slider_->setValue(0);
  z_velocity_slider_->setTickPosition(QSlider::TicksLeft);
  z_velocity_slider_->setTickInterval(20);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(z_velocity_slider_);
  setLayout(layout);

  connect(z_velocity_slider_, &QSlider::valueChanged, this, &DriveWidget::setZVelocity);
}

void DriveWidget::setZVelocity(int value)
{
  z_velocity_ = static_cast<float>(value) / 100.0F;
  update();
}

void DriveWidget::paintEvent(QPaintEvent * event)
{
  (void)event;

  QColor background = isEnabled() ? Qt::white : Qt::lightGray;
  QColor crosshair = isEnabled() ? Qt::black : Qt::darkGray;

  const int w = width();
  const int h = height();
  const int size = ((w > h) ? h : w) - 1 - 60;
  const int hpad = (w - size) / 2 + 30;
  const int vpad = (h - size) / 2;

  QPainter painter(this);
  painter.setBrush(background);
  painter.setPen(crosshair);

  painter.drawRect(QRect(hpad, vpad, size, size));
  painter.drawLine(hpad + size / 2, vpad, hpad + size / 2, vpad + size);
  painter.drawLine(hpad, vpad + static_cast<int>((size / 2.0) * 1.588),
    hpad + size, vpad + static_cast<int>((size / 2.0) * (1.0 - 0.588)));
  painter.drawLine(hpad, vpad + static_cast<int>((size / 2.0) * (1.0 - 0.588)),
    hpad + size, vpad + static_cast<int>((size / 2.0) * 1.588));

  if (isEnabled() && (angular_velocity_ != 0.0F || linear_velocity_ != 0.0F)) {
    QPen pen;
    pen.setWidth(size / 150);
    pen.setColor(Qt::darkRed);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    painter.setPen(pen);

    QPointF joystick[2];
    joystick[0].setX(hpad + size / 2);
    joystick[0].setY(vpad + size / 2);

    float x = x_mouse_ > hpad + size / 2 ?
      std::min<float>(hpad + size, x_mouse_) :
      std::max<float>(hpad, x_mouse_);
    float y = y_mouse_ > vpad + size / 2 ?
      std::min<float>(vpad + size, y_mouse_) :
      std::max<float>(vpad, y_mouse_);
    joystick[1].setX(x);
    joystick[1].setY(y);

    painter.drawPolyline(joystick, 2);
    painter.drawEllipse(static_cast<int>(x) - 10, static_cast<int>(y) - 10, 20, 20);
  }
}

void DriveWidget::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  z_velocity_slider_->setMinimumWidth(50);
  const int new_height = ((width() > height()) ? height() : width()) - 1 - 60;
  z_velocity_slider_->setMaximumHeight(new_height);
}

void DriveWidget::mouseMoveEvent(QMouseEvent * event)
{
  sendVelocitiesFromMouse(event->x(), event->y(), width(), height());
}

void DriveWidget::mousePressEvent(QMouseEvent * event)
{
  sendVelocitiesFromMouse(event->x(), event->y(), width(), height());
}

void DriveWidget::mouseReleaseEvent(QMouseEvent * event)
{
  (void)event;
  stop();
}

void DriveWidget::leaveEvent(QEvent * event)
{
  (void)event;
  stop();
}

void DriveWidget::sendVelocitiesFromMouse(int x, int y, int width, int height)
{
  const int size = (width > height) ? height : width;
  const int hpad = (width - size) / 2;
  const int vpad = (height - size) / 2;

  x_mouse_ = static_cast<float>(std::clamp(x, width / 2 - size / 2, width / 2 + size / 2));
  y_mouse_ = static_cast<float>(std::clamp(y, height / 2 - size / 2, height / 2 + size / 2));

  linear_velocity_ = (1.0F - static_cast<float>(y - vpad) / static_cast<float>(size / 2)) *
    linear_scale_;
  linear_velocity_ = std::clamp(linear_velocity_, -1.0F, 1.0F);
  if (std::fabs(linear_velocity_) < 0.1F) {
    linear_velocity_ = 0.0F;
  }

  angular_velocity_ = (1.0F - static_cast<float>(x - hpad) / static_cast<float>(size / 2)) *
    angular_scale_;
  angular_velocity_ = std::clamp(angular_velocity_, -1.0F, 1.0F);

  mouse_pressed_ = true;
  Q_EMIT outputVelocity(linear_velocity_, angular_velocity_, mouse_pressed_, z_velocity_);
  update();
}

void DriveWidget::stop()
{
  linear_velocity_ = 0.0F;
  angular_velocity_ = 0.0F;
  mouse_pressed_ = false;
  Q_EMIT outputVelocity(linear_velocity_, angular_velocity_, mouse_pressed_, z_velocity_);
  update();
}

}  // namespace teleop_rviz_plugin
