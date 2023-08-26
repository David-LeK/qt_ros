#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <custom_msg/gps_msg.h>
#include <custom_msg/mpu_msg.h>
#include <custom_msg/encoder_output_msg.h>
#include <custom_msg/encoder_input_msg.h>
#include <custom_msg/stanley_constants.h>
#include <custom_msg/stanley_outputs.h>
#include "pathcontroller.h"
#include "pathreference.h"
#include <QGeoPath>
#include <QFileDialog>
#include <QXmlStreamReader>
#include <QStringList>
#include <GeographicLib/UTMUPS.hpp>
#include "qcustomplot.h"
#include <QMessageBox>

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  void gpsCallback(const custom_msg::gps_msg::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void mpuCallback(const custom_msg::mpu_msg::ConstPtr& msg);
  void pidCallback(const custom_msg::encoder_output_msg::ConstPtr& msg);
  void pidctrlCallback(const custom_msg::encoder_input_msg::ConstPtr& msg);
  void stanleyCallback(const custom_msg::stanley_outputs::ConstPtr& msg);

public slots:
  void spinOnce();

private slots:
  void on_kml_browse_clicked();

  void on_pid_send_clicked();

  void on_stop_send_clicked();

  void on_clear_button_clicked();

  void set_up_graph();

  void set_up_graph_timer();

  void realtime_pid_plot();

  void on_stanley_send_clicked();

  void on_reset_stanley_clicked();

  void on_inverse_stanley_clicked();

private:
  Ui::MainWindow *ui;
  QTimer *ros_timer;

  ros::NodeHandlePtr nh_;

  ros::Subscriber gps_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber mpu_sub_;
  ros::Subscriber pid_sub_;
  ros::Subscriber pid_ctrl_set_sub_;
  ros::Subscriber stanley_sub_;

  ros::Publisher easting_pub_;
  ros::Publisher northing_pub_;
  ros::Publisher pid_ctrl_pub_;
  ros::Publisher stanley_pub_;
  ros::Publisher stop_pub_;

  PathController controller;
  PathReference refpathcontroller;
  QGeoPath path;

  QVector<double> cubic_x;
  QVector<double> cubic_y;
  QVector<double> northing_set;
  QVector<double> easting_set;

  float setpoint1;
  float setpoint2;

  QCPGraph* xyPoints;
  QCPGraph* cubicPoints;
  QTimer* dataTimer;

  float rpm1;
  float rpm2;
};

#endif // MAIN_WINDOW_H
