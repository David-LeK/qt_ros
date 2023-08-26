/*
# MIT License

# Copyright (c) 2022 Kristopher Krasnosky

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
*/


#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  nh_.reset(new ros::NodeHandle("~"));

  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate


  // setup subscriber by according to the ~/chatter_topic param

  std::string gps_topic;
  nh_->param<std::string>("gps_topic", gps_topic,"/GPS_data");
  gps_sub_ = nh_->subscribe<custom_msg::gps_msg>(gps_topic, 1, &MainWindow::gpsCallback, this);

  std::string path_topic;
  nh_->param<std::string>("path_topic", path_topic,"/path");
  path_sub_ = nh_->subscribe<nav_msgs::Path>(path_topic, 1, &MainWindow::pathCallback, this);

  std::string mpu_topic;
  nh_->param<std::string>("mpu_topic", mpu_topic,"/MPU_data");
  mpu_sub_ = nh_->subscribe<custom_msg::mpu_msg>(mpu_topic, 1, &MainWindow::mpuCallback, this);

  std::string pid_topic;
  nh_->param<std::string>("pid_topic", pid_topic,"/PID_data");
  pid_sub_ = nh_->subscribe<custom_msg::encoder_output_msg>(pid_topic, 1, &MainWindow::pidCallback, this);

  std::string pid_ctrl_set_topic;
  nh_->param<std::string>("pid_ctrl_set_topic", pid_ctrl_set_topic,"/cmd_vel");
  pid_ctrl_set_sub_ = nh_->subscribe<custom_msg::encoder_input_msg>(pid_ctrl_set_topic, 1, &MainWindow::pidctrlCallback, this);

  std::string stanley_output_topic;
  nh_->param<std::string>("stanley_output_topic", stanley_output_topic,"/Stanley_outputs");
  stanley_sub_ = nh_->subscribe<custom_msg::stanley_outputs>(stanley_output_topic, 1, &MainWindow::stanleyCallback, this);

  // publish a message on the channel specified by ~/hello_topic param

  std::string easting_topic;
  nh_->param<std::string>("easting_topic", easting_topic,"easting_kml");
  easting_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(easting_topic,1);

  std::string northing_topic;
  nh_->param<std::string>("northing_topic", northing_topic,"northing_kml");
  northing_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(northing_topic,1);

  std::string pid_ctrl_topic;
  nh_->param<std::string>("pid_ctrl_topic", pid_ctrl_topic,"/PID_ctrl");
  pid_ctrl_pub_ = nh_->advertise<custom_msg::encoder_input_msg>(pid_ctrl_topic,1);

  std::string stanley_topic;
  nh_->param<std::string>("stanley_topic", stanley_topic,"/Stanley_ctrl");
  stanley_pub_ = nh_->advertise<custom_msg::stanley_constants>(stanley_topic,1);

  std::string stop_topic;
  nh_->param<std::string>("stop_topic", stop_topic,"/stop");
  stop_pub_ = nh_->advertise<std_msgs::Int16>(stop_topic,1);

  const char* ros_master_uri = std::getenv("ROS_MASTER_URI");
  ui->ros_uri->setText(ros_master_uri);

  ui->google_maps->rootContext()->setContextProperty("pathController", &controller);
  ui->google_maps->rootContext()->setContextProperty("refpathController", &refpathcontroller);
  ui->google_maps->setSource(QUrl(QStringLiteral("file::/qrc/mapnavigation.qml")));

  setpoint1 = 0.0;
  setpoint2 = 0.0;

  ui->cubic_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
  ui->PID_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
  set_up_graph();
  showMaximized();

  set_up_graph_timer();
}

MainWindow::~MainWindow()
{
  delete ui;
  delete ros_timer;
}


void MainWindow::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

void MainWindow::gpsCallback(const custom_msg::gps_msg::ConstPtr &msg){
  path.addCoordinate(QGeoCoordinate(static_cast<double>(msg->latitude), static_cast<double>(msg->longitude)));
  controller.setGeoPath(path);
  auto latitude = QString::number(static_cast<double>(msg->latitude), 'f', 6);
  auto longitude = QString::number(static_cast<double>(msg->longitude), 'f', 6);
  auto easting = QString::number(static_cast<double>(msg->easting), 'f', 6);
  auto northing = QString::number(static_cast<double>(msg->northing), 'f', 6);
  auto speed_kmh = QString::number(static_cast<double>(msg->speed_kmh), 'f', 6);
  auto tracking_angle = QString::number(static_cast<double>(msg->tracking_angle), 'f', 6);
  ui->latitude->setText(latitude);
  ui->longitude->setText(longitude);
  ui->easting->setText(easting);
  ui->northing->setText(northing);
  ui->speed_kmh->setText(speed_kmh);
  ui->gps_angle->setText(tracking_angle);
  northing_set.append(northing.toDouble()-1191350.0);
  easting_set.append(easting.toDouble()-681500.0);
  int graph_count = ui->cubic_plot->graphCount();
  if (graph_count == 0)
  {
    ui->cubic_plot->addGraph();
  }
  xyPoints->setData(easting_set, northing_set);
  ui->cubic_plot->graph(0)->setLineStyle((static_cast<QCPGraph::LineStyle>(QCPGraph::lsNone)));
  ui->cubic_plot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
  ui->cubic_plot->xAxis->setLabel("x");
  ui->cubic_plot->yAxis->setLabel("y");
  ui->cubic_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
  ui->cubic_plot->axisRect()->setupFullAxesBox();
  ui->cubic_plot->rescaleAxes();
  ui->cubic_plot->replot();
  ui->cubic_plot->update();
}

void MainWindow::pathCallback(const nav_msgs::Path::ConstPtr &msg){
  auto path_cubic = msg->poses;
  QVector<double> temp_x;
  QVector<double> temp_y;
  QVector<double> scale_x;
  QVector<double> scale_y;
  for (unsigned long i=0; i < path_cubic.size(); i++) {
    temp_x.append(path_cubic[i].pose.position.x);
    temp_y.append(path_cubic[i].pose.position.y);
  }
  cubic_x = temp_x;
  cubic_y = temp_y;
  for (int i=0; i < cubic_x.size(); i++) {
    scale_x.append(cubic_x[i]-681500.0);
    scale_y.append(cubic_y[i]-1191350.0);
  }
  ui->total_path->setText(QString::number(cubic_x.size()-1));
  double latitude_ref;
  double longitude_ref;
  QGeoPath refpath;
  for (int i = 0; i < cubic_x.size(); i++)
  {
    GeographicLib::UTMUPS::Reverse(48, true, cubic_x[i], cubic_y[i], latitude_ref, longitude_ref);
    refpath.addCoordinate(QGeoCoordinate(latitude_ref, longitude_ref));
  }
  refpathcontroller.setGeoPathref(refpath);
  cubicPoints->setData(scale_x, scale_y);
  ui->cubic_plot->rescaleAxes();
  ui->cubic_plot->replot();
  ui->cubic_plot->update();
}

void MainWindow::mpuCallback(const custom_msg::mpu_msg::ConstPtr &msg){
  ui->roll->setText(QString::number(static_cast<double>(msg->roll), 'f', 6));
  ui->pitch->setText(QString::number(static_cast<double>(msg->pitch), 'f', 6));
  ui->yaw->setText(QString::number(static_cast<double>(msg->yaw), 'f', 6));
  controller.rot = msg->yaw;
}

void MainWindow::pidCallback(const custom_msg::encoder_output_msg::ConstPtr& msg){
  rpm1 = msg->output_rpm_m1;
  rpm2 = msg->output_rpm_m2;
  ui->rpm1->setText(QString::number(static_cast<double>(msg->output_rpm_m1), 'f', 6));
  ui->rpm2->setText(QString::number(static_cast<double>(msg->output_rpm_m2), 'f', 6));
  ui->controller1->setText(QString::number(static_cast<double>(msg->output_controller_m1), 'f', 6));
  ui->controller2->setText(QString::number(static_cast<double>(msg->output_controller_m2), 'f', 6));
  ui->error1->setText(QString::number(static_cast<double>(msg->error_m1), 'f', 6));
  ui->error2->setText(QString::number(static_cast<double>(msg->error_m2), 'f', 6));
}

void MainWindow::pidctrlCallback(const custom_msg::encoder_input_msg::ConstPtr &msg){
  setpoint1 = msg->input_setpoint_m1;
  setpoint2 = msg->input_setpoint_m2;
  ui->cmdvel_setpoint1->setText(QString::number(static_cast<double>(setpoint1), 'f', 6));
  ui->cmdvel_setpoint2->setText(QString::number(static_cast<double>(setpoint2), 'f', 6));
}

void MainWindow::on_kml_browse_clicked()
{
  QString filePath = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("All Files (*.*)"));

  if (!filePath.isEmpty()) {
      ui->kml_path->setText(filePath);
  }

  QFile file(filePath);
  QList<double> lats_kml;
  QList<double> lons_kml;
  QList<double> easting_kml;
  QList<double> northing_kml;
  double northing = 0.0;
  double easting = 0.0;

  int zone;
  bool northp;

  if (file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
      QXmlStreamReader reader(&file);

      while (!reader.atEnd())
      {
          reader.readNext();

          if (reader.isStartElement() && reader.name() == "Placemark")
          {
              QString coordinates;
              while (!reader.atEnd())
              {
                  reader.readNext();

                  if (reader.isEndElement() && reader.name() == "Placemark")
                  {
                      break;
                  }

                  if (reader.isStartElement() && reader.name() == "coordinates")
                  {
                      coordinates = reader.readElementText().trimmed();
                  }
              }

              if (!coordinates.isEmpty())
              {
                  QStringList coords = coordinates.split(" ");
                  foreach (QString coord, coords)
                  {
                      QStringList latlon = coord.split(",");
                      if (latlon.size() == 3)
                      {
                          double lat = latlon[1].toDouble();
                          double lon = latlon[0].toDouble();
                          lats_kml.append(lat);
                          lons_kml.append(lon);
                      }
                  }
              }
          }
      }
      file.close();
  }
  for (int i = 0; i < lats_kml.size(); i++)
  {
      GeographicLib::UTMUPS::Forward(lats_kml[i], lons_kml[i], zone, northp, easting, northing);
      easting_kml.append(easting);
      northing_kml.append(northing);
  }
  std_msgs::Float32MultiArray msg1;
  std_msgs::Float32MultiArray msg2;
  for (int i = 0; i < lats_kml.size(); i++)
  {
    msg1.data.push_back(static_cast<float>(easting_kml[i]));
    msg2.data.push_back(static_cast<float>(northing_kml[i]));
  }
  easting_pub_.publish(msg1);
  northing_pub_.publish(msg2);
}

void MainWindow::on_pid_send_clicked()
{
  custom_msg::encoder_input_msg msg;
  if (ui->ki1->text() == "")
  {
    ui->ki1->setText("0");
  }
  if (ui->ki2->text() == "")
  {
    ui->ki2->setText("0");
  }
  if (ui->kp1->text() == "")
  {
    ui->kp1->setText("0");
  }
  if (ui->kp2->text() == "")
  {
    ui->kp2->setText("0");
  }
  if (ui->kd1->text() == "")
  {
    ui->kd1->setText("0");
  }
  if (ui->kd2->text() == "")
  {
    ui->kd2->setText("0");
  }
  msg.input_setpoint_m1 = ui->setpoint1->text().toFloat();
  setpoint1 = ui->setpoint1->text().toFloat();
  msg.input_setpoint_m2 = ui->setpoint2->text().toFloat();
  setpoint2 = ui->setpoint2->text().toFloat();
  msg.input_Kp_m1 = ui->kp1->text().toFloat();
  msg.input_Ki_m1 = ui->ki1->text().toFloat();
  msg.input_Kd_m1 = ui->kd1->text().toFloat();
  msg.input_Kp_m2 = ui->kp2->text().toFloat();
  msg.input_Ki_m2 = ui->ki2->text().toFloat();
  msg.input_Kd_m2 = ui->kd2->text().toFloat();
  pid_ctrl_pub_.publish(msg);
  std_msgs::Int16 stop;
  stop.data = 0;
  stop_pub_.publish(stop);
}

void MainWindow::on_stop_send_clicked()
{
  std_msgs::Int16 stop;
  if (ui->force_Stop->isChecked())
  {
    stop.data = 2;
  }
  else {
    stop.data = 1;
  }
  stop_pub_.publish(stop);
}

void MainWindow::on_clear_button_clicked()
{
  path.clearPath();
  northing_set.clear();
  easting_set.clear();
  ui->cubic_plot->clearGraphs();
  set_up_graph();
}

void MainWindow::set_up_graph()
{
  //GPS graph
  xyPoints = new QCPGraph(ui->cubic_plot->xAxis, ui->cubic_plot->yAxis);
  xyPoints->setAdaptiveSampling(false);
  xyPoints->setLineStyle(QCPGraph::lsNone);
  xyPoints->setScatterStyle(QCPScatterStyle::ssCircle);
  xyPoints->setPen(QPen(QBrush(Qt::red), 2));

  //Cubic graph
  cubicPoints = new QCPGraph(ui->cubic_plot->xAxis, ui->cubic_plot->yAxis);
  cubicPoints->setAdaptiveSampling(false);
  cubicPoints->setLineStyle(QCPGraph::lsNone);
  cubicPoints->setScatterStyle(QCPScatterStyle::ssCircle);
  cubicPoints->setPen(QPen(QBrush(Qt::blue), 2));
}

void MainWindow::set_up_graph_timer()
{
  dataTimer = new QTimer(this);

  //Legend
  ui->PID_plot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
  ui->PID_plot->legend->setVisible(true);
  QFont legendFont = font();  // start out with MainWindow's font..
  legendFont.setPointSize(9); // and make a bit smaller for legend
  ui->PID_plot->legend->setFont(legendFont);
  ui->PID_plot->legend->setBrush(QBrush(QColor(255,255,255,230)));
  // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
  ui->PID_plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

  ui->PID_plot->addGraph(); // blue line
  QPen bluePen;
  bluePen.setColor(Qt::blue);
  bluePen.setWidthF(1);
  ui->PID_plot->graph(0)->setPen(bluePen);
  ui->PID_plot->graph(0)->setName("RPM1");
  ui->PID_plot->addGraph(); // red line
  QPen redPen;
  redPen.setColor(QColor(255, 110, 40));
  redPen.setWidthF(1);
  ui->PID_plot->graph(1)->setPen(redPen);
  ui->PID_plot->graph(1)->setName("RPM2");
  QPen Penforsetpoint1;
  Penforsetpoint1.setColor(QColor(204, 0, 204));
  Penforsetpoint1.setWidthF(3);
  ui->PID_plot->addGraph(); // Setpoint1
  ui->PID_plot->graph(2)->setPen(Penforsetpoint1);
  ui->PID_plot->graph(2)->setName("Setpoint 1");
  ui->PID_plot->addGraph(); // Setpoint2
  QPen Penforsetpoint2;
  Penforsetpoint2.setColor(QColor(102, 0, 51));
  Penforsetpoint2.setWidthF(3);
  ui->PID_plot->graph(3)->setPen(Penforsetpoint2);
  ui->PID_plot->graph(3)->setName("Setpoint 2");

  QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
  timeTicker->setTimeFormat("%h:%m:%s");
  ui->PID_plot->xAxis->setTicker(timeTicker);
  ui->PID_plot->xAxis->setTickLabels(false);
  ui->PID_plot->axisRect()->setupFullAxesBox();
  ui->PID_plot->yAxis->setRange(-1.2, 1.2);

  // make left and bottom axes transfer their ranges to right and top axes:
  connect(ui->PID_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->PID_plot->xAxis2, SLOT(setRange(QCPRange)));
  connect(ui->PID_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->PID_plot->yAxis2, SLOT(setRange(QCPRange)));

  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
  connect(dataTimer, SIGNAL(timeout()), this, SLOT(realtime_pid_plot()));
  dataTimer->start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::realtime_pid_plot()
{
  static QTime time(QTime::currentTime());
  // calculate two new data points:
  double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
  //ui->PID_plot->xAxis->setRange(key, 300, Qt::AlignRight); // Set range to 5 minutes (300 seconds)
  static double lastPointKey = 0;
  if (key-lastPointKey > 0.001) // at most add point every 1 ms
  {
    // add data to lines:
    ui->PID_plot->graph(0)->addData(key, static_cast<double>(rpm1));
    ui->PID_plot->graph(1)->addData(key, static_cast<double>(rpm2));
    ui->PID_plot->graph(2)->addData(key, static_cast<double>(setpoint1));
    ui->PID_plot->graph(3)->addData(key, static_cast<double>(setpoint2));
    // rescale value (vertical) axis to fit the current data:
    ui->PID_plot->graph(0)->rescaleValueAxis(true);
    ui->PID_plot->graph(1)->rescaleValueAxis(true);
    ui->PID_plot->graph(2)->rescaleValueAxis(true);
    ui->PID_plot->graph(3)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->PID_plot->xAxis->setRange(key, 8, Qt::AlignRight);
  ui->PID_plot->replot();
}

void MainWindow::on_stanley_send_clicked()
{
  custom_msg::stanley_constants msg;
  if (ui->ki1_2->text() == "")
  {
    ui->ki1_2->setText("0");
  }
  if (ui->ki2_2->text() == "")
  {
    ui->ki2_2->setText("0");
  }
  if (ui->kp1_2->text() == "")
  {
    ui->kp1_2->setText("0");
  }
  if (ui->kp2_2->text() == "")
  {
    ui->kp2_2->setText("0");
  }
  if (ui->kd1_2->text() == "")
  {
    ui->kd1_2->setText("0");
  }
  if (ui->kd2_2->text() == "")
  {
    ui->kd2_2->setText("0");
  }
  msg.V_desired = ui->vdesired->text().toFloat();
  msg.K = ui->k_set->text().toFloat();
  msg.input_Kp_m1 = ui->kp1_2->text().toFloat();
  msg.input_Ki_m1 = ui->ki1_2->text().toFloat();
  msg.input_Kd_m1 = ui->kd1_2->text().toFloat();
  msg.input_Kp_m2 = ui->kp2_2->text().toFloat();
  msg.input_Ki_m2 = ui->ki2_2->text().toFloat();
  msg.input_Kd_m2 = ui->kd2_2->text().toFloat();
  stanley_pub_.publish(msg);
  std_msgs::Int16 stop;
  stop.data = 0;
  stop_pub_.publish(stop);
}

void MainWindow::stanleyCallback(const custom_msg::stanley_outputs::ConstPtr &msg)
{
  ui->theta_d->setText(QString::number(static_cast<double>(msg->theta_d)*180.0/3.14, 'f', 6));
  ui->theta_e->setText(QString::number(static_cast<double>(msg->theta_e)*180.0/3.14, 'f', 6));
  ui->delta->setText(QString::number(static_cast<double>(msg->delta)*180.0/3.14, 'f', 6));
  ui->e_fa->setText(QString::number(static_cast<double>(msg->e_fa), 'f', 6));
  ui->v_linear->setText(QString::number(static_cast<double>(msg->v_linear), 'f', 6));
  ui->omega->setText(QString::number(static_cast<double>(msg->omega), 'f', 6));
  ui->steering->setText(QString::number(static_cast<double>(msg->steering_angle)*180.0/3.14, 'f', 6));
  ui->car_yaw->setText(QString::number(static_cast<double>(msg->car_yaw)*180.0/3.14, 'f', 6));
  ui->ref_yaw->setText(QString::number(static_cast<double>(msg->ref_yaw)*180.0/3.14, 'f', 6));
  ui->dx->setText(QString::number(static_cast<double>(msg->dx), 'f', 6));
  ui->dy->setText(QString::number(static_cast<double>(msg->dy), 'f', 6));
  ui->target_radius->setText(QString::number(static_cast<double>(msg->target_radius), 'f', 6));
  ui->distance->setText(QString::number(static_cast<double>(msg->distance), 'f', 6));
  ui->total_path->setText(QString::number(msg->total_path_index));
  ui->current_index->setText(QString::number(msg->current_path_index));
}

void MainWindow::on_reset_stanley_clicked()
{
  std_msgs::Int16 stop;
  stop.data = 3;
  stop_pub_.publish(stop);
}

void MainWindow::on_inverse_stanley_clicked()
{
  std_msgs::Int16 stop;
  stop.data = 4;
  stop_pub_.publish(stop);
}
