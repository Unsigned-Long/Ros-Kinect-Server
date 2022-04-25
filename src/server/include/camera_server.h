#ifndef SERVER_H
#define SERVER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <memory>

namespace ns_runtime_flags
{
  // format: flag[message...]
  static const std::string COLOR_DEPTH_TIME = "#0";
  static const std::string INFO_MESSAGE_TEXTVIEW = "#1";
  static const std::string INFO_MESSAGE_TOAST = "#2";
  static const std::string IMU_MESSAGE = "#3";
} // namespace ns_runtime_flags

class Server : public QObject
{
  // Q_OBJECT

public:
  Server(ros::NodeHandle &handler, quint16 portMsg, quint16 portRgbImg, quint16 portDepImg, QObject *parent = nullptr);
  ~Server();

  void sendState2Client(const std_msgs::String::ConstPtr &msg);

  void sendColorImg2Client(const sensor_msgs::Image::ConstPtr &img);

  void sendDepthImg2Client(const sensor_msgs::Image::ConstPtr &img);

  void registerPublisher(ros::NodeHandle &handler);

  void createServer(quint16 portMsg, quint16 portRgbImg, quint16 portDepImg);

  void connection();

public:
  QTcpServer *_serverMsg = nullptr;
  QTcpSocket *_socketMsg = nullptr;

  QTcpServer *_serverRgbImg = nullptr;
  QTcpSocket *_socketRgbImg = nullptr;

  QTcpServer *_serverDepImg = nullptr;
  QTcpSocket *_socketDepImg = nullptr;

  ros::NodeHandle _handler;
  ros::Publisher _cameraController;
  ros::Subscriber _stateChecker;
  ros::Subscriber _colorImgSuber;
  ros::Subscriber _depthImgSuber;
};

#endif // SERVER_H
