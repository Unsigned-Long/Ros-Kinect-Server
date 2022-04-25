#include "camera_server.h"
#include "QDebug"
#include "QEventLoop"
#include "QNetworkInterface"
#include "QTimer"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include "QImage"
#include "QBuffer"

Server::Server(ros::NodeHandle &handler, quint16 portMsg, quint16 portRgbImg, quint16 portDepImg, QObject *parent)
    : QObject(parent)
{
  this->registerPublisher(handler);

  // create the server and socket
  this->createServer(portMsg, portRgbImg, portDepImg);

  // create connection
  this->connection();
}

Server::~Server()
{
}

void Server::connection()
{
  connect(this->_serverMsg, &QTcpServer::newConnection, this, [=]() {
    // when a new connection comming
    while (this->_serverMsg->hasPendingConnections())
    {
      // connect
      this->_socketMsg = this->_serverMsg->nextPendingConnection();

      // get info of this connection
      QString ip = this->_socketMsg->peerAddress().toString();
      qint16 port = this->_socketMsg->peerPort();

      ROS_INFO("'new connection [text msg]': {'ip': '%s', 'port': '%d'}", ip.toStdString().c_str(), port);
      qint64 res = this->_socketMsg->write(QString::fromStdString(ns_runtime_flags::INFO_MESSAGE_TOAST + "server msg has been connected").toUtf8());
      if (res == -1)
      {
        ROS_WARN("'server': send message '%s' to client failed", "server msg has been connected");
      }

      // read
      connect(this->_socketMsg, &QTcpSocket::readyRead, this, [=]() {
        while (this->_socketMsg->bytesAvailable())
        {
          QByteArray buf = this->_socketMsg->readAll();
          ROS_INFO("'server' gets message from 'client': '%s'", buf.toStdString().c_str());

          // handler message [start, stop]
          std_msgs::String msg;
          msg.data = buf.toStdString();
          this->_cameraController.publish(msg);
        }
      });
    }
  });

  connect(this->_serverRgbImg, &QTcpServer::newConnection, this, [=]() {
    // when a new connection comming
    while (this->_serverRgbImg->hasPendingConnections())
    {
      // connect
      this->_socketRgbImg = this->_serverRgbImg->nextPendingConnection();

      // get info of this connection
      QString ip = this->_socketRgbImg->peerAddress().toString();
      qint16 port = this->_socketRgbImg->peerPort();

      ROS_INFO("'new connection [color image]': {'ip': '%s', 'port': '%d'}", ip.toStdString().c_str(), port);
    }
  });

  connect(this->_serverDepImg, &QTcpServer::newConnection, this, [=]() {
    // when a new connection comming
    while (this->_serverDepImg->hasPendingConnections())
    {
      // connect
      this->_socketDepImg = this->_serverDepImg->nextPendingConnection();

      // get info of this connection
      QString ip = this->_socketDepImg->peerAddress().toString();
      qint16 port = this->_socketDepImg->peerPort();

      ROS_INFO("'new connection [depth image]': {'ip': '%s', 'port': '%d'}", ip.toStdString().c_str(), port);
    }
  });
}

void Server::createServer(quint16 portMsg, quint16 portRgbImg, quint16 portDepImg)
{
  // create the server and socket
  this->_serverMsg = new QTcpServer(this);
  this->_socketMsg = new QTcpSocket(this);

  this->_serverRgbImg = new QTcpServer(this);
  this->_socketRgbImg = new QTcpSocket(this);

  this->_serverDepImg = new QTcpServer(this);
  this->_socketDepImg = new QTcpSocket(this);
  // listen
  this->_serverMsg->listen(QHostAddress::Any, portMsg);
  this->_serverRgbImg->listen(QHostAddress::Any, portRgbImg);
  this->_serverDepImg->listen(QHostAddress::Any, portDepImg);

  // get ip address list
  QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
  for (auto &elem : ipAddressesList)
  {
    ROS_INFO("'vaild ip': '%s'", elem.toString().toStdString().c_str());
  }

  ROS_INFO("'server msg port': '%d'", this->_serverMsg->serverPort());
  ROS_INFO("'server rgb img port': '%d'", this->_serverRgbImg->serverPort());
  ROS_INFO("'server depth img port': '%d'", this->_serverDepImg->serverPort());
  return;
}

void Server::sendState2Client(const std_msgs::String::ConstPtr &msg)
{
  QEventLoop loop;

  ROS_INFO("'server' gets message from 'dk_camera': '%s'", msg->data.c_str());

  auto data = msg->data;
  // send to the client: color, depth, time
  if (this->_socketMsg != nullptr)
  {
    if (this->_socketMsg->isOpen())
    {
      qint64 res = this->_socketMsg->write(QString::fromStdString(data).toUtf8());
      if (res == -1)
      {
        ROS_WARN("'server': send message '%s' to client failed", data.c_str());
      }
    }
    else
    {
      ROS_WARN("'server': '%s'", "scoket is not connected");
    }
  }

  QTimer::singleShot(30, &loop, SLOT(quit()));
  loop.exec();
  return;
}

void Server::sendColorImg2Client(const sensor_msgs::Image::ConstPtr &img)
{
  // cv::Mat cvImg = cv_bridge::toCvCopy(*img, "rgb8")->image;
  // cv::imshow("win", cvImg);
  // todo: splite image and message

  QEventLoop loop;

  ROS_INFO("'server' gets color image from 'dk_camera'");

  // send to the client: color, depth, time
  if (this->_socketRgbImg != nullptr)
  {
    if (this->_socketRgbImg->isOpen())
    {
      QByteArray ary;
      QBuffer buf(&ary);
      buf.open(QIODevice::WriteOnly);
      // construct a QImage
      QImage qimg(img->data.data(), img->width, img->height, QImage ::Format::Format_RGB888);
      qimg.save(&buf, "jpg");
      ROS_INFO("'server color img byte size': '%d'", ary.size());
      int idx = 0;
      int flag = 0;
      while (true)
      {
        if (idx + 2043 > ary.size())
        {
          // todo, left size!!!

          int sendSize = ary.size() - idx;

          // head[1], len[4], data[...], tail[2048-1-4-...]
          std::string lenStr = std::to_string(sendSize);
          if (lenStr.size() < 4)
          {
            lenStr = std::string(4 - lenStr.size(), '0') + lenStr;
          }
          this->_socketRgbImg->write(QByteArray(1, 'E') + QByteArray(lenStr.data(), 4) + QByteArray(ary.data() + idx, sendSize) + QByteArray(2048 - 1 - 4 - sendSize, ' '));
          break;
        }
        else if (idx + 2043 == ary.size())
        {
          int sendSize = 2043;
          // head[1], len[4], data[2043]
          this->_socketRgbImg->write(QByteArray(1, 'E') + QByteArray("2043") + QByteArray(ary.data() + idx, sendSize));
          break;
        }
        else
        {
          int sendSize = 2043;
          if (flag == 0)
          {
            // head[1], len[4], data[2043]
            this->_socketRgbImg->write(QByteArray(1, 'S') + QByteArray(4, ' ') + QByteArray(ary.data() + idx, sendSize));
            ++flag;
          }
          else
          {
            this->_socketRgbImg->write(QByteArray(1, 'M') + QByteArray(4, ' ') + QByteArray(ary.data() + idx, sendSize));
          }
          idx += 2043;
        }
      }
    }
    else
    {
      ROS_WARN("'server': '%s'", "scoket is not connected");
    }
  }

  QTimer::singleShot(30, &loop, SLOT(quit()));
  loop.exec();
  return;
}

void Server::sendDepthImg2Client(const sensor_msgs::Image::ConstPtr &img)
{
  // cv::Mat cvImg = cv_bridge::toCvCopy(*img, "rgb8")->image;
  // cv::imshow("win", cvImg);
  // // todo: splite image and message
  // return;

  QEventLoop loop;

  ROS_INFO("'server' gets depth image from 'dk_camera'");

  // send to the client: color, depth, time
  if (this->_socketDepImg != nullptr)
  {
    if (this->_socketDepImg->isOpen())
    {
      QByteArray ary;
      QBuffer buf(&ary);
      buf.open(QIODevice::WriteOnly);
      // construct a QImage
      QImage qimg(img->data.data(), img->width, img->height, QImage ::Format::Format_RGB888);
      qimg.save(&buf, "jpg");
      ROS_INFO("'server depth img byte size': '%d'", ary.size());
      int idx = 0;
      int flag = 0;
      while (true)
      {
        if (idx + 2043 > ary.size())
        {
          // todo, left size!!!

          int sendSize = ary.size() - idx;

          // head[1], len[4], data[...], tail[2048-1-4-...]
          std::string lenStr = std::to_string(sendSize);
          if (lenStr.size() < 4)
          {
            lenStr = std::string(4 - lenStr.size(), '0') + lenStr;
          }
          this->_socketDepImg->write(QByteArray(1, 'E') + QByteArray(lenStr.data(), 4) + QByteArray(ary.data() + idx, sendSize) + QByteArray(2048 - 1 - 4 - sendSize, ' '));
          break;
        }
        else if (idx + 2043 == ary.size())
        {
          int sendSize = 2043;
          // head[1], len[4], data[2043]
          this->_socketDepImg->write(QByteArray(1, 'E') + QByteArray("2043") + QByteArray(ary.data() + idx, sendSize));
          break;
        }
        else
        {
          int sendSize = 2043;
          if (flag == 0)
          {
            // head[1], len[4], data[2043]
            this->_socketDepImg->write(QByteArray(1, 'S') + QByteArray(4, ' ') + QByteArray(ary.data() + idx, sendSize));
            ++flag;
          }
          else
          {
            this->_socketDepImg->write(QByteArray(1, 'M') + QByteArray(4, ' ') + QByteArray(ary.data() + idx, sendSize));
          }
          idx += 2043;
        }
      }
    }
    else
    {
      ROS_WARN("'server': '%s'", "scoket is not connected");
    }
  }

  QTimer::singleShot(30, &loop, SLOT(quit()));
  loop.exec();

  return;
}

void Server::registerPublisher(ros::NodeHandle &handler)
{
  // start, stop
  _cameraController = handler.advertise<std_msgs::String>("camera_control", 10);
  _stateChecker = handler.subscribe("runtime_state", 10, &Server::sendState2Client, this);
  _colorImgSuber = handler.subscribe("dk_color", 36, &Server::sendColorImg2Client, this);
  _depthImgSuber = handler.subscribe("dk_depth", 36, &Server::sendDepthImg2Client, this);
}