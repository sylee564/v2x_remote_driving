// src/manager/ManagerNode.cpp
#include <QApplication>
#include <QTimer>
#include <QObject>
#include <ros/ros.h>
#include <ros/package.h>
#include <csignal>
#include <string>
#include <iostream>

#include "tod_manager/Manager.h"

static QApplication* g_app_ptr = nullptr;

static void handleSigInt(int)
{
  if (ros::ok()) {
    ros::shutdown();
  }
  if (g_app_ptr) {
    QMetaObject::invokeMethod(g_app_ptr, "quit", Qt::QueuedConnection);
  }
}

int main(int argc, char** argv)
{
#if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps, true);
#endif

  ros::init(argc, argv, "tod_manager_node", ros::init_options::NoSigintHandler);

  QApplication app(argc, argv);
  g_app_ptr = &app;
  QCoreApplication::setApplicationName("TOD Manager");
  QCoreApplication::setOrganizationName("YourOrg");
  // QApplication::setQuitOnLastWindowClosed(true); // Optional (Qt provides default true).

  std::signal(SIGINT, handleSigInt);

  ros::NodeHandle pnh("~");
  std::string yaml_path_param;
  std::string key_param;

  const std::string pkg_path = ros::package::getPath("tod_manager");
  const std::string default_yaml = pkg_path + "/config/V2X_Config.yaml";
  const std::string default_key  = "ConfigNodes";

  pnh.param<std::string>("v2x_config_path", yaml_path_param, default_yaml);
  pnh.param<std::string>("v2x_config_key",  key_param,       default_key);

  if (yaml_path_param.empty()) {
    yaml_path_param = default_yaml;
  }

  try {
    Manager manager(yaml_path_param, key_param);
    manager.show_window();

    ros::AsyncSpinner spinner(4);
    spinner.start();

    QTimer rosOkWatcher;
    QObject::connect(&rosOkWatcher, &QTimer::timeout, [&]() {
      if (!ros::ok()) {
        app.quit();
      }
    });
    rosOkWatcher.start(100);

    const int rc = app.exec();

    if (ros::ok()) {
      ros::shutdown();
    }
    spinner.stop();
    return rc;
  }
  catch (const std::exception& ex) {
    std::cerr << "[ManagerNode] Exception: " << ex.what() << std::endl;
    if (ros::ok()) {
      ros::shutdown();
    }
    return 1;
  }
  catch (...) {
    std::cerr << "[ManagerNode] Unknown exception occurred." << std::endl;
    if (ros::ok()) {
      ros::shutdown();
    }
    return 1;
  }
}