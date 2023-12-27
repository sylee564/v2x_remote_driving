#include "ros/ros.h"
#include <QtWidgets/QApplication>
#include "InputDeviceController.h"

int main(int argc, char **argv) {
  QApplication a(argc, argv);
  InputDeviceController device(argc, argv);

  a.setQuitOnLastWindowClosed(false);
  return a.exec();
}