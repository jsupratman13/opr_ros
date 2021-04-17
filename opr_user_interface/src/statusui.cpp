#include <ros/ros.h>
#include <QApplication>
#include <QWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include "statusui.hpp"
#include "husendcom.hpp"
#include "color_calibration.hpp"

StatusUI::StatusUI(QTabWidget *parent) : QTabWidget(parent), nh_()
{

    husendcom_tab = new HuSendCom();
    husendcom_tab->update();
    addTab(husendcom_tab, "HuSendCom");

    color_calibration_tab = new ColorCalibration();
    color_calibration_tab->update();
    addTab(color_calibration_tab, "ColorCalibration");

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "opr_rviz");

    QApplication app(argc, argv);
    QTabWidget *window = new QTabWidget;
    QVBoxLayout *layout = new QVBoxLayout;

    StatusUI *gui = new StatusUI(window);
    layout->addWidget(gui);
    window->setLayout(layout);
    window->show();

    ros::Rate rate(20);
    while(ros::ok()){
        ros::spinOnce();
        app.processEvents();
        rate.sleep();
    }

}
