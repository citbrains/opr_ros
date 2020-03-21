#include <ros/ros.h>
#include <QApplication>
#include <QWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include "statusui.hpp"
#include "husendcom.hpp"

StatusUI::StatusUI(QTabWidget *parent) : QTabWidget(parent), nh_()
{

    husendcom_tab = new HuSendCom();
    husendcom_tab->update();
    addTab(husendcom_tab, "HuSendCom");

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
