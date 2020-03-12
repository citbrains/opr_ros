#include <ros/ros.h>
#include <QApplication>
#include <QWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include "panel.hpp"
#include "husendcom.hpp"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Panel, rviz::Panel)

Panel::Panel(QWidget *parent) : rviz::Panel(parent)
{
    QTabWidget *tab_widget = new QTabWidget();
    QVBoxLayout *layout = new QVBoxLayout;

    husendcom_tab = new HuSendCom();
    husendcom_tab->update();
    tab_widget->addTab(husendcom_tab, "HuSendCom");

    layout->addWidget(tab_widget);
    setLayout(layout);

}

void Panel::save(rviz::Config config) const
{
}

void Panel::load(const rviz::Config &config)
{
}
