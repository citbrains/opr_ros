#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QWidget>
#include <QTabWidget>

class HuSendCom;

class Panel : public rviz::Panel
{
    Q_OBJECT
    public:
        Panel(QWidget *parent=0);
        HuSendCom *husendcom_tab;

        virtual void load(const rviz::Config &config);
        virtual void save(rviz::Config config) const;

    private:
        QTabWidget *tab_widget;
};
