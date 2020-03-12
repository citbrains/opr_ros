#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <QWidget>
#include <QTabWidget>

class HuSendCom;

class StatusUI : public QTabWidget
{
    Q_OBJECT
    public:
        StatusUI(QTabWidget *parent);
        HuSendCom *husendcom_tab;

    private:
        ros::NodeHandle nh_;
};
