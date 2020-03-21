#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <QWidget>
#include <QLineEdit>

class HuSendCom : public QWidget
{
    Q_OBJECT
    public:
        HuSendCom(QWidget *parent=0);

    private Q_SLOTS:
        void walk_publish();
        void motion_publish();
        void cancel_publish();

    private:
        QLineEdit *walk_step_edit;
        QLineEdit *walk_x_edit;
        QLineEdit *walk_y_edit;
        QLineEdit *walk_angle_edit;
        QLineEdit *walk_period_edit;
        QLineEdit *motion_repeat_edit;
        QLineEdit *motion_id_edit;

        ros::NodeHandle nh_;
        ros::Publisher walk_pub;
        ros::Publisher motion_pub;
        ros::Publisher cancel_pub;
};

