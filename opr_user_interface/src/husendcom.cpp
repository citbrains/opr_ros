#include <ros/ros.h>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QIntValidator>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "husendcom.hpp"
#include "opr_msgs/HajimeWalk.h"
#include "opr_msgs/HajimeMotion.h"
#include <std_msgs/Bool.h>

HuSendCom::HuSendCom(QWidget *parent) : QWidget(parent), nh_()
{
    QVBoxLayout *v_layer = new QVBoxLayout;

    QHBoxLayout *walk_layer = new QHBoxLayout;
    walk_layer->addWidget(new QLabel("step:"));
    walk_step_edit = new QLineEdit("0");
    walk_step_edit->setValidator(new QIntValidator(0, 24, this));
    walk_layer->addWidget(walk_step_edit);
    walk_layer->addWidget(new QLabel("period:"));
    walk_period_edit = new QLineEdit("0");
    walk_period_edit->setValidator(new QIntValidator(-24, 24, this));
    walk_layer->addWidget(walk_period_edit);
    walk_layer->addWidget(new QLabel("x:"));
    walk_x_edit = new QLineEdit("0");
    walk_x_edit->setValidator(new QIntValidator(-24, 24, this));
    walk_layer->addWidget(walk_x_edit);
    walk_layer->addWidget(new QLabel("y:"));
    walk_y_edit = new QLineEdit("0");
    walk_y_edit->setValidator(new QIntValidator(-24, 24, this));
    walk_layer->addWidget(walk_y_edit);
    walk_layer->addWidget(new QLabel("angle:"));
    walk_angle_edit = new QLineEdit("0");
    walk_angle_edit->setValidator(new QIntValidator(-24, 24, this));
    walk_layer->addWidget(walk_angle_edit);
    QPushButton *walk_button = new QPushButton("publish");
    walk_layer->addWidget(walk_button);
    v_layer->addLayout(walk_layer);

    QHBoxLayout *motion_layer = new QHBoxLayout;
    motion_layer->addWidget(new QLabel("repeat:"));
    motion_repeat_edit = new QLineEdit("1");
    motion_repeat_edit->setValidator(new QIntValidator(0, 24, this));
    motion_layer->addWidget(motion_repeat_edit);
    motion_layer->addWidget(new QLabel("motion ID:"));
    motion_id_edit = new QLineEdit("0");
    motion_id_edit->setValidator(new QIntValidator(0, 100, this));
    motion_layer->addWidget(motion_id_edit);
    QPushButton *motion_button = new QPushButton("publish");
    motion_layer->addWidget(motion_button);
    v_layer->addLayout(motion_layer);

    QPushButton *cancel_button = new QPushButton("cancel");
    v_layer->addWidget(cancel_button);

    connect(walk_button, SIGNAL(clicked()), this, SLOT(walk_publish()));
    connect(motion_button, SIGNAL(clicked()), this, SLOT(motion_publish()));
    connect(cancel_button, SIGNAL(clicked()), this, SLOT(cancel_publish()));

    setLayout(v_layer);
    
    walk_pub = nh_.advertise<opr_msgs::HajimeWalk>("/hajime_walk/WalkCommand", 2);
    motion_pub = nh_.advertise<opr_msgs::HajimeMotion>("/hajime_walk/MotionCommand", 2);
    cancel_pub = nh_.advertise<std_msgs::Bool>("/hajime_walk/CancelCommand", 1);
}

void HuSendCom::walk_publish()
{
    opr_msgs::HajimeWalk data;

    if(walk_step_edit->text().isEmpty()){
        data.num_step = 0;
        walk_step_edit->setText("0");
    } else{
        data.num_step = walk_step_edit->text().toInt();
    }
    if(walk_period_edit->text().isEmpty()){
        data.period = 0;
        walk_period_edit->setText("0");
    } else{
        data.period = walk_period_edit->text().toInt();
    }
    if(walk_x_edit->text().isEmpty()){
        data.stride_x = 0;
        walk_x_edit->setText("0");
    } else{
        data.stride_x = walk_x_edit->text().toInt();
    }
    if(walk_y_edit->text().isEmpty()){
        data.stride_y = 0;
        walk_y_edit->setText("0");
    } else{
        data.stride_y = walk_y_edit->text().toInt();
    }
    if(walk_angle_edit->text().isEmpty()){
        data.stride_th = 0;
        walk_angle_edit->setText("0");
    } else{
        data.stride_th = walk_angle_edit->text().toInt();
    }

    walk_pub.publish(data);
}

void HuSendCom::motion_publish()
{
    opr_msgs::HajimeMotion data;
    
    if(motion_repeat_edit->text().isEmpty()){
        data.num_repeat = 1;
        motion_repeat_edit->setText("1");
    } else{
        data.num_repeat = motion_repeat_edit->text().toInt();
    }
    if(motion_id_edit->text().isEmpty()){
        data.motion_id = 0;
        motion_id_edit->setText("0");
    } else{
        data.motion_id = motion_id_edit->text().toInt();
    }

    motion_pub.publish(data);
}

void HuSendCom::cancel_publish()
{
    std_msgs::Bool data;
    data.data = false;
    cancel_pub.publish(data);
}
