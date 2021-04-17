#include <ros/ros.h>
#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QPixmap>
#include <QImage>
#include <QComboBox>
#include <QMouseEvent>
#include "opr_msgs/SetColor.h"
#include "opr_msgs/Color.h"
#include "color_calibration.hpp"

ColorCalibration::ColorCalibration(QWidget *parent) : QWidget(parent), nh_(), it_(nh_)
{
    QHBoxLayout *h_layer = new QHBoxLayout;

    QVBoxLayout *image_layer = new QVBoxLayout;
    imageLabel = new QLabel;
    QImage img(300, 300, QImage::Format_RGB888);
    imageLabel->setPixmap(QPixmap::fromImage(img));
    image_layer->addWidget(imageLabel);
    h_layer->addLayout(image_layer);

    QVBoxLayout *ui_layer = new QVBoxLayout;
    colorMode = new QComboBox;
    colorMode->insertItem(7, "WHITE");
    colorMode->insertItem(8, "GREEN");
    QPushButton *saveButton = new QPushButton("SAVE");
    QPushButton *loadButton = new QPushButton("LOAD");
    QPushButton *clearButton = new QPushButton("CLEAR");
    ui_layer->addWidget(colorMode);
    ui_layer->addWidget(saveButton);
    ui_layer->addWidget(loadButton);
    ui_layer->addWidget(clearButton);
    h_layer->addLayout(ui_layer);

    connect(saveButton, SIGNAL(clicked()), this, SLOT(saveFile()));
    connect(loadButton, SIGNAL(clicked()), this, SLOT(loadFile()));
    connect(clearButton, SIGNAL(clicked()), this, SLOT(clearFile()));

    setLayout(h_layer);

    currentImage = new QImage(imageLabel->pixmap()->toImage());

    save_client_ = nh_.serviceClient<std_srvs::Empty>("/vision/save_color_table");
    load_client_ = nh_.serviceClient<std_srvs::Empty>("/vision/load_color_table");
    clear_client_ = nh_.serviceClient<opr_msgs::Color>("/vision/clear_color_table");
    set_color_pub_ = nh_.advertise<opr_msgs::SetColor>("/vision/set_color_table", 1);
    image_sub_ = it_.subscribe("/camera_sensor/image_raw", 1, &ColorCalibration::imageCb, this);
}

void ColorCalibration::saveFile()
{
    //TODO save target filename
    std_srvs::Empty srv;
    save_client_.call(srv);
}

void ColorCalibration::loadFile()
{
    //TODO load target filename
    std_srvs::Empty srv;
    load_client_.call(srv);
}

void ColorCalibration::clearFile()
{
    opr_msgs::Color srv;
    srv.request.color = colorMode->currentIndex();
    clear_client_.call(srv);
}

void ColorCalibration::onWidthChanged()
{
}

void ColorCalibration::onMarginChanged()
{
}

void ColorCalibration::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    QImage::Format format;
    if(msg->encoding == "mono8"){
        format = QImage::Format_Grayscale8;
    }
    else if (msg->encoding == "bgr8" || msg->encoding == "rgb8"){
        format = QImage::Format_RGB888;
    }
    else{
        ROS_ERROR("image pixel format not supported: %s", msg->encoding.c_str());
        return;
    }
    *currentImage = QImage(&msg->data[0], msg->width, msg->height, msg->step, format);
    if(msg->encoding == "bgr8")
        *currentImage = currentImage->rgbSwapped();

    imageLabel->setPixmap(QPixmap::fromImage(*currentImage));
}

void ColorCalibration::mouseMoveEvent(QMouseEvent *event)
{
    int y = event->y();
    int x = event->x();
    int color = colorMode->currentIndex();
    for (int yi = y; yi < (y + width); yi++){
        for (int xi = x; xi < (x + width); xi++){
            if(xi < 0 || currentImage->width() <= xi || yi < 0 || currentImage->height() <= yi)
                return;

            if(event->button() == Qt::RightButton)
                clearmode = true;
            if(event->button() == Qt::LeftButton)
                clearmode = false;

            QRgb pix = currentImage->pixel(xi, yi);
            
            //TODO: setColorA
            // vision->setColorTable(0, qBlue(pix), qGreen(pix), qRed(pix), margin, COLOR, clearmode);
            opr_msgs::SetColor color;
            color.r = qBlue(pix);
            color.g = qGreen(pix);
            color.b = qRed(pix);
            color.margin = margin;
            color.clearmode = clearmode;
            set_color_pub_.publish(color);
            std::cout << qRed(pix) << std::endl;
            std::cout << qGreen(pix) << std::endl;
            std::cout << qBlue(pix) << std::endl;
            std::cout << "===" << std::endl;
        }
    }
}
