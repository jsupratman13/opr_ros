#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#endif

#include <QWidget>
#include <QComboBox>
#include <QLabel>

class ColorCalibration : public QWidget
{
    Q_OBJECT
    public:
        ColorCalibration(QWidget *parent=0);

    private Q_SLOTS:
        void saveFile();
        void loadFile();
        void clearFile();
        void onWidthChanged();
        void onMarginChanged();

    private:
        ros::NodeHandle nh_;
        ros::ServiceClient save_client_;
        ros::ServiceClient load_client_;
        ros::ServiceClient clear_client_;
        ros::Publisher set_color_pub_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;

        QImage *currentImage;
        QComboBox *colorMode;
        QLabel *imageLabel;

        int width = 2;
        int margin = 2;
        bool clearmode = false;

        virtual void mouseMoveEvent(QMouseEvent *event);
        void imageCb(const sensor_msgs::ImageConstPtr &msg);
};
