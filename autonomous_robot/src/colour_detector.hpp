#ifndef COLOUR_DETECTOR_HPP
#define COLOUR_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ColourDetector
{
public:
    ColourDetector();

    enum ProductColour {
        RED,
        YELLOW,
        BLUE,
        GREEN,
        ORANGE,
        PURPLE,
        NOTHING
    };

    ProductColour detectBox(const sensor_msgs::msg::Image::SharedPtr msg);
    

private:

    

};

#endif // COLOUR_DETECTOR_HPP
