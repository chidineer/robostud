#include "colour_detector.hpp"

ColourDetector::ColourDetector(){

}

ColourDetector::ProductColour ColourDetector::detectBox(const sensor_msgs::msg::Image::SharedPtr msg) {

    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_image_ptr;
    try
    {
        cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        return ColourDetector::NOTHING;; // Return a default value or handle this case appropriately
    }

    // Convert the image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_image_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Define color ranges and create masks
    cv::Scalar red_lower(0, 100, 0), red_upper(10, 255, 255);
    cv::Scalar blue_lower(100, 150, 0), blue_upper(140, 255, 255);
    cv::Scalar yellow_lower(20, 100, 100), yellow_upper(30, 255, 255);
    cv::Scalar green_lower(40, 40, 40), green_upper(70, 255, 255);
    cv::Scalar orange_lower(5, 150, 150), orange_upper(15, 255, 255);
    cv::Scalar purple_lower(130, 50, 50), purple_upper(160, 255, 255);

    // <color>1.0 0.0 0.0 1.0</color> <!-- Red color -->
    // <color>1.0 1.0 0.0 1.0</color> <!-- Yellow color -->
    // <color>0.0 0.0 1.0 1.0</color> <!-- Blue color -->
    // <color>0.0 1.0 0.0 1.0</color> <!-- Green color -->
    // <color>1.0 0.5 0.0 1.0</color> <!-- Orange color -->
    // <color>0.5 0.0 0.5 1.0</color> <!-- Purple color -->

    // Create masks for each color
    cv::Mat red_mask, blue_mask, yellow_mask, green_mask, orange_mask, purple_mask;
    cv::inRange(hsv_image, red_lower, red_upper, red_mask);
    cv::inRange(hsv_image, blue_lower, blue_upper, blue_mask);
    cv::inRange(hsv_image, yellow_lower, yellow_upper, yellow_mask);
    cv::inRange(hsv_image, green_lower, green_upper, green_mask);
    cv::inRange(hsv_image, orange_lower, orange_upper, orange_mask);
    cv::inRange(hsv_image, purple_lower, purple_upper, purple_mask);

    // Calculate the area for each color mask
    int red_area = cv::countNonZero(red_mask);
    int blue_area = cv::countNonZero(blue_mask);
    int yellow_area = cv::countNonZero(yellow_mask);
    int green_area = cv::countNonZero(green_mask);
    int orange_area = cv::countNonZero(orange_mask);
    int purple_area = cv::countNonZero(purple_mask);

    // Create an array of areas for easier comparison
    int color_areas[] = { red_area, yellow_area, blue_area, green_area, orange_area, purple_area };

    // Find the index of the largest area
    int max_index = std::distance(color_areas, std::max_element(color_areas, color_areas + 6));

    // Return the corresponding enumerator value
    switch (max_index) {
        case 0: return ColourDetector::RED;
        case 1: return ColourDetector::YELLOW;
        case 2: return ColourDetector::BLUE;
        case 3: return ColourDetector::GREEN;
        case 4: return ColourDetector::ORANGE;
        case 5: return ColourDetector::PURPLE;
        default: return ColourDetector::NOTHING;  // Default in case of an error
    }
}

