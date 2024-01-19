#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp" 

class LaneDetectionNode : public rclcpp::Node {
public:
    LaneDetectionNode() : Node("lane_detection_node") {
        if (load_image_from_path_) {
            image_ = cv::imread(image_path_);
            processImage();
        } else {
            image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image", 10, std::bind(&LaneDetectionNode::imageCallback, this, std::placeholders::_1));
        }

        detected_lane_data_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/detected_lane_data", 10);
        
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_ = cv_ptr_->image;
            processImage();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void processImage() {
        cv::cvtColor(image_, image_, cv::COLOR_BGR2RGB);

        int height = image_.rows;
        int width = image_.cols;

        std::vector<cv::Point> region_of_interest_vertices = {
            {0, height},
            {width/2, height/2},
            {width, height}
        };

        cv::Mat gray_image, canny_image;
        cv::cvtColor(image_, gray_image, cv::COLOR_RGB2GRAY);
        cv::Canny(gray_image, canny_image, 100, 200);

        std::vector<std::vector<cv::Point>> roi_vertices = { region_of_interest_vertices };
        region_of_interest(canny_image, roi_vertices);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(canny_image, lines, 6, CV_PI/180, 160, 40, 25);

        for (const cv::Vec4i& line : lines) {
            std::cout << line << std::endl;
        }

        draw_the_lines(image_, lines);

        cv_bridge::CvImage cv_image_msg(std_msgs::msg::Header(), "rgb8", image_);
        sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_image_msg.toImageMsg();
        detected_lane_data_publisher_->publish(*processed_image_msg);
    }

    void region_of_interest(cv::Mat& img, const std::vector<std::vector<cv::Point>>& vertices) {
        cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
        cv::fillPoly(mask, vertices, cv::Scalar(255));
        cv::bitwise_and(img, mask, img);
    }

    void draw_the_lines(cv::Mat& img, const std::vector<cv::Vec4i>& lines) {
        cv::Mat blank_image = cv::Mat::zeros(img.size(), CV_8UC3);

        for (const cv::Vec4i& line : lines) {
            cv::line(blank_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 10);
        }

        cv::addWeighted(img, 0.8, blank_image, 1, 0.0, img);
    }

    bool load_image_from_path_ = false;  
    std::string image_path_ = "src/test.png";
    cv::Mat image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_lane_data_publisher_;
    cv_bridge::CvImagePtr cv_ptr_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
