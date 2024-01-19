#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

class LaneDetectionTestNode : public rclcpp::Node
{
public:

    LaneDetectionTestNode() : Node("lane_detection_test_node")
    {
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image", 10);

        video_capture_.open("src/test.mp4"); 

        if (!video_capture_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source.");
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            publishFrame();
        });
    }


private:
  void publishFrame()
  {
    cv::Mat frame;
    video_capture_ >> frame;

    if (!frame.empty())
    {
      auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
      img_msg->header.stamp = this->now();
      img_msg->header.frame_id = "camera_frame"; 
      img_msg->height = frame.rows;
      img_msg->width = frame.cols;
      img_msg->encoding = "bgr8";
      img_msg->is_bigendian = false;
      img_msg->step = frame.step;
      size_t size = frame.rows * frame.cols * frame.elemSize();
      img_msg->data.resize(size);
      memcpy(&img_msg->data[0], frame.data, size);

      image_publisher_->publish(*img_msg);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture video_capture_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetectionTestNode>());
  rclcpp::shutdown();

  return 0;
}
