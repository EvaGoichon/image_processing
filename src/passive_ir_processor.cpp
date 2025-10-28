#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class PassiveIRProcessor : public rclcpp::Node
{
public:
    PassiveIRProcessor() : Node("passive_ir_processor")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("input_topic", "/ir/image_raw");
        this->declare_parameter<std::string>("output_topic", "/camera/image_raw");
        
        // Get the parameter values
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        // Subscribe to the 16-bit IR image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 
            10, 
            std::bind(&PassiveIRProcessor::imageCallback, this, std::placeholders::_1)
        );

        // Publish the processed 8-bit image topic
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Passive IR Processor node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
    }

private:
    // Callback function to process the incoming image
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image message to OpenCV Mat (16-bit grayscale)
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image_16bit = cv_ptr->image;

        // Apply image processing steps
        cv::Mat outputImg, outputImg8UC1;
        double min_val, max_val;
        double gamma = 0.6;

        // Initialize output image with float type
        outputImg = cv::Mat::zeros(image_16bit.rows, image_16bit.cols, CV_32FC1);

        // Get min and max values from the image
        cv::minMaxIdx(image_16bit, &min_val, &max_val);

        // Apply gamma correction and normalize the image
        for (int i = 0; i < image_16bit.rows; i++)
        {
            for (int j = 0; j < image_16bit.cols; j++)
            {
                outputImg.at<float>(cv::Point2d(i, j)) = std::min<float>(
                    std::pow(
                        static_cast<float>(image_16bit.at<uint16_t>(cv::Point2d(i, j))) / 
                        static_cast<float>(static_cast<uint16_t>(max_val) >> 3), 
                        gamma
                    ), 
                    1.0f
                );
            }
        }

        // Convert the float image to 8-bit for display and publishing
        outputImg.convertTo(outputImg8UC1, CV_8UC1, 255.0);

        // Convert processed 8-bit image to ROS message and publish
        sensor_msgs::msg::Image::SharedPtr output_msg = 
            cv_bridge::CvImage(msg->header, "mono8", outputImg8UC1).toImageMsg();
        
        publisher_->publish(*output_msg);

        RCLCPP_DEBUG(this->get_logger(), "Processed and published IR image");
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PassiveIRProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
