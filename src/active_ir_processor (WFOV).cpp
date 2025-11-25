#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class IRImageProcessor : public rclcpp::Node
{
public:
    IRImageProcessor()
    : Node("ir_image_processor"),
      use_fixed_range_(true),
      range_initialized_(false)
    {
        // Declare parameters for topics
        this->declare_parameter<std::string>("input_topic", "/ir/image_raw");
        this->declare_parameter<std::string>("output_topic", "/ir_processed/image_raw");
        this->declare_parameter<bool>("use_fixed_range", true);
        
        // Get parameter values
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        this->get_parameter("use_fixed_range", use_fixed_range_);

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&IRImageProcessor::imageCallback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Active IR Processor node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "use_fixed_range = %s", use_fixed_range_ ? "true" : "false");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    bool use_fixed_range_;
    bool range_initialized_;
    double fixed_min_;
    double fixed_max_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            rclcpp::Time t0 = this->now();

            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
            cv::Mat image_16 = cv_ptr->image;

            cv::Mat image_filtered;
            cv::medianBlur(image_16, image_filtered, 3);

            double minVal, maxVal;
            if (use_fixed_range_)
            {
                if (!range_initialized_)
                {
                    computePercentileClip(image_filtered, 1.0, 99.0, fixed_min_, fixed_max_);
                    range_initialized_ = true;
                    RCLCPP_DEBUG(this->get_logger(), "Fixed range initialized: [%.2f, %.2f]", fixed_min_, fixed_max_);
                }
                minVal = fixed_min_;
                maxVal = fixed_max_;
            }
            else
            {
                computePercentileClip(image_filtered, 1.0, 99.0, minVal, maxVal);
            }

            if (maxVal - minVal < 10)
            {
                minVal = 0;
                maxVal = 65535;
            }

            cv::Mat image_8bit;
            image_filtered.convertTo(image_8bit, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

            cv::Mat image_equalized = image_8bit;
            cv::Mat image_filtered_final;
            cv::bilateralFilter(image_equalized, image_filtered_final, 5, 75, 75);

            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header;
            out_msg.encoding = "mono8";
            out_msg.image = image_filtered_final;
            pub_->publish(*out_msg.toImageMsg());

            rclcpp::Duration duration = this->now() - t0;
            RCLCPP_DEBUG(this->get_logger(), "Processing time: %.4fs", duration.seconds());
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void computePercentileClip(const cv::Mat& input, double p_low, double p_high, double& out_low, double& out_high)
    {
        CV_Assert(input.type() == CV_16U);
        std::vector<uint16_t> pixels(input.begin<uint16_t>(), input.end<uint16_t>());
        size_t total = pixels.size();

        size_t low_idx = static_cast<size_t>(p_low / 100.0 * total);
        size_t high_idx = static_cast<size_t>(p_high / 100.0 * total);

        std::nth_element(pixels.begin(), pixels.begin() + low_idx, pixels.end());
        out_low = pixels[low_idx];

        std::nth_element(pixels.begin(), pixels.begin() + high_idx, pixels.end());
        out_high = pixels[high_idx];
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IRImageProcessor>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
