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
        // parameters
        this->declare_parameter<std::string>("input_topic", "/ir/image_raw");
        this->declare_parameter<std::string>("output_topic", "/ir_processed/image_raw");
        this->declare_parameter<std::string>("mode_ir", "NFOV");  
        this->declare_parameter<bool>("use_fixed_range", true);

        // Get parameter values
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        this->get_parameter("use_fixed_range", use_fixed_range_);
        mode_ir_ = this->get_parameter("mode_ir").as_string();

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic,
            rclcpp::QoS(10).reliable(),   // au lieu de rclcpp::SensorDataQoS()
            std::bind(&IRImageProcessor::imageCallback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, rclcpp::QoS(10).reliable());

        RCLCPP_INFO(this->get_logger(), "IR Processor node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "mode_ir = %s", mode_ir_.c_str());
        RCLCPP_INFO(this->get_logger(), "use_fixed_range = %s", use_fixed_range_ ? "true" : "false");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    bool use_fixed_range_;
    bool range_initialized_;
    double fixed_min_;
    double fixed_max_;
    std::string mode_ir_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
            cv::Mat image_16 = cv_ptr->image;
            cv::Mat image_filtered;
            cv::medianBlur(image_16, image_filtered, 3);

            double minVal, maxVal;
            cv::Mat image_8bit;  

            // === SWITCH ===
            if (mode_ir_ == "WFOV") {
                if (use_fixed_range_) {
                    if (!range_initialized_) {
                        computePercentileClip(image_filtered, 1.0, 99.0, fixed_min_, fixed_max_);
                        range_initialized_ = true;
                        RCLCPP_INFO(this->get_logger(), "[WFOV] fixed range: [%.1f, %.1f]", fixed_min_, fixed_max_);
                    }
                    minVal = fixed_min_;
                    maxVal = fixed_max_;
                } else {
                    computePercentileClip(image_filtered, 1.0, 99.0, minVal, maxVal);
                }
                if (maxVal - minVal < 10) {
                    minVal = 0;
                    maxVal = 65535;
                }

                image_filtered.convertTo(image_8bit, CV_8U,
                    255.0 / (maxVal - minVal),
                    -minVal * 255.0 / (maxVal - minVal));
            }

            else if (mode_ir_ == "NFOV") {
                computePercentileClip(image_filtered, 1.0, 99, minVal, maxVal);
                if (maxVal - minVal < 10) {
                    minVal = 0;
                    maxVal = 65535;
                }

                cv::Mat image_norm;
                image_filtered.convertTo(image_norm, CV_32F);
                image_norm = cv::max(image_norm, minVal);
                image_norm = cv::min(image_norm, maxVal);
                image_norm = (image_norm - minVal) / (maxVal - minVal);
                cv::log(1 + 1 * image_norm, image_norm);  
                cv::normalize(image_norm, image_norm, 0, 255, cv::NORM_MINMAX);
                image_norm.convertTo(image_8bit, CV_8U);
            }

            else {
                // fallback
                RCLCPP_WARN_ONCE(this->get_logger(),
                    "Unknown mode_ir '%s', using default [0,65535]", mode_ir_.c_str());
                minVal = 0;
                maxVal = 65535;
                image_filtered.convertTo(image_8bit, CV_8U,
                    255.0 / (maxVal - minVal),
                    -minVal * 255.0 / (maxVal - minVal));
            }

            cv::Mat image_final;
            cv::bilateralFilter(image_8bit, image_final, 5, 75, 75);

            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header;
            out_msg.encoding = "mono8";
            out_msg.image = image_final;
            pub_->publish(*out_msg.toImageMsg());
        }
        catch (cv_bridge::Exception& e) {
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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}