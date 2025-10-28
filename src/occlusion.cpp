#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <getopt.h>
#include <string>

class OcclusionNode : public rclcpp::Node
{
public:
    OcclusionNode(const std::string& image_type, int rate, const std::string& mode)
        : Node("occlusion_node"), image_type_(image_type), occlusion_rate_(rate), 
          occlusion_mode_(mode)
    {
        RCLCPP_INFO(this->get_logger(), 
                   "Occlusion node started - Type: %s, Rate: %d%%, Mode: %s", 
                   image_type_.c_str(), occlusion_rate_, occlusion_mode_.c_str());
        
        generateOcclusionPattern();
    }
    
    void initialize()
    {
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        setupAllStreams();
    }

private:
    void generateOcclusionPattern()
    {
        occlusion_mask_generated_ = false;
    }
    
    void ensureOcclusionMask(int width, int height, bool use_circle)
    {
        // Return early if mask already generated with correct dimensions
        if (occlusion_mask_generated_ && 
            occlusion_mask_.cols == width && 
            occlusion_mask_.rows == height) {
            return; 
        }
        
        // Initialize mask to all white (no occlusion)
        occlusion_mask_ = cv::Mat::ones(height, width, CV_8UC1) * 255;
        
        // No occlusion needed
        if (occlusion_rate_ == 0) {
            occlusion_mask_generated_ = true;
            return; 
        }
        
        // Full occlusion
        if (occlusion_rate_ >= 100) {
            occlusion_mask_.setTo(cv::Scalar(0));
            occlusion_mask_generated_ = true;
            return;
        }
        
        // Apply occlusion pattern based on mode
        if (occlusion_mode_ == "left") {
            int occlusion_width = (width * occlusion_rate_) / 100;
            cv::Rect occlusion_rect(0, 0, occlusion_width, height);
            occlusion_mask_(occlusion_rect).setTo(cv::Scalar(0));
            
        } else if (occlusion_mode_ == "right") {
            int occlusion_width = (width * occlusion_rate_) / 100;
            cv::Rect occlusion_rect(width - occlusion_width, 0, occlusion_width, height);
            occlusion_mask_(occlusion_rect).setTo(cv::Scalar(0));
            
        } else if (occlusion_mode_ == "center") {
            if (use_circle) {
                // Circular occlusion for IR sensor
                cv::Point2f center(width / 2.0f, height / 2.0f);
                float max_radius = std::min(width, height) / 2.0f;
                float radius = (max_radius * occlusion_rate_) / 100.0f;
                cv::circle(occlusion_mask_, center, static_cast<int>(radius), cv::Scalar(0), -1);
                
            } else {
                // Rectangular occlusion for RGB camera
                int occlusion_width = (width * occlusion_rate_) / 100;
                int occlusion_height = (height * occlusion_rate_) / 100;
                int start_x = (width - occlusion_width) / 2;
                int start_y = (height - occlusion_height) / 2;
                cv::Rect occlusion_rect(start_x, start_y, occlusion_width, occlusion_height);
                occlusion_mask_(occlusion_rect).setTo(cv::Scalar(0));
            }
        }
        
        occlusion_mask_generated_ = true;
    }
    
    void setupAllStreams()
    {
        // Subscribe to all camera streams
        rgb_subscriber_ = image_transport_->subscribe(
            "/rgb/image_raw", 1,
            std::bind(&OcclusionNode::rgbImageCallback, this, std::placeholders::_1));
        
        depth_rgb_subscriber_ = image_transport_->subscribe(
            "/depth_to_rgb/image_raw", 1,
            std::bind(&OcclusionNode::depthRgbImageCallback, this, std::placeholders::_1));
        
        ir_subscriber_ = image_transport_->subscribe(
            "/ir_processed/image_raw", 1,
            std::bind(&OcclusionNode::irImageCallback, this, std::placeholders::_1));
        
        depth_subscriber_ = image_transport_->subscribe(
            "/depth/image_raw", 1,
            std::bind(&OcclusionNode::depthImageCallback, this, std::placeholders::_1));
        
        // Create publishers for modified streams
        rgb_publisher_ = image_transport_->advertise("/camera/color/image_raw", 1);
        depth_rgb_publisher_ = image_transport_->advertise("/camera/depth/image_raw", 1);
        ir_publisher_ = image_transport_->advertise("/camera/ir/image_raw", 1);
        depth_publisher_ = image_transport_->advertise("/camera/depth_ir/image_raw", 1);
    }
    
    void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            if (image_type_ == "rgb") {
                applyOcclusion(cv_ptr->image, false, false);
                RCLCPP_DEBUG(this->get_logger(), "Applied RGB occlusion");
            }
            
            rgb_publisher_.publish(cv_ptr->toImageMsg());
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception for RGB: %s", e.what());
        }
    }
    
    void depthRgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            if (image_type_ == "ir") {
                applyOcclusion(cv_ptr->image, true, true);
                RCLCPP_DEBUG(this->get_logger(), "Applied IR sensor failure to depth_rgb");
            }
            
            depth_rgb_publisher_.publish(cv_ptr->toImageMsg());
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception for depth RGB: %s", e.what());
        }
    }
    
    void irImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            
            if (image_type_ == "ir") {
                applyOcclusion(cv_ptr->image, true, false);
                RCLCPP_DEBUG(this->get_logger(), "Applied IR occlusion");
            }
            
            ir_publisher_.publish(cv_ptr->toImageMsg());
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception for IR: %s", e.what());
        }
    }
    
    void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            if (image_type_ == "ir") {
                applyOcclusion(cv_ptr->image, true, true);
                RCLCPP_DEBUG(this->get_logger(), "Applied IR depth occlusion");
            }
            
            depth_publisher_.publish(cv_ptr->toImageMsg());
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception for depth: %s", e.what());
        }
    }
    
    void applyOcclusion(cv::Mat& image, bool use_circle, bool is_depth)
    {
        int height = image.rows;
        int width = image.cols;
        
        // No occlusion needed
        if (occlusion_rate_ == 0) {
            return; 
        }
        
        // Handle dimming mode separately
        if (occlusion_mode_ == "dimming") {
            applyDimming(image, is_depth);
            return;
        }
        
        // Generate occlusion mask for this image size
        ensureOcclusionMask(width, height, use_circle);
        
        // Blackout: set occluded pixels to 0 (black)
        cv::Scalar occlusion_value = cv::Scalar(0);
        
        // Handle noise mode
        if (occlusion_mode_ == "noise") {
            if (is_depth) {
                return;
            }
            
            cv::Mat noise = cv::Mat(height, width, image.type());
            cv::randn(noise, 0, 50);
            
            cv::Mat mask_inv;
            cv::bitwise_not(occlusion_mask_, mask_inv);
            
            if (image.channels() == 1) {
                cv::add(image, noise, image, mask_inv);
            } else {
                std::vector<cv::Mat> channels;
                cv::split(image, channels);
                std::vector<cv::Mat> noise_channels;
                cv::split(noise, noise_channels);
                
                for (size_t i = 0; i < channels.size(); i++) {
                    cv::add(channels[i], noise_channels[i], channels[i], mask_inv);
                }
                cv::merge(channels, image);
            }
            return;
        }
        
        // Handle blur mode
        if (occlusion_mode_ == "blur") {
            if (is_depth) {
                return;
            }
            
            cv::Mat blurred;
            int kernel_size = 15 + (occlusion_rate_ / 5);
            if (kernel_size % 2 == 0) kernel_size++;
            cv::GaussianBlur(image, blurred, cv::Size(kernel_size, kernel_size), 0);
            
            cv::Mat mask_inv;
            cv::bitwise_not(occlusion_mask_, mask_inv);
            
            if (image.channels() == 1) {
                blurred.copyTo(image, mask_inv);
            } else {
                std::vector<cv::Mat> channels;
                cv::split(image, channels);
                std::vector<cv::Mat> blurred_channels;
                cv::split(blurred, blurred_channels);
                
                for (size_t i = 0; i < channels.size(); i++) {
                    blurred_channels[i].copyTo(channels[i], mask_inv);
                }
                cv::merge(channels, image);
            }
            return;
        }
        
        // Apply blackout using the mask
        cv::Mat mask_inv;
        cv::bitwise_not(occlusion_mask_, mask_inv);
        
        if (image.channels() == 1) {
            image.setTo(occlusion_value, mask_inv);
        } else {
            std::vector<cv::Mat> channels;
            cv::split(image, channels);
            for (auto& channel : channels) {
                channel.setTo(occlusion_value, mask_inv);
            }
            cv::merge(channels, image);
        }
    }
    
    void applyDimming(cv::Mat& image, bool is_depth)
    {
        // Dimming doesn't apply to depth images
        if (is_depth) {
            return;
        }
        
        // Calculate dimming factor: 0% = no change, 100% = black
        double dimming_factor = 1.0 - (static_cast<double>(occlusion_rate_) / 100.0);
        
        cv::Mat dimmed_image;
        image.convertTo(dimmed_image, -1, dimming_factor, 0);
        
        dimmed_image.copyTo(image);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Applied dimming: rate=%d%%, factor=%.2f", 
                    occlusion_rate_, dimming_factor);
    }
    
    // Member variables
    std::string image_type_;
    int occlusion_rate_;
    std::string occlusion_mode_;
    
    cv::Mat occlusion_mask_;
    bool occlusion_mask_generated_ = false;
    
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    
    // Subscribers
    image_transport::Subscriber ir_subscriber_;
    image_transport::Subscriber depth_subscriber_;
    image_transport::Subscriber rgb_subscriber_;
    image_transport::Subscriber depth_rgb_subscriber_;
    
    // Publishers
    image_transport::Publisher ir_publisher_;
    image_transport::Publisher depth_publisher_;
    image_transport::Publisher rgb_publisher_;
    image_transport::Publisher depth_rgb_publisher_;
};

void printUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " [OPTIONS]\n\n";
    std::cout << "ROS2 node for simulating camera failures in RGBIRD SLAM\n\n";
    std::cout << "Options:\n";
    std::cout << "  -i, --image TYPE     Camera type to simulate failure: 'ir' or 'rgb'\n";
    std::cout << "                       'rgb': affects ONLY RGB stream\n";
    std::cout << "                       'ir': affects IR + depth_ir + depth_rgb (IR sensor failure)\n";
    std::cout << "  -r, --rate PERCENT   Failure intensity (0-100)\n";
    std::cout << "  -m, --mode MODE      Failure pattern:\n";
    std::cout << "                       'left': occlusion from left edge\n";
    std::cout << "                       'right': occlusion from right edge\n";
    std::cout << "                       'center': center occlusion (rect for RGB, circle for IR)\n";
    std::cout << "                       'noise': gaussian noise (images only)\n";
    std::cout << "                       'blur': motion blur (images only)\n";
    std::cout << "                       'dimming': reduce brightness (RGB or IR)\n";
    std::cout << "  -h, --help           Show this help\n\n";
    std::cout << "Examples:\n";
    std::cout << "  # RGB camera blackout (only RGB affected)\n";
    std::cout << "  " << program_name << " -i rgb -r 50 -m center\n\n";
    std::cout << "  # IR sensor blackout (IR + depth_ir + depth_rgb all affected with SAME pattern)\n";
    std::cout << "  " << program_name << " -i ir -r 70 -m center\n\n";
    std::cout << "  # RGB dimming (reduce brightness by 30%)\n";
    std::cout << "  " << program_name << " -i rgb -r 30 -m dimming\n\n";
    std::cout << "Logic:\n";
    std::cout << "  RGB failure: Only RGB camera fails with blackout (pixels set to 0)\n";
    std::cout << "  IR failure: IR sensor fails → IR, depth_ir, AND depth_rgb all show IDENTICAL blackout\n";
    std::cout << "  Dimming mode: Reduces brightness by rate% (0% = no change, 100% = black)\n";
    std::cout << "  This simulates realistic sensor failure scenarios\n";
}

int main(int argc, char** argv)
{
    std::string image_type = "";
    int rate = 0;
    std::string mode = "";
    
    static struct option long_options[] = {
        {"image", required_argument, 0, 'i'},
        {"rate", required_argument, 0, 'r'},
        {"mode", required_argument, 0, 'm'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };
    
    int option_index = 0;
    int c;
    
    while ((c = getopt_long(argc, argv, "i:r:m:h", long_options, &option_index)) != -1) {
        switch (c) {
            case 'i':
                image_type = optarg;
                break;
            case 'r':
                try {
                    rate = std::stoi(optarg);
                    if (rate < 0 || rate > 100) {
                        std::cerr << "Error: Rate must be between 0 and 100\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid rate value\n";
                    return 1;
                }
                break;
            case 'm':
                mode = optarg;
                break;
            case 'h':
                printUsage(argv[0]);
                return 0;
            case '?':
                printUsage(argv[0]);
                return 1;
            default:
                break;
        }
    }
    
    // Validate required arguments
    if (image_type.empty() || mode.empty()) {
        std::cerr << "Error: Image type (-i) and mode (-m) are required\n";
        printUsage(argv[0]);
        return 1;
    }
    
    if (image_type != "ir" && image_type != "rgb") {
        std::cerr << "Error: Image type must be 'ir' or 'rgb'\n";
        return 1;
    }
    
    std::vector<std::string> valid_modes = {"left", "right", "center", "noise", "blur", "dimming"};
    if (std::find(valid_modes.begin(), valid_modes.end(), mode) == valid_modes.end()) {
        std::cerr << "Error: Invalid mode. Valid: left, right, center, noise, blur, dimming\n";
        return 1;
    }
    
    if (mode == "dimming" && image_type != "rgb" && image_type != "ir") {
        std::cerr << "Error: Dimming mode is only available for RGB or IR images (-i rgb or -i ir)\n";
        return 1;
    }
    
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<OcclusionNode>(image_type, rate, mode);
        node->initialize();
        
        std::cout << "\n=== RGBIRD FAILURE SIMULATION ACTIVE ===\n";
        std::cout << "Camera: " << image_type << "\n";
        std::cout << "Intensity: " << rate << "%\n";
        std::cout << "Pattern: " << mode << "\n";
        std::cout << "Affected streams: ";
        if (image_type == "rgb") {
            if (mode == "dimming") {
                std::cout << "RGB brightness reduced by " << rate << "%\n";
            } else {
                std::cout << "RGB only (blackout)\n";
            }
        } else {
            if (mode == "dimming") {
                std::cout << "IR brightness reduced by " << rate << "% (depth streams unaffected)\n";
            } else {
                std::cout << "IR + depth_ir + depth_rgb (IDENTICAL blackout pattern)\n";
            }
        }
        std::cout << "==========================================\n\n";
        
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
