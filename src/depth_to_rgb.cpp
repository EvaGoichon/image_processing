#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using std::placeholders::_1;

class DepthToRGBNode : public rclcpp::Node
{
public:
    DepthToRGBNode() : Node("depth_to_rgb")
    {
        this->declare_parameter<double>("depth_scale", 0.001);
        this->get_parameter("depth_scale", depth_scale_);

        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&DepthToRGBNode::depthCallback, this, _1));

        pub_aligned_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/depth_to_rgb/image_raw", 10);

        initIntrinsics();
        initExtrinsics();

        RCLCPP_INFO(this->get_logger(), "Node depth_to_rgb initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_aligned_;
    double depth_scale_;

    cv::Mat K_d_, D_d_, K_rgb_, D_rgb_;
    Eigen::Matrix4d T_;

    void initIntrinsics()
    {
        // ==== Depth camera (512x512) ====
        K_d_ = (cv::Mat_<double>(3,3) <<
            252.18, 0.0, 258.233,
            0.0, 252.275, 256.963,
            0.0, 0.0, 1.0);

        // (k1,k2,k3,p1,p2)
        D_d_ = (cv::Mat_<double>(1,5) <<
            -0.304917, 0.09239, -0.0123752, 8.73484e-05, -1.34749e-05);

        // ==== RGB camera (2048x1536) ====
        K_rgb_ = (cv::Mat_<double>(3,3) <<
            968.381, 0.0, 1025.42,
            0.0, 968.368, 780.155,
            0.0, 0.0, 1.0);

        D_rgb_ = (cv::Mat_<double>(1,5) <<
            0.0736269, 0.437159, -3.23322, -0.000323023, 1.75379);
    }

    void initExtrinsics()
    {
        Eigen::Quaterniond q(0.9988010863820517,
                             0.0488315560101656,
                             0.0003209717389778205,
                             0.0034281581399996505);
        Eigen::Vector3d t(0.03215164832062954,
                          0.0026993268087001157,
                          -0.0039073853888447925);

        Eigen::Matrix4d T_original = Eigen::Matrix4d::Identity();
        T_original.block<3,3>(0,0) = q.toRotationMatrix();
        T_original.block<3,1>(0,3) = t;
        T_ = T_original.inverse();  // Depth → RGB
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        cv::Mat depth_raw = cv_ptr->image;
        cv::Mat depth_undist, map1, map2;
        cv::initUndistortRectifyMap(K_d_, D_d_, cv::noArray(), K_d_,
                                    depth_raw.size(), CV_32FC1, map1, map2);
        cv::remap(depth_raw, depth_undist, map1, map2, cv::INTER_NEAREST);

        // === Output same size as RGB ===
        int rgb_cols = 2048;
        int rgb_rows = 1536;
        cv::Mat aligned = cv::Mat::zeros(cv::Size(rgb_cols, rgb_rows), CV_16UC1);

        for (int v = 0; v < depth_undist.rows; v++) {
            for (int u = 0; u < depth_undist.cols; u++) {
                float z = depth_undist.at<uint16_t>(v,u) * depth_scale_;
                if (z <= 0.0) continue;

                double x = (u - K_d_.at<double>(0,2)) * z / K_d_.at<double>(0,0);
                double y = (v - K_d_.at<double>(1,2)) * z / K_d_.at<double>(1,1);

                Eigen::Vector4d pt_d(x, y, z, 1.0);
                Eigen::Vector4d pt_rgb = T_ * pt_d;

                double xn = pt_rgb(0) / pt_rgb(2);
                double yn = pt_rgb(1) / pt_rgb(2);

                // Distorsion RGB (radiale + tangentielle)
                double k1 = D_rgb_.at<double>(0,0);
                double k2 = D_rgb_.at<double>(0,1);
                double k3 = D_rgb_.at<double>(0,2);
                double p1 = D_rgb_.at<double>(0,3);
                double p2 = D_rgb_.at<double>(0,4);

                double r2 = xn*xn + yn*yn;
                double r4 = r2*r2;
                double r6 = r4*r2;
                double radial = 1 + k1*r2 + k2*r4 + k3*r6;

                double xd = xn*radial + 2*p1*xn*yn + p2*(r2 + 2*xn*xn);
                double yd = yn*radial + p1*(r2 + 2*yn*yn) + 2*p2*xn*yn;

                int u_rgb = static_cast<int>(K_rgb_.at<double>(0,0)*xd + K_rgb_.at<double>(0,2));
                int v_rgb = static_cast<int>(K_rgb_.at<double>(1,1)*yd + K_rgb_.at<double>(1,2));

                if (u_rgb >= 0 && u_rgb < rgb_cols && v_rgb >= 0 && v_rgb < rgb_rows)
                    aligned.at<uint16_t>(v_rgb, u_rgb) = static_cast<uint16_t>(z / depth_scale_);
            }
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        out_msg.image = aligned;
        pub_aligned_->publish(*out_msg.toImageMsg());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToRGBNode>());
    rclcpp::shutdown();
    return 0;
}

