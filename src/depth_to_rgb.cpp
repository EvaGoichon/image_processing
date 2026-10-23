// ===========================================================================
// depth_to_rgb.cpp  —  ROS2 node
//
// Reprojects depth images (512x512, IR camera) into the geometry
// of the RGB camera (2048x1536), for RGBD/RID-SLAM overlay.
//
// ROS2 parameters:
//   calibration_file  (string)  — path to RID_MuSeM.yaml (or any yaml
//                                 with Camera / CameraIR / Transformation sections)
//   depth_topic       (string)  — default: "/depth/image_raw"
//   output_topic      (string)  — default: "/depth_to_rgb/image_raw"
//
// Expected YAML format (same as RID_MuSeM.yaml):
//   Camera:    fx, fy, cx, cy, k1, k2, k3, p1, p2, cols, rows
//   CameraIR:  fx, fy, cx, cy, k1, k2, k3, p1, p2, cols, rows
//   Transformation:
//     translation:         [tx, ty, tz]          (in meters)
//     rotation_quaternion: [w, x, y, z]
//   Preprocessing:
//     depthmap_factor: 1000.0
//
// OpenCV distortion order used internally: k1 k2 p1 p2 k3
// (the YAML stores k1 k2 k3 p1 p2 — the conversion is done when reading)
// ===========================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>

using std::placeholders::_1;

// ---------------------------------------------------------------------------
// YAML helpers
// ---------------------------------------------------------------------------

static double yamlDouble(const YAML::Node& node, const std::string& key)
{
    if (!node[key])
        throw std::runtime_error("YAML: missing key: " + key);
    return node[key].as<double>();
}

static int yamlInt(const YAML::Node& node, const std::string& key)
{
    if (!node[key])
        throw std::runtime_error("YAML: missing key: " + key);
    return node[key].as<int>();
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

class DepthToRGBNode : public rclcpp::Node
{
public:
    DepthToRGBNode() : Node("depth_to_rgb")
    {
        // ---- ROS2 parameters ------------------------------------------------
        this->declare_parameter<std::string>("calibration_file", "");
        this->declare_parameter<std::string>("depth_topic",  "/depth/image_raw");
        this->declare_parameter<std::string>("output_topic", "/depth_to_rgb/image_raw");

        const std::string calib_path   = this->get_parameter("calibration_file").as_string();
        const std::string depth_topic  = this->get_parameter("depth_topic").as_string();
        const std::string output_topic = this->get_parameter("output_topic").as_string();

        if (calib_path.empty())
        {
            RCLCPP_FATAL(this->get_logger(),
                "Parameter 'calibration_file' is not set.\n"
                "Usage: ros2 run <pkg> depth_to_rgb "
                "--ros-args -p calibration_file:=/home/eva/ros2_ws/src/stella_vslam_ros_RID/yaml/RID_WFOV_u.yaml");
            throw std::runtime_error("missing calibration_file");
        }

        // ---- YAML loading ---------------------------------------------------
        loadCalibration(calib_path);

        // The LUT is built on the first message (to read the actual dimensions)

        // ---- Topics ---------------------------------------------------------
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, rclcpp::SensorDataQoS(),
            std::bind(&DepthToRGBNode::depthCallback, this, _1));

        pub_aligned_ = this->create_publisher<sensor_msgs::msg::Image>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(),
            "depth_to_rgb ready.\n"
            "  calib  : %s\n"
            "  sub    : %s\n"
            "  pub    : %s\n"
            "  depth  : %dx%d  →  rgb : %dx%d\n"
            "  depth_scale : %.6f",
            calib_path.c_str(),
            depth_topic.c_str(), output_topic.c_str(),
            depth_cols_, depth_rows_, rgb_cols_, rgb_rows_,
            depth_scale_);
    }

private:
    // ---- ROS2 ---------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_aligned_;

    // ---- Intrinsics ---------------------------------------------------------
    cv::Mat K_d_, D_d_;          // Depth/IR camera  (OpenCV: k1 k2 p1 p2 k3)
    cv::Mat K_rgb_, D_rgb_;      // RGB camera       (OpenCV: k1 k2 p1 p2 k3)
    int depth_cols_{0}, depth_rows_{0};
    int rgb_cols_{0},   rgb_rows_{0};
    double depth_scale_{0.001};  // = 1 / depthmap_factor

    // ---- Extrinsics ---------------------------------------------------------
    Eigen::Matrix4d T_depth_to_rgb_;   // Depth → RGB

    // ---- Precomputed LUT ----------------------------------------------------
    // Built on the first message to use the actual dimensions.
    cv::Mat lut_u_rgb_;   // CV_32SC1, depth_rows x depth_cols
    cv::Mat lut_v_rgb_;   // CV_32SC1
    bool lut_ready_{false};

    // ---- Depth undistortion maps (precomputed) ------------------------------
    cv::Mat undist_map1_, undist_map2_;

    // =========================================================================
    // loadCalibration  —  reads a YAML in the RID_MuSeM.yaml format
    // =========================================================================
    void loadCalibration(const std::string& path)
    {
        YAML::Node cfg;
        try {
            cfg = YAML::LoadFile(path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Cannot read YAML '" + path + "': " + e.what());
        }

        // ---- Depth / IR camera (CameraIR) -----------------------------------
        if (!cfg["CameraIR"])
            throw std::runtime_error("Section 'CameraIR' missing from YAML '" + path + "'.");
        {
            const auto c = cfg["CameraIR"];
            K_d_ = (cv::Mat_<double>(3,3) <<
                yamlDouble(c,"fx"), 0.0,               yamlDouble(c,"cx"),
                0.0,               yamlDouble(c,"fy"), yamlDouble(c,"cy"),
                0.0,               0.0,                1.0);

            // YAML : k1 k2 k3 p1 p2   →   OpenCV : k1 k2 p1 p2 k3
            D_d_ = (cv::Mat_<double>(1,5) <<
                yamlDouble(c,"k1"), yamlDouble(c,"k2"),
                yamlDouble(c,"p1"), yamlDouble(c,"p2"),
                yamlDouble(c,"k3"));

            depth_cols_ = yamlInt(c,"cols");
            depth_rows_ = yamlInt(c,"rows");
        }

        // ---- RGB camera (Camera) --------------------------------------------
        if (!cfg["Camera"])
            throw std::runtime_error("Section 'Camera' missing from YAML '" + path + "'.");
        {
            const auto c = cfg["Camera"];
            K_rgb_ = (cv::Mat_<double>(3,3) <<
                yamlDouble(c,"fx"), 0.0,               yamlDouble(c,"cx"),
                0.0,               yamlDouble(c,"fy"), yamlDouble(c,"cy"),
                0.0,               0.0,                1.0);

            // YAML : k1 k2 k3 p1 p2   →   OpenCV : k1 k2 p1 p2 k3
            D_rgb_ = (cv::Mat_<double>(1,5) <<
                yamlDouble(c,"k1"), yamlDouble(c,"k2"),
                yamlDouble(c,"p1"), yamlDouble(c,"p2"),
                yamlDouble(c,"k3"));

            rgb_cols_ = yamlInt(c,"cols");
            rgb_rows_ = yamlInt(c,"rows");
        }

        // ---- Transformation (Depth → RGB) -----------------------------------
        // YAML convention (same as ORB-SLAM3 / RIDSLAM):
        //   translation + rotation_quaternion describe T_rgb_depth (RGB←Depth)
        //   → inverted to get T_depth_to_rgb_ (Depth→RGB)
        if (!cfg["Transformation"])
            throw std::runtime_error("Section 'Transformation' missing from YAML '" + path + "'.");
        {
            const auto tf = cfg["Transformation"];

            const auto tv = tf["translation"];
            if (!tv || tv.size() != 3)
                throw std::runtime_error("Transformation.translation must have 3 elements [tx,ty,tz].");
            const Eigen::Vector3d t(tv[0].as<double>(), tv[1].as<double>(), tv[2].as<double>());

            const auto qv = tf["rotation_quaternion"];
            if (!qv || qv.size() != 4)
                throw std::runtime_error("Transformation.rotation_quaternion must have 4 elements [w,x,y,z].");
            Eigen::Quaterniond q(
                qv[0].as<double>(),  // w
                qv[1].as<double>(),  // x
                qv[2].as<double>(),  // y
                qv[3].as<double>()); // z
            q.normalize();

            Eigen::Matrix4d T_orig = Eigen::Matrix4d::Identity();
            T_orig.block<3,3>(0,0) = q.toRotationMatrix();
            T_orig.block<3,1>(0,3) = t;

            // stella_vslam convention: T_orig = T_color_from_depth
            // (expresses a depth point in the color frame, = direct Depth→RGB)
            T_depth_to_rgb_ = T_orig;
        }

        // ---- depthmap_factor ------------------------------------------------
        if (cfg["Preprocessing"] && cfg["Preprocessing"]["depthmap_factor"])
            depth_scale_ = 1.0 / cfg["Preprocessing"]["depthmap_factor"].as<double>();
    }

    // =========================================================================
    // buildReprojectionLUT
    //
    // Precomputes the RGB destination of each depth pixel (u,v).
    // The baseline translation is handled by assuming
    // that the ray direction does not change significantly with z
    // (valid approximation for z >> baseline, i.e. z > 0.3m with a ~32mm baseline).
    //
    // Note: for maximum accuracy at short range, the translation correction
    // is done at runtime using the real z (see comment
    // in depthCallback if it needs to be enabled).
    // =========================================================================
    void buildReprojectionLUT(int actual_cols, int actual_rows)
    {
        // Use the ACTUAL dimensions of the incoming image.
        // If they differ from the YAML (CameraIR.cols/rows), update them.
        if (actual_cols != depth_cols_ || actual_rows != depth_rows_)
        {
            RCLCPP_WARN(this->get_logger(),
                "Actual depth dimensions (%dx%d) != YAML (%dx%d). "
                "Using the actual dimensions and rescaling K_d_.",
                actual_cols, actual_rows, depth_cols_, depth_rows_);

            // Rescale the intrinsics if the resolution changed
            const double sx = static_cast<double>(actual_cols) / depth_cols_;
            const double sy = static_cast<double>(actual_rows) / depth_rows_;
            K_d_.at<double>(0,0) *= sx;  // fx
            K_d_.at<double>(1,1) *= sy;  // fy
            K_d_.at<double>(0,2) *= sx;  // cx
            K_d_.at<double>(1,2) *= sy;  // cy
            depth_cols_ = actual_cols;
            depth_rows_ = actual_rows;
        }
        // Depth camera undistortion maps (applied to every frame)
        cv::initUndistortRectifyMap(
            K_d_, D_d_, cv::noArray(), K_d_,
            cv::Size(depth_cols_, depth_rows_),
            CV_32FC1, undist_map1_, undist_map2_);

        lut_u_rgb_ = cv::Mat(depth_rows_, depth_cols_, CV_32SC1, cv::Scalar(-1));
        lut_v_rgb_ = cv::Mat(depth_rows_, depth_cols_, CV_32SC1, cv::Scalar(-1));

        // RGB distortion coefficients in OpenCV order: k1 k2 p1 p2 k3
        const double k1 = D_rgb_.at<double>(0,0);
        const double k2 = D_rgb_.at<double>(0,1);
        const double p1 = D_rgb_.at<double>(0,2);
        const double p2 = D_rgb_.at<double>(0,3);
        const double k3 = D_rgb_.at<double>(0,4);

        const Eigen::Matrix3d R = T_depth_to_rgb_.block<3,3>(0,0);

        const double fx_rgb = K_rgb_.at<double>(0,0);
        const double fy_rgb = K_rgb_.at<double>(1,1);
        const double cx_rgb = K_rgb_.at<double>(0,2);
        const double cy_rgb = K_rgb_.at<double>(1,2);
        const double fx_d   = K_d_.at<double>(0,0);
        const double fy_d   = K_d_.at<double>(1,1);
        const double cx_d   = K_d_.at<double>(0,2);
        const double cy_d   = K_d_.at<double>(1,2);

        for (int v = 0; v < depth_rows_; ++v)
        {
            int32_t* lu = lut_u_rgb_.ptr<int32_t>(v);
            int32_t* lv = lut_v_rgb_.ptr<int32_t>(v);

            for (int u = 0; u < depth_cols_; ++u)
            {
                // Normalized ray in the depth frame (z=1)
                const Eigen::Vector3d ray_d(
                    (u - cx_d) / fx_d,
                    (v - cy_d) / fy_d,
                    1.0);

                // Rotation to the RGB frame (no translation: baseline << z for z>0.3m)
                const Eigen::Vector3d ray_rgb = R * ray_d;
                const double xn = ray_rgb(0) / ray_rgb(2);
                const double yn = ray_rgb(1) / ray_rgb(2);

                // RGB radial + tangential distortion
                const double r2     = xn*xn + yn*yn;
                const double r4     = r2*r2;
                const double r6     = r4*r2;
                const double radial = 1.0 + k1*r2 + k2*r4 + k3*r6;
                const double xd = xn*radial + 2.0*p1*xn*yn        + p2*(r2 + 2.0*xn*xn);
                const double yd = yn*radial + p1*(r2 + 2.0*yn*yn)  + 2.0*p2*xn*yn;

                const int u_rgb = static_cast<int>(fx_rgb*xd + cx_rgb + 0.5);
                const int v_rgb = static_cast<int>(fy_rgb*yd + cy_rgb + 0.5);

                if (u_rgb >= 0 && u_rgb < rgb_cols_ &&
                    v_rgb >= 0 && v_rgb < rgb_rows_)
                {
                    lu[u] = u_rgb;
                    lv[u] = v_rgb;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(),
            "Reprojection LUT built (%dx%d → %dx%d pixels).",
            depth_cols_, depth_rows_, rgb_cols_, rgb_rows_);
    }

    // =========================================================================
    // depthCallback
    // =========================================================================
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        // ---- Build the LUT on the first call with the actual dimensions ----
        if (!lut_ready_)
        {
            buildReprojectionLUT(
                static_cast<int>(msg->width),
                static_cast<int>(msg->height));
            lut_ready_ = true;
        }

        // ---- Depth-map undistortion -----------------------------------------
        cv::Mat depth_undist;
        cv::remap(cv_ptr->image, depth_undist, undist_map1_, undist_map2_, cv::INTER_NEAREST);

        // ---- Projection into the RGB geometry with the full T (R + t) -------
        cv::Mat aligned = cv::Mat::zeros(rgb_rows_, rgb_cols_, CV_16UC1);

        const Eigen::Matrix3d R = T_depth_to_rgb_.block<3,3>(0,0);
        const Eigen::Vector3d t = T_depth_to_rgb_.block<3,1>(0,3);

        const double fx_d   = K_d_.at<double>(0,0);
        const double fy_d   = K_d_.at<double>(1,1);
        const double cx_d   = K_d_.at<double>(0,2);
        const double cy_d   = K_d_.at<double>(1,2);
        const double fx_rgb = K_rgb_.at<double>(0,0);
        const double fy_rgb = K_rgb_.at<double>(1,1);
        const double cx_rgb = K_rgb_.at<double>(0,2);
        const double cy_rgb = K_rgb_.at<double>(1,2);
        const double k1 = D_rgb_.at<double>(0,0);
        const double k2 = D_rgb_.at<double>(0,1);
        const double p1 = D_rgb_.at<double>(0,2);
        const double p2 = D_rgb_.at<double>(0,3);
        const double k3 = D_rgb_.at<double>(0,4);

        for (int v = 0; v < depth_rows_; ++v)
        {
            const uint16_t* row_d = depth_undist.ptr<uint16_t>(v);
            for (int u = 0; u < depth_cols_; ++u)
            {
                const uint16_t raw = row_d[u];
                if (raw == 0) continue;

                // Real depth in meters
                const double z = raw * depth_scale_;

                // Back-project into the depth frame
                const Eigen::Vector3d pt_d(
                    (u - cx_d) / fx_d * z,
                    (v - cy_d) / fy_d * z,
                    z);

                // Transform to the RGB frame (full R + t)
                const Eigen::Vector3d pt_rgb = R * pt_d + t;
                if (pt_rgb(2) <= 0.0) continue;

                // RGB projection + distortion
                const double xn = pt_rgb(0) / pt_rgb(2);
                const double yn = pt_rgb(1) / pt_rgb(2);
                const double r2     = xn*xn + yn*yn;
                const double radial = 1.0 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2;
                const double xd = xn*radial + 2.0*p1*xn*yn       + p2*(r2 + 2.0*xn*xn);
                const double yd = yn*radial + p1*(r2 + 2.0*yn*yn) + 2.0*p2*xn*yn;

                const int u_rgb = static_cast<int>(fx_rgb*xd + cx_rgb + 0.5);
                const int v_rgb = static_cast<int>(fy_rgb*yd + cy_rgb + 0.5);

                if (u_rgb >= 0 && u_rgb < rgb_cols_ &&
                    v_rgb >= 0 && v_rgb < rgb_rows_)
                {
                    // Z-buffer: keep the closest pixel
                    uint16_t& dst = aligned.at<uint16_t>(v_rgb, u_rgb);
                    if (dst == 0 || raw < dst) dst = raw;
                }
            }
        }

        // ---- Densification by morphological dilation ------------------------
        cv::Mat aligned_dense;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::dilate(aligned, aligned_dense, kernel);

        // Publish
        cv_bridge::CvImage out;
        out.header   = msg->header;
        out.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        out.image    = aligned_dense;
        pub_aligned_->publish(*out.toImageMsg());
    }
};

// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<DepthToRGBNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("depth_to_rgb"), "Fatal error: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
