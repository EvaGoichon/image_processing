// ===========================================================================
// depth_to_rgb.cpp  —  ROS2 node
//
// Reprojecte les images depth (512x512, IR camera) dans la géométrie
// de la caméra RGB (2048x1536), pour superposition RGBD/RID-SLAM.
//
// Paramètres ROS2 :
//   calibration_file  (string)  — chemin vers RID_MuSeM.yaml (ou tout yaml
//                                 avec sections Camera / CameraIR / Transformation)
//   depth_topic       (string)  — défaut : "/depth/image_raw"
//   output_topic      (string)  — défaut : "/depth_to_rgb/image_raw"
//
// Format YAML attendu (même format que RID_MuSeM.yaml) :
//   Camera:    fx, fy, cx, cy, k1, k2, k3, p1, p2, cols, rows
//   CameraIR:  fx, fy, cx, cy, k1, k2, k3, p1, p2, cols, rows
//   Transformation:
//     translation:         [tx, ty, tz]          (en mètres)
//     rotation_quaternion: [w, x, y, z]
//   Preprocessing:
//     depthmap_factor: 1000.0
//
// Ordre distorsion OpenCV utilisé en interne : k1 k2 p1 p2 k3
// (le YAML stocke k1 k2 k3 p1 p2 — la conversion est faite à la lecture)
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
// Helpers YAML
// ---------------------------------------------------------------------------

static double yamlDouble(const YAML::Node& node, const std::string& key)
{
    if (!node[key])
        throw std::runtime_error("YAML: clé manquante : " + key);
    return node[key].as<double>();
}

static int yamlInt(const YAML::Node& node, const std::string& key)
{
    if (!node[key])
        throw std::runtime_error("YAML: clé manquante : " + key);
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
        // ---- Paramètres ROS2 ------------------------------------------------
        this->declare_parameter<std::string>("calibration_file", "");
        this->declare_parameter<std::string>("depth_topic",  "/depth/image_raw");
        this->declare_parameter<std::string>("output_topic", "/depth_to_rgb/image_raw");

        const std::string calib_path   = this->get_parameter("calibration_file").as_string();
        const std::string depth_topic  = this->get_parameter("depth_topic").as_string();
        const std::string output_topic = this->get_parameter("output_topic").as_string();

        if (calib_path.empty())
        {
            RCLCPP_FATAL(this->get_logger(),
                "Paramètre 'calibration_file' non défini.\n"
                "Usage : ros2 run <pkg> depth_to_rgb "
                "--ros-args -p calibration_file:=/home/eva/ros2_ws/src/stella_vslam_ros_RID/yaml/RID_WFOV_u.yaml");
            throw std::runtime_error("calibration_file manquant");
        }

        // ---- Chargement YAML ------------------------------------------------
        loadCalibration(calib_path);

        // La LUT est construite au premier message (on lit les vraies dimensions)

        // ---- Topics ---------------------------------------------------------
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, rclcpp::SensorDataQoS(),
            std::bind(&DepthToRGBNode::depthCallback, this, _1));

        pub_aligned_ = this->create_publisher<sensor_msgs::msg::Image>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(),
            "depth_to_rgb prêt.\n"
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

    // ---- Intrinsèques -------------------------------------------------------
    cv::Mat K_d_, D_d_;          // caméra Depth/IR  (OpenCV: k1 k2 p1 p2 k3)
    cv::Mat K_rgb_, D_rgb_;      // caméra RGB       (OpenCV: k1 k2 p1 p2 k3)
    int depth_cols_{0}, depth_rows_{0};
    int rgb_cols_{0},   rgb_rows_{0};
    double depth_scale_{0.001};  // = 1 / depthmap_factor

    // ---- Extrinsèques -------------------------------------------------------
    Eigen::Matrix4d T_depth_to_rgb_;   // Depth → RGB

    // ---- LUT pré-calculée ---------------------------------------------------
    // Construite au premier message pour utiliser les vraies dimensions.
    cv::Mat lut_u_rgb_;   // CV_32SC1, depth_rows x depth_cols
    cv::Mat lut_v_rgb_;   // CV_32SC1
    bool lut_ready_{false};

    // ---- Maps d'undistorsion depth (pré-calculées) --------------------------
    cv::Mat undist_map1_, undist_map2_;

    // =========================================================================
    // loadCalibration  —  lit le YAML format RID_MuSeM.yaml
    // =========================================================================
    void loadCalibration(const std::string& path)
    {
        YAML::Node cfg;
        try {
            cfg = YAML::LoadFile(path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Impossible de lire le YAML '" + path + "' : " + e.what());
        }

        // ---- Caméra Depth / IR (CameraIR) -----------------------------------
        if (!cfg["CameraIR"])
            throw std::runtime_error("Section 'CameraIR' absente du YAML '" + path + "'.");
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

        // ---- Caméra RGB (Camera) --------------------------------------------
        if (!cfg["Camera"])
            throw std::runtime_error("Section 'Camera' absente du YAML '" + path + "'.");
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
        // Convention du YAML (même qu'ORB-SLAM3 / RIDSLAM) :
        //   translation + rotation_quaternion décrivent T_rgb_depth (RGB←Depth)
        //   → on l'inverse pour obtenir T_depth_to_rgb_ (Depth→RGB)
        if (!cfg["Transformation"])
            throw std::runtime_error("Section 'Transformation' absente du YAML '" + path + "'.");
        {
            const auto tf = cfg["Transformation"];

            const auto tv = tf["translation"];
            if (!tv || tv.size() != 3)
                throw std::runtime_error("Transformation.translation doit avoir 3 éléments [tx,ty,tz].");
            const Eigen::Vector3d t(tv[0].as<double>(), tv[1].as<double>(), tv[2].as<double>());

            const auto qv = tf["rotation_quaternion"];
            if (!qv || qv.size() != 4)
                throw std::runtime_error("Transformation.rotation_quaternion doit avoir 4 éléments [w,x,y,z].");
            Eigen::Quaterniond q(
                qv[0].as<double>(),  // w
                qv[1].as<double>(),  // x
                qv[2].as<double>(),  // y
                qv[3].as<double>()); // z
            q.normalize();

            Eigen::Matrix4d T_orig = Eigen::Matrix4d::Identity();
            T_orig.block<3,3>(0,0) = q.toRotationMatrix();
            T_orig.block<3,1>(0,3) = t;

            // Convention stella_vslam : T_orig = T_color_from_depth
            // (exprime un point depth dans le repère color, = Depth→RGB direct)
            T_depth_to_rgb_ = T_orig;
        }

        // ---- depthmap_factor ------------------------------------------------
        if (cfg["Preprocessing"] && cfg["Preprocessing"]["depthmap_factor"])
            depth_scale_ = 1.0 / cfg["Preprocessing"]["depthmap_factor"].as<double>();
    }

    // =========================================================================
    // buildReprojectionLUT
    //
    // Pré-calcule la destination RGB de chaque pixel depth (u,v).
    // La translation de la baseline est prise en compte en assumant
    // que la direction du rayon ne change pas significativement avec z
    // (approximation valide pour z >> baseline, i.e. z > 0.3m avec baseline ~32mm).
    //
    // Note : pour une précision maximale à courte distance, la correction
    // de translation se fait au runtime en utilisant z réel (voir commentaire
    // dans depthCallback si besoin de l'activer).
    // =========================================================================
    void buildReprojectionLUT(int actual_cols, int actual_rows)
    {
        // Utilise les dimensions RÉELLES de l'image entrante.
        // Si elles diffèrent du YAML (CameraIR.cols/rows), on met à jour.
        if (actual_cols != depth_cols_ || actual_rows != depth_rows_)
        {
            RCLCPP_WARN(this->get_logger(),
                "Dimensions depth réelles (%dx%d) ≠ YAML (%dx%d). "
                "On utilise les dimensions réelles et on rescale K_d_.",
                actual_cols, actual_rows, depth_cols_, depth_rows_);

            // Rescale des intrinsèques si la résolution a changé
            const double sx = static_cast<double>(actual_cols) / depth_cols_;
            const double sy = static_cast<double>(actual_rows) / depth_rows_;
            K_d_.at<double>(0,0) *= sx;  // fx
            K_d_.at<double>(1,1) *= sy;  // fy
            K_d_.at<double>(0,2) *= sx;  // cx
            K_d_.at<double>(1,2) *= sy;  // cy
            depth_cols_ = actual_cols;
            depth_rows_ = actual_rows;
        }
        // Maps d'undistorsion de la caméra depth (appliquées à chaque frame)
        cv::initUndistortRectifyMap(
            K_d_, D_d_, cv::noArray(), K_d_,
            cv::Size(depth_cols_, depth_rows_),
            CV_32FC1, undist_map1_, undist_map2_);

        lut_u_rgb_ = cv::Mat(depth_rows_, depth_cols_, CV_32SC1, cv::Scalar(-1));
        lut_v_rgb_ = cv::Mat(depth_rows_, depth_cols_, CV_32SC1, cv::Scalar(-1));

        // Coefficients de distorsion RGB en ordre OpenCV : k1 k2 p1 p2 k3
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
                // Rayon normalisé dans le repère depth (z=1)
                const Eigen::Vector3d ray_d(
                    (u - cx_d) / fx_d,
                    (v - cy_d) / fy_d,
                    1.0);

                // Rotation vers le repère RGB (sans translation : baseline << z pour z>0.3m)
                const Eigen::Vector3d ray_rgb = R * ray_d;
                const double xn = ray_rgb(0) / ray_rgb(2);
                const double yn = ray_rgb(1) / ray_rgb(2);

                // Distorsion radiale + tangentielle RGB
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
            "LUT de reprojection construite (%dx%d → %dx%d pixels).",
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

        // ---- Construire la LUT au premier appel avec les vraies dimensions ----
        if (!lut_ready_)
        {
            buildReprojectionLUT(
                static_cast<int>(msg->width),
                static_cast<int>(msg->height));
            lut_ready_ = true;
        }

        // ---- Undistorsion de la depth-map -----------------------------------
        cv::Mat depth_undist;
        cv::remap(cv_ptr->image, depth_undist, undist_map1_, undist_map2_, cv::INTER_NEAREST);

        // ---- Projection dans la géométrie RGB avec T complète (R + t) -------
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

                // Profondeur réelle en mètres
                const double z = raw * depth_scale_;

                // Back-project dans le repère depth
                const Eigen::Vector3d pt_d(
                    (u - cx_d) / fx_d * z,
                    (v - cy_d) / fy_d * z,
                    z);

                // Transformer vers le repère RGB (R + t complets)
                const Eigen::Vector3d pt_rgb = R * pt_d + t;
                if (pt_rgb(2) <= 0.0) continue;

                // Projection + distorsion RGB
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
                    // Z-buffer : on garde le pixel le plus proche
                    uint16_t& dst = aligned.at<uint16_t>(v_rgb, u_rgb);
                    if (dst == 0 || raw < dst) dst = raw;
                }
            }
        }

        // ---- Densification par dilatation morphologique ---------------------
        cv::Mat aligned_dense;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::dilate(aligned, aligned_dense, kernel);

        // Publication
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
        RCLCPP_FATAL(rclcpp::get_logger("depth_to_rgb"), "Erreur fatale : %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
