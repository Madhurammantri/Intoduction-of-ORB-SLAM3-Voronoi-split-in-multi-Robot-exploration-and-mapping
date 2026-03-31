#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "System.h"
#include "ImuTypes.h"
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <deque>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <stdexcept>

class OrbSlam3Node : public rclcpp::Node
{
public:
    OrbSlam3Node()
    : rclcpp::Node("slam")
    {
        sensor_type_ = declare_parameter<std::string>("sensor_type", "mono-inertial");
        vocabulary_path_ = declare_parameter<std::string>("vocabulary", "");
        settings_path_ = declare_parameter<std::string>("settings_file", "");
        publish_dummy_odom_ = declare_parameter<bool>("publish_dummy_odom", false);
        show_debug_image_ = declare_parameter<bool>("show_debug_image", false);

        RCLCPP_INFO(get_logger(), "ORB-SLAM3 wrapper started");
        RCLCPP_INFO(get_logger(), "  sensor_type        = %s", sensor_type_.c_str());
        RCLCPP_INFO(get_logger(), "  vocabulary         = %s", vocabulary_path_.c_str());
        RCLCPP_INFO(get_logger(), "  settings file      = %s", settings_path_.c_str());
        RCLCPP_INFO(get_logger(), "  publish_dummy_odom = %s", publish_dummy_odom_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  show_debug_image   = %s", show_debug_image_ ? "true" : "false");

        if (vocabulary_path_.empty() || settings_path_.empty()) {
            throw std::runtime_error("ORB-SLAM3 vocabulary/settings path is empty");
        }

        ORB_SLAM3::System::eSensor orb_sensor;
        if (sensor_type_ == "mono-inertial") {
            orb_sensor = ORB_SLAM3::System::IMU_MONOCULAR;
        } else if (sensor_type_ == "mono") {
            orb_sensor = ORB_SLAM3::System::MONOCULAR;
        } else {
            throw std::runtime_error("Unsupported sensor_type: " + sensor_type_);
        }

        slam_ = std::make_shared<ORB_SLAM3::System>(
            vocabulary_path_,
            settings_path_,
            orb_sensor,
            false
        );

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&OrbSlam3Node::imageCallback, this, std::placeholders::_1)
        );

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&OrbSlam3Node::imuCallback, this, std::placeholders::_1)
        );

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("slam/pose", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("orb/odom", 10);

        if (publish_dummy_odom_) {
            RCLCPP_WARN(get_logger(),
                        "publish_dummy_odom:=true is set. This will publish dummy odometry instead of real ORB odometry.");
        } else {
            RCLCPP_INFO(get_logger(),
                        "Dummy odom disabled. This node will publish only real ORB-SLAM3 odometry.");
        }
    }

    ~OrbSlam3Node() override
    {
        try {
            if (slam_) {
                slam_->Shutdown();
            }
        } catch (...) {
        }

        if (show_debug_image_) {
            try {
                cv::destroyWindow(get_name());
            } catch (...) {
            }
        }
    }

private:
    std::string robotNsNoSlash() const
    {
        const std::string ns = get_namespace();
        if (!ns.empty() && ns[0] == '/') {
            return ns.substr(1);
        }
        return ns;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_buf_.push_back(*msg);
        have_imu_ = true;

        while (imu_buf_.size() > 4000) {
            imu_buf_.pop_front();
        }
    }

    bool convertImageToGray(const sensor_msgs::msg::Image::SharedPtr &msg, cv::Mat &gray_out)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            if (msg->encoding == "mono8") {
                cv_ptr = cv_bridge::toCvShare(msg, "mono8");
                gray_out = cv_ptr->image.clone();
                return true;
            }

            if (msg->encoding == "rgb8") {
                cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
                cv::cvtColor(cv_ptr->image, gray_out, cv::COLOR_RGB2GRAY);
                return true;
            }

            if (msg->encoding == "bgr8") {
                cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
                cv::cvtColor(cv_ptr->image, gray_out, cv::COLOR_BGR2GRAY);
                return true;
            }

            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv::cvtColor(cv_ptr->image, gray_out, cv::COLOR_BGR2GRAY);
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "cv_bridge conversion failed: %s", e.what());
            return false;
        }
    }

    std::vector<ORB_SLAM3::IMU::Point> collectImuMeasurements(const rclcpp::Time &img_time)
    {
        std::vector<ORB_SLAM3::IMU::Point> v_imu;

        std::lock_guard<std::mutex> lock(imu_mutex_);

        while (!imu_buf_.empty() &&
               rclcpp::Time(imu_buf_.front().header.stamp) <= last_image_time_) {
            imu_buf_.pop_front();
        }

        for (const auto &imu_msg : imu_buf_) {
            const rclcpp::Time t_imu(imu_msg.header.stamp);

            if (t_imu > last_image_time_ && t_imu <= img_time) {
                v_imu.emplace_back(
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y,
                    imu_msg.linear_acceleration.z,
                    imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z,
                    t_imu.seconds()
                );
            }
        }

        while (!imu_buf_.empty() &&
               rclcpp::Time(imu_buf_.front().header.stamp) <= img_time) {
            imu_buf_.pop_front();
        }

        return v_imu;
    }

    bool isPoseValid(const Sophus::SE3f &Tcw) const
    {
        const Eigen::Matrix4f m = Tcw.matrix();
        return m.allFinite();
    }

    void publishTrackedPose(const sensor_msgs::msg::Image::SharedPtr &msg, const Sophus::SE3f &Tcw)
    {
        const Sophus::SE3f Twc = Tcw.inverse();
        const Eigen::Vector3f t = Twc.translation();
        const Eigen::Quaternionf q(Twc.unit_quaternion());

        const std::string robot = robotNsNoSlash();

        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = msg->header.stamp;
        p.header.frame_id = robot + "/map";
        p.pose.position.x = t.x();
        p.pose.position.y = t.y();
        p.pose.position.z = t.z();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();
        pose_pub_->publish(p);

        nav_msgs::msg::Odometry o;
        o.header.stamp = msg->header.stamp;
        o.header.frame_id = robot + "/map";
        o.child_frame_id = robot + "/base_footprint";
        o.pose.pose = p.pose;

        for (size_t i = 0; i < 36; ++i) {
            o.pose.covariance[i] = 0.0;
            o.twist.covariance[i] = 0.0;
        }

        o.pose.covariance[0]  = 0.05;
        o.pose.covariance[7]  = 0.05;
        o.pose.covariance[14] = 0.10;
        o.pose.covariance[21] = 0.10;
        o.pose.covariance[28] = 0.10;
        o.pose.covariance[35] = 0.08;

        odom_pub_->publish(o);
    }

    void publishDummyOdom()
    {
        const std::string robot = robotNsNoSlash();

        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = now();
        p.header.frame_id = robot + "/map";
        p.pose.orientation.w = 1.0;
        pose_pub_->publish(p);

        nav_msgs::msg::Odometry o;
        o.header.stamp = p.header.stamp;
        o.header.frame_id = robot + "/odom";
        o.child_frame_id = robot + "/base_footprint";
        o.pose.pose.orientation.w = 1.0;

        for (size_t i = 0; i < 36; ++i) {
            o.pose.covariance[i] = 0.0;
        }

        o.pose.covariance[0]  = 0.25;
        o.pose.covariance[7]  = 0.25;
        o.pose.covariance[35] = 0.30;

        odom_pub_->publish(o);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat gray;
        if (!convertImageToGray(msg, gray)) {
            return;
        }

        if (show_debug_image_) {
            cv::imshow(get_name(), gray);
            cv::waitKey(1);
        }

        if (gray.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received empty image");
            return;
        }

        const rclcpp::Time img_time(msg->header.stamp);

        if (publish_dummy_odom_) {
            publishDummyOdom();
            last_image_time_ = img_time;
            return;
        }

        std::vector<ORB_SLAM3::IMU::Point> v_imu;
        if (sensor_type_ == "mono-inertial") {
            v_imu = collectImuMeasurements(img_time);

            if (v_imu.empty()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 3000,
                    "Received image but no IMU samples available yet for this frame");
                return;
            }
        }

        Sophus::SE3f Tcw;
        try {
            Tcw = slam_->TrackMonocular(gray, img_time.seconds(), v_imu, "");
        } catch (const std::exception &e) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ORB-SLAM3 tracking exception: %s", e.what());
            return;
        } catch (...) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ORB-SLAM3 tracking failed with unknown exception");
            return;
        }

        if (!isPoseValid(Tcw)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "ORB-SLAM3 has not produced a valid pose yet");
            last_image_time_ = img_time;
            return;
        }

        publishTrackedPose(msg, Tcw);
        last_image_time_ = img_time;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Published real ORB odometry (IMU=%s)",
            have_imu_ ? "true" : "false");
    }

    std::string sensor_type_;
    std::string vocabulary_path_;
    std::string settings_path_;
    bool publish_dummy_odom_{false};
    bool show_debug_image_{false};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::shared_ptr<ORB_SLAM3::System> slam_;

    std::deque<sensor_msgs::msg::Imu> imu_buf_;
    std::mutex imu_mutex_;
    rclcpp::Time last_image_time_{0, 0, RCL_ROS_TIME};

    bool have_imu_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrbSlam3Node>());
    rclcpp::shutdown();
    return 0;
}
