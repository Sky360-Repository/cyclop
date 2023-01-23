#include <qhyccd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

class QHYCameraNode : public rclcpp::Node
{
public:
  QHYCameraNode() : Node("qhy_camera_node") {
  // Create parameters
    this->declare_parameter("camera_name", "QHY183");
    this->declare_parameter("resolution_x", 1920);
    this->declare_parameter("resolution_y", 1080);
    this->declare_parameter("frame_rate", 60);

    // Get parameters
    std::string camera_name = this->get_parameter("camera_name").as_string();
    int resolution_x = this->get_parameter("resolution_x").as_int();
    int resolution_y = this->get_parameter("resolution_y").as_int();
    int frame_rate = this->get_parameter("frame_rate").as_int();

    // Setup Diagnostics
    diagnostics_ = std::make_shared<diagnostic_updater::Updater>(this, "qhy_camera");
    diagnostics_->setHardwareID("QHY 183");
  
    // Initialize the mutex
    frame_mutex_ = std::make_unique<std::mutex>();

    // Connect to the camera
    int ret = OpenQHYCCD(camera_name.c_str());
    if (ret != QHYCCD_SUCCESS) {
      // Error connecting to camera
        RCLCPP_ERROR(this->get_logger(), "Error connecting to camera");
        return;
    }

    camhandle_ = GetQHYCCDId(0);
    
    // Set the camera resolution
    ret = SetQHYCCDResolution(camhandle_, 0, 0, resolution_x, resolution_y);
    if (ret != QHYCCD_SUCCESS) {
      // Error setting resolution
      RCLCPP_ERROR(this->get_logger(), "Error setting resolution");
      CloseQHYCCD(camhandle_);
      return;
    }

    // Create a publisher for the image topic
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 1);
    
    // Begin exposure
    ret = BeginQHYCCDLive(camhandle_);
    if (ret != QHYCCD_SUCCESS) {
      // Error starting exposure
      RCLCPP_ERROR(this->get_logger(), "Error starting exposure");
      CloseQHYCCD(camhandle_);
      return;
    }
    // Create a timer to get new frames
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frame_rate), std::bind(&QHYCameraNode::grabAndPublishFrame, this));
  }

private:
  std::shared_ptr<diagnostic_updater::Updater> diagnostics_;
void grabAndPublishFrame()
{
    int width, height, bpp, channels;
    std::shared_ptr<unsigned char> imgdata;

    // Wait for the exposure to finish
    int ret = GetQHYCCDLiveFrame(camhandle_, &width, &height, &bpp, &channels, &imgdata);
    if (ret != QHYCCD_SUCCESS) {
        // Error getting live frame
        RCLCPP_ERROR(this->get_logger(), "Error getting live frame");
        EndQHYCCDLive(camhandle_);
        CloseQHYCCD(camhandle_);

        // Report error to diagnostics
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "QHY 183 Camera";
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Error getting live frame";
        diagnostics_->broadcast(status);
        return;
    }

    // Create a sensor_msgs/Image message and fill in the data
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = height;
    msg->width = width;
    msg->step = width * channels;
    msg->encoding = channels == 1 ? "mono8" : "bgr8";
    msg->data = imgdata;

    // Publish the message
    pub_->publish(std::move(msg));

    // Report success to diagnostics
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "QHY 183 Camera";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Frame acquired successfully";
    diagnostics_->broadcast(status);
}
