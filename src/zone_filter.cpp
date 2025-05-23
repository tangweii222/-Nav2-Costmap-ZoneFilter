#include "zone_filter/zone_filter.hpp"

#include <string>
#include <memory>
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_costmap_2d
{

ZoneFilter::ZoneFilter()
:   filter_info_sub_(nullptr),
    mask_sub_(nullptr),
    zone_state_pub_(nullptr),
    filter_mask_(nullptr),
    global_frame_(""),
    default_state_("M1"),
    current_state_(default_state_)
{
}

void ZoneFilter::initializeFilter(const std::string & filter_info_topic)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

//  declareParameter("default_state", rclcpp::ParameterValue(false));
    declareParameter("default_state", rclcpp::ParameterValue("M1"));    
    node->get_parameter(name_ + ".default_state", default_state_);
    declareParameter("zone_state_topic", rclcpp::ParameterValue("zone_state"));
    node->get_parameter(name_ + ".zone_state_topic", zone_state_topic_);
    declareParameter("flip_threshold", rclcpp::ParameterValue(50.0));

    filter_info_topic_ = filter_info_topic;
    filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
        filter_info_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&ZoneFilter::filterInfoCallback, this, std::placeholders::_1));

    global_frame_ = layered_costmap_->getGlobalFrameID();

    zone_state_pub_ = node->create_publisher<std_msgs::msg::String>(
        zone_state_topic_, rclcpp::QoS(10));
    zone_state_pub_->on_activate();

    base_ = BASE_DEFAULT;
    multiplier_ = MULTIPLIER_DEFAULT;

    changeState(default_state_);
}

void ZoneFilter::filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    auto node = node_.lock();
    if (!node) return;

    mask_topic_ = msg->filter_mask_topic;
    mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        mask_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&ZoneFilter::maskCallback, this, std::placeholders::_1));

    RCLCPP_INFO(logger_, "ZoneFilter: Subscribing to mask topic: %s", mask_topic_.c_str());
}

void ZoneFilter::maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    filter_mask_ = msg;
    RCLCPP_INFO(logger_, "ZoneFilter: maskCallback received map, size: %d x %d",
        msg->info.width, msg->info.height);
}

void ZoneFilter::process(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const geometry_msgs::msg::Pose2D & pose)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    if (!filter_mask_) {
        RCLCPP_WARN_THROTTLE(logger_, *(clock_), 2000,
        "ZoneFilter: Filter mask was not received");
        return;
    }

    geometry_msgs::msg::Pose2D mask_pose;
    if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
        return;
    }

    unsigned int mx, my;
    if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mx, my)) {
        RCLCPP_WARN(logger_, "ZoneFilter: Robot is outside of filter mask. Resetting to default.");
        changeState(default_state_);
        return;
    }

    int8_t mask_value = getMaskData(filter_mask_, mx, my);
    if (mask_value == nav2_util::OCC_GRID_UNKNOWN) {
        RCLCPP_WARN_THROTTLE(
            logger_, *(clock_), 2000,
            "ZoneFilter: Filter mask [%i, %i]` data is unknown.", mx, my);
        return;
    }

    std::string zone;
    // RCLCPP_INFO(logger_, "ZoneFilter: mask_value = %d", mask_value);

    if (mask_value == 0) {
        zone = "M3";  // 255 白色
    } else if (mask_value == 40) {
        zone = "M2";  // 150 淺灰
    } else if (mask_value == 100) {
        zone = "M1";  // 70 深灰
    } else {
        RCLCPP_WARN(logger_, "ZoneFilter: Unknown mask_value = %d", mask_value);
        zone = default_state_;  // Default to the default state if unknown
    }

    static rclcpp::Time last_publish_time = clock_->now();
    rclcpp::Duration publish_interval = rclcpp::Duration::from_seconds(0.5);

    if (zone != current_state_ || (clock_->now() - last_publish_time) >= publish_interval) {
        changeState(zone);
        last_publish_time = clock_->now();
    }
}

void ZoneFilter::resetFilter()
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    changeState(default_state_);
    filter_info_sub_.reset();
    mask_sub_.reset();
    zone_state_pub_->on_deactivate();
    zone_state_pub_.reset();
}

bool ZoneFilter::isActive()
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return filter_mask_ != nullptr;
}

void ZoneFilter::changeState(const std::string & state)
{
    current_state_ = state;
    // RCLCPP_INFO(logger_, "ZoneFilter: Switching to zone: %s", state.c_str());

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = state;
    zone_state_pub_->publish(std::move(msg));
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ZoneFilter, nav2_costmap_2d::Layer)
