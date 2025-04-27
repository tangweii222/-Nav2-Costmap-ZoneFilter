#ifndef ZONE_FILTER__ZONE_FILTER_HPP_
#define ZONE_FILTER__ZONE_FILTER_HPP_

#include <string>
#include <memory>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

#include "std_msgs/msg/string.hpp"

namespace nav2_costmap_2d
{

class ZoneFilter : public CostmapFilter
{
public:
    ZoneFilter();

    void initializeFilter(const std::string & filter_info_topic) override;

    void process(
        Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j,
        const geometry_msgs::msg::Pose2D & pose) override;

    void resetFilter() override;

    bool isActive() ;

private:
    void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
    void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void changeState(const std::string & state);

    std::string global_frame_;
    std::string filter_info_topic_;
    std::string mask_topic_;
    std::string zone_state_topic_;

    rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
    // pub type
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr zone_state_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr filter_mask_;

    double base_, multiplier_;
    // double flip_threshold_;
    // state variables
    std::string default_state_;
    std::string current_state_;
};

}  // namespace nav2_costmap_2d

#endif  // ZONE_FILTER__ZONE_FILTER_HPP_
