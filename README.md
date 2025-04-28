# Nav2-Costmap-ZoneFilter
# ZoneFilter - Nav2 Costmap Layer

## Overview

**ZoneFilter** is a custom ROS2 Costmap2D filter plugin designed for the Navigation2 (Nav2) stack.  
It dynamically classifies the robot's current location into different predefined zones based on a mask map, and publishes the zone information as a ROS topic.

This allows behavior trees, planners, or other systems to adjust robot behaviors (such as speed limits, safety modes, or navigation strategies) according to the zone the robot is currently located in.

---

## Key Features

- Subscribe to a mask map (`OccupancyGrid`) and classify zones based on pixel values.
- Publish zone state (`std_msgs/String`) at 0.5s intervals or when zone changes.
- Support for multiple custom zone types (e.g., M1, M2, M3).
- Designed for integration with Nav2 layered costmaps.

---

## Quick Start

### 1. Clone the repository

```bash
cd ~/your_ws/src
git clone git@github.com:tangweii222/-Nav2-Costmap-ZoneFilter.git zone_filter
```
### 2. Build the package
```bash
cd ~/your_ws
colcon build --packages-select zone_filter
```
### 3. Source the workspace
```bash
source install/setup.bash
```
### 4. Configure and launch
- Add zone_filter as a plugin in your nav2_params.yaml under plugins: list.
- Set parameters like default_state, zone_state_topic, and flip_threshold if needed.
- Ensure a proper mask map (OccupancyGrid) is provided to the filter.
Example:
```nav2_param.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      filters: [ "zone_filter", ... ]

      zone_filter:
        plugin: "nav2_costmap_2d::ZoneFilter"
        enabled: True
        filter_info_topic: "/zone_filter_info"
        zone_state_topic: "/zone_state"
        default_state: "M2"
```
Example Zones (based on mask value)
| Mask Value | Zone | Description             |
|:-----------|:-----|:-------------------------|
| 0          | M3   | Free driving area         |
| 40         | M2   | Side-constrained zone     |
| 100        | M1   | Restricted zone           |
