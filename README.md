# `zone_filter` package

A modular ROS 2 package for managing multiple **Costmap2D filters** (e.g., keepout zones, binary zones) with dynamic configuration and reusable launch structures.

### Features

* Supports multiple filters (e.g., `zone`, `keepout`) launched simultaneously.
* Each filter uses its own parameter YAML and mask map.
* Simple extensibility

---

## Usage

```bash
ros2 launch zone_filter multi_filter_bringup.launch.py
```

### Optional arguments:

```bash
use_zone:=true       # Launch 'zone' filter
use_keepout:=true    # Launch 'keepout' filter
```

---

## Adding a New Filter

To add a new filter (e.g., `xfilter`):

### 1. Edit `multi_filter_bringup.launch.py`

Add your filter name to the list:

```python
filter_types = ['zone', 'keepout', 'xfilter']
```

### 2. Prepare the files

* `params/xfilter_params.yaml`(and matching `xfilter`)
* `maps/xfilter_mask.yaml` (and matching `.pgm`)
* `maps/xfilter_mask.pgm`


Make sure the mask YAML uses an **absolute path** or is placed inside `install/share/zone_filter/maps`.

---