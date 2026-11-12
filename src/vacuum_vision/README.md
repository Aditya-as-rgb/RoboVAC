# vacuum_vision

Runs the trained dirt detector (`models/dirt_detector/dirt_yolo11n.onnx`) on the floor camera and
turns what it sees into dirt spots on the map.

```
camera/image ──> YOLO11n (ONNX) ──> boxes ──> pixel_to_floor (camera pose from TF) ──> map x,y
                                                    └─> SpotTracker: merge repeated sightings,
                                                        confirm after min_hits, drop when the camera
                                                        looks straight at the spot and finds nothing
```

## Install onnxruntime once (system Python, not a venv)

```
pip install --user --break-system-packages onnxruntime "numpy<2"
```

## Run

```
ros2 launch vacuum_bringup vacuum.launch.py detector:=true dirt:=15 dirt_seed:=3 dirt_clean:=false
# or on its own, with the sim already running with cameras:=true
ros2 launch vacuum_vision detector.launch.py
```

`detector:=true` turns the floor camera on by itself. RViz then has two image panels, **Camera**
(`/camera/image`) and **Dirt detections** (`/dirt/debug_image`, boxes drawn), and a **Dirt detected
(camera)** marker layer next to the (off by default) **Dirt ground truth** layer.

| Topic | Type | What |
| ----- | ---- | ---- |
| `/dirt/detections` | `std_msgs/String` (JSON, latched) | confirmed spots `{id, x, y, kind, conf, hits}` in `map` |
| `/dirt/detected_markers` | `MarkerArray` | the same spots in RViz |
| `/dirt/debug_image` | `sensor_msgs/Image` | camera frame with boxes (published only while watched) |

Parameters: `config/detector.yaml`. Model path: `model:=...`, `$VACUUM_DIRT_MODEL`, or searched for
in the repo.

## Tests (no ROS, no Gazebo)

```
python3 -m pytest src/vacuum_vision/test -q
```
