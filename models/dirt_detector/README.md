# Dirt detector (YOLO11n)

Trained on simulated floor-camera images from the RoboVAC Gazebo sim
(10 episodes, 3040 train / 1062 val / 1296 test images, split by episode).
Classes: 0 crumbs, 1 dust, 2 spill, 3 hair. Input 640x640.

Held-out test episodes (mAP50): all 0.68, crumbs 0.74, dust 0.54, spill 0.84, hair 0.60.
Dust recall is low (0.29). Not yet tested on real photos.

Files: .pt (Ultralytics weights), .onnx (for ONNX Runtime on the robot).
Training: tools/dirt_dataset/train_dirt_yolo.ipynb
