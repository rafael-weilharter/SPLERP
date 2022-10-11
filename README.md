# Overview

This spline/slerp (SPLERP) algorithm can help to record a video that blends between real images and a corresponding pointcloud. 

# Usage for the Tanks and Temples dataset

### 1) Generate a trajectory file from the MVSNet cam folder:
```
python cam_to_tum.py --cams=/path/to/cams --output=/output/path/trajectory.txt
```

### 2) Interpolate the trajectory with the SPLERP algorithm:
```
python pointcloud_video.py --trajectory=/path/to/trajectory.txt --mode=interpolate --splines=10 --outdir=/path/to/trajectory_interpolated.txt
```

### 3) Create pointcloud and depth images:
```
python pointcloud_video.py --trajectory=/path/to/trajectory_interpolated.txt --mode=view --pcd=/path/to/pointcloud.ply --record --outdir=/path/to/output_folder
```
This will create two folders `depth/` and `image/` in `output_folder/` with the depth.png and rendered.png images.

### 4) Extract frames from TaT video:
```
python video_helpers.py --mode=frames --pathFrames=/path/to/frame_folder --path/to/video.mp4
```

### 5) Put the different frames (video, rendered and depth) into one folder with ascending numbers

### 6) Combine frames to video
```
python video_helpers.py --mode=video --fps=29 --pathFrames=/path/to/frame_folder --path/to/video_blended.mp4
```