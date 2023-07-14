voxblox++ debug idea

20221011:
## 1) TF tree: two timestamp: 
one from the vehicle: map -> odom -> base_link -> laser
one from the camera: camera_body -> rgb / depth_link

trick 1: manually set transform from camera and base_link, directly convert point cloud from camera_link to base_link.

trick 2: since voxblox++ focus on timestamp, so, we listner transform from base_link to odom and attach current odom timestamp to point cloud 

## 2) agiprobot.yaml: configuration
the point cloud are defined as three different type: 
see controller.cpp: 
    voxblox::PointSemanticInstanceType (xyz, rgb, semantic, instance)
    voxblox::PointLabelType (xyz, rgb, label)
    voxblox::PointType (xyz, rgb)
to select the point type:
    "semantic_instance_segmentation/enable_semantic_instance_segmentation": false
    "use_label_propagation": true
the code also add PCL::PointType, see PointSurfelSemanticInstance in common.h

trick: if we need to build the global map, choose general point type, if we just use object, choose pointLabel type. when to use semanticInstance type?

## 3) visualier
the visualier is based on PCL::Visualier to visualize mesh: mesh_merged_layer_
enable "meshing/visualize: true" to show mesh in PCL::Visualier
enable "publishers/publish_scene_mesh: true" to publish voxblox_mesh in rviz

## 4) integrater
std::shared_ptr<LabelTsdfIntegrator> integrator_;
in fact it is based on labelTsdf. 
process: 
1) point cloud call back
2) check integrated frame -> merge point cloud
3) process segmented point cloud -> convert point to segments with 1/3 VoxbloxPointType 
4) choose one VoxbloxPointType and update

# 20221012
how to run the code for agiprobot
terminal 1: -> launch object segmentation node, convert rgbd to point cloud
roslaunch object_segmentation debug.launch 

terminal 2: -> launch voxblox_plusplus node, configuration is in agiprobot.yaml
roslaunch object_segmentation voxblox_plusplus.launch 

terminal 3: -> play rosbag
rosbag play /home/benchun/dataset/agiprobot/agiprobot_2.bag

attention: 
we convert rgbd to point cloud and transform them into base_link.

# 20221026
1. when connect to voxblox++, we use label information, where the point cloud type is pcl::PointCloud<pcl::PointXYZRGBL>. 
2. when publish, we should publish the segmentation with only one label every time, not the whole point cloud with different labels. 
3. currently, we just need to publish object pcl in base_link, just local info, not global, we do not do object association and updating. 

# 20221201
1. since the wheel odom from agiprobot bag is not good, we create a new tf from odom to new_base_link, it works better. we do not publish it in odom frame, which does not work for voxblox++
