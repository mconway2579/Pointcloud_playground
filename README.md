# Pointcloud_playground
experimenting with point-clouds to be displayed in rviz
## Launch Files
### pc2_color_from_image.launch
* uses pc2-from_rgb_image.py to create a point cloud
* takes absolute paths to the depth image and rgb image
* displays the overlay of the two images in rviz
### pc2_cube.launch
* uses pc2_cube.py to create a cube of points colored along the x y z axis
* opens an rviz window to display the cube
### pc2_from_bag.launch
* plays a bag file like the ones found in the nsh_east dataset
* uses pc2_from_ros.py to overlay the two images
* opens an rviz window to display the new point cloud
### pc2_from_image.launch
* takes an absolute path to a depth map image
* uses rviz to display the pointcloud created from the depthmap colored along the x y z axis

## Scripts
### shared.py
* defines the data in each point (x,y,z, rgb)
### reasign_frame_id.py
* take an Image message, change the frame_id field in the header, republish the message
### pc2_cube.py
* creates a PointCloud2 message that displays a 3d array of points
* ![point cloud cube](/readme_images/pc_cube.png)
### pc2_from_image.py
* given a depth map image create a point cloud from the depth map
* ![depth image](/readme_images/teddy_depth.png)
* ![point cloud](/readme_images/teddy_depth_pc2.png)
### pc2_from_rgb_image.py
* given a depth map image and an rgb image, overlay the two images into a pointcloud
* ![depth image](/readme_images/teddy_depth.png)
* ![rgb image](/readme_images/teddy_rgb.png)
* ![point cloud](/readme_images/teddy_rgb_pc2.png)
### pc2_from_ros.py
* given a depth image topic and an rgb image topic, overlay the two images into a pointcloud
* ![depth image topic](/readme_images/bag_depth.png)
* ![rbg image topic](/readme_images/bag_rgb.png)
* ![overlay](/readme_images/bag_pc2.png)



## Datasets
### depth_maps
* contains various depth maps from the internet
### middlebury
* https://vision.middlebury.edu/stereo/data/scenes2003/
### nsh_east
* https://drive.google.com/file/d/19iRkS9xEVwJXkP0er7KT7YZxsu8fFcmw/view
### ros_examples
* http://wiki.ros.org/depth_image_proc
### NYU
* https://cs.nyu.edu/~silberman/datasets/nyu_depth_v2.html#raw_parts
