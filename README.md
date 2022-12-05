# Pointcloud_playground
experimenting with point-clouds in the ros ecosystem
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
* ![point cloud cube]()
### pc2_from_image.py
* given a depth map image create a point cloud from the depth map
* ![depth image]()
* ![point cloud]()
### pc2_from_rgb_image.py
* given a depth map image and an rgb image, overlay the two images into a pointcloud
* ![depth image]()
* ![rgb image]()
* ![point cloud]()
### pc2_from_ros.py
* given a depth image topic and an rgb image topic, overlay the two images into a pointcloud
* ![depth image topic]()
* ![rbg image topic]()



## Datasets
### depth_maps
### middlebury
### nsh_east
### ros_examples
