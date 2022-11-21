# intelligent-robotics-2223
Repository for the subject Intelligent Robotics held in University of Padova.

# How to execute
## Exercise 4.1
### Starting package launch file
In order to start the launch file provided by the exercise you must execute:

``roslaunch exercise_4_1 static_robot_tf.launch``
### Playing the bag
In order to play the bag provided by the exercise you have to execute, in bag folder:

``rosbag play bag_es_41.bag -l``
### Starting the apriltag_ros node
First starting the apriltag_ros node we have to specify what tags we are going to detect, in terms of their IDs and sizes. This can be done adding to the ``apriltag_ros/apriltag_ros/config/tags.yaml`` config file the following:
```
tag_bundles:
  [
    {
        name: 'EXERCISE_4_BUNDLE',
        layout:
        [
            {id: 0, size: 0.15},
            {id: 1, size: 0.15},
            {id: 3, size: 0.15},
            {id: 9, size: 0.15},
            {id: 33, size: 0.15},
            {id: 34, size: 0.15}
        ]
    }
  ]
```
We need to do this because of the video stream provided in the exercise. It shows 6 apriltags in a bundle, so we will be detecting bundles of apriltags. Info about sizes: https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation

In order to start the apriltag_ros node that detects the apriltags with the camera data you have to execute:

```roslaunch apriltag_ros continuous_detection.launch camera_name:=/kinect/rgb image_topic:=image_rect_color```

After that, you will se info through the ``tag_detections`` topic.

### Starting the client
In order to start the client developed by us you have to execute:

``rosrun exercise_4_1 client_node``
