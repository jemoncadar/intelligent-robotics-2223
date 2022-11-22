# intelligent-robotics-2223
Repository for the subject Intelligent Robotics held in University of Padova.

# How to execute
## Exercise 4.1.a
### Playing the bag
In order to play the bag provided by the exercise you have to execute, in bag folder:

``rosbag play bag_es_41.bag -l``
### Starting the apriltag_ros node
In order to start the apriltag_ros node, execute:

``roslaunch exercise_4_1 iaslab_apriltag.launch``

After that, you will se info through the ``tag_detections`` topic.

### Starting the static transformation broadcaster
In order to start the static transformation broadcaster, execute:

``roslaunch exercise_4_1 static_robot_tf.launch``

### Starting the node
In order to start the client developed by us you have to execute:

``rosrun exercise_4_1 detections_printer_node``

## Exercise 4.1.b

### Starting the node

With all the previous commands running, execute:

``rosrun exercise_4_1 transformations_printer_node``

## Exercise 4.2
### Playing the bag
In order to play the bag provided by the exercise you have to execute, in bag folder:

``rosbag play bag_es_42.bag -l``

### Starting the apriltag_ros node
In order to start the apriltag_ros node, execute:

``roslaunch exercise_4_1 iaslab_apriltag.launch``

### Starting the node
In order to start the client developed by us you have to execute:

``rosrun exercise_4_2 average_position_node``
