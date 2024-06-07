# AutoMind Assignment

Explanations about approach can be found in this [report](https://drive.google.com/file/d/1fIlzZgROak-ZKlqGXKAAQM6tgJJrGrda/view?usp=sharing)

## Task 1

To run the simulation:

``` bash
roslaunch automind-assignment sim.launch
```

And to run the controller:

``` bash
rosrun automind-assignment controller.py
```

## Task 2

Download the kitti dataset and convert them to rosbags by following the instructions in the [kitti2bag](https://github.com/tomas789/kitti2bag) repository

To play the rosbag:

``` bash
rosbag play -l kitti_2011_09_26_drive_xxxx_synced.bag
```

Make sure to replace 'xxxx' with the actual file number

To run the code:

``` bash
rosrun automind-assignment objdetect.py
```
