# webremote to control mobile robots on ROS2

A flask app that serve a remote controller interface that can be accessed from any device and publish twist msg on /cmd_vel

* depends on p2os_msgs

```
cd /smart_home_ws
source install/setup.bash

cd <webremote.py directory>
python3 webremote.py
```

<img src="demo.png" width=40% height=40%>
 