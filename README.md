# gluon-control-moveit

## 配置

在minisca官网下载其ros包，直接放入catkin_ws/src/ros_gluon/cm_moveit/examples中

然后在cm_moveit的CMakeLists文件加入一下语句，来添加该子目录

```
add_subdirectory(examples/gluon_control_moveit)
```

编译运行即可

## 功能

gluon_final.cpp中对gluon机械臂进行了机械臂解算，求出了解析解