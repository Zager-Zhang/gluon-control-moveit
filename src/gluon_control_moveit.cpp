#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// 执行当前计划中的轨迹（若无法达到则返回false并输出FAILED）
bool my_execute_plan(moveit::planning_interface::MoveGroupInterface &move_group,moveit::planning_interface::MoveGroupInterface::Plan &my_plan){
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if (success)
    move_group.execute(my_plan.trajectory_);
  else
    return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_sim");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /********设置********/

  static const std::string PLANNING_GROUP = "gluon";

  //:planning_interface:`MoveGroupInterface` 类可以很容易地
  //仅使用您想要控制和计划的计划组的名称进行设置。
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  //我们将使用 :planning_interface:`PlanningSceneInterface`
  //在我们的“虚拟世界”场景中添加和删除碰撞对象的类
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //原始指针经常用于引用计划组以提高性能。
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /********可视化********/

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
  visual_tools.deleteAllMarkers();

  //远程控制是一种内省工具，允许用户单步执行高级脚本
  //通过 RViz 中的按钮和键盘快捷键
  visual_tools.loadRemoteControl();

  //RViz 提供了多种类型的标记，在这个演示中我们将使用文本、圆柱体和球体
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  //批量发布用于减少发送到 RViz 的大型可视化消息的数量
  visual_tools.trigger();

  /********获取基本信息********/

  //打印这个机器人的参考系名称。
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  //打印该组的末端执行器链接的名称。
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  //获得机器人中所有组的列表：
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  while(ros::ok()) {
    /********回到初始位置********/

    // 设置最大速度和加速度为其最大限制的20%
    move_group.setMaxVelocityScalingFactor(0.20);
    move_group.setMaxAccelerationScalingFactor(0.20);

    // 设置home位置
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go to home");
    std::vector<double> joint_home_positions(6, 0.0);
    move_group.setJointValueTarget(joint_home_positions);
    ROS_INFO("Go to home");

    // 判断规划是否成功，成功则执行
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_execute_plan(move_group,my_plan);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    /********移动到一个目标位置********/

    // 设置目标位置
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.20;
    target_pose1.position.y = -0.15;
    target_pose1.position.z = 0.22;
    move_group.setPoseTarget(target_pose1);

    my_execute_plan(move_group,my_plan);
    
    // 使用真正的机器人时，取消下面一行的注释
    // move_group.move();

    // 可视化规划结果
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    /********带中间点的笛卡尔目标规划********/

    // 设置目标点2的位置
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.y = 0.70711;
    start_pose2.orientation.w = 0.70711;
    start_pose2.position.x = 0.20;
    start_pose2.position.y = 0.0;
    start_pose2.position.z = 0.10;

    // 路径中要包含的点
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);

    geometry_msgs::Pose target_pose3 = start_pose2;

    // 移动到pose2起点
    move_group.setPoseTarget(start_pose2);
    ROS_INFO("Move to start_pose2");
    my_execute_plan(move_group,my_plan);
    
    /*使用真正的机器人时，取消下面一行的注释*/
    // move_group.move();

    target_pose3.position.x += 0.10;
    waypoints.push_back(target_pose3);  

    target_pose3.position.y += 0.30;
    waypoints.push_back(target_pose3);  

    target_pose3.position.z += 0.20;
    waypoints.push_back(target_pose3);  

    //我们希望以 1 cm 的分辨率对笛卡尔路径进行插值
    //这就是为什么我们将 0.01 指定为笛卡尔坐标的最大步长
    //翻译。我们将跳跃阈值指定为 0.0，有效地禁用它。
    //警告 -在操作真实硬件时禁用跳跃阈值可能会导致
    //冗余关节的大量不可预测的运动，可能是一个安全问题
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    //在 RViz 中可视化计划
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
      visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    //笛卡尔运动通常应该很慢，例如接近物体时。笛卡尔速度
    //目前无法通过 maxVelocityScalingFactor 设置计划，但需要您计时
    //手动轨迹，如 [此处](https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4) 所述。
    //欢迎请求请求。

    //  执行轨迹。
    int nrOfPoints = trajectory.joint_trajectory.points.size();
    ROS_INFO("nrOfPoints %d", nrOfPoints);
    for (int i = 0; i < nrOfPoints; i++) {
      //ActuatorController::UnifiedID actuator = uIDArray.at(i);
      int nrOfJoints = trajectory.joint_trajectory.points[i].positions.size();
      for (int j = 0; j < nrOfJoints; j++) {
        ROS_INFO("point[%d]: %s, %f", i, trajectory.joint_trajectory.joint_names[j].c_str(), 
            trajectory.joint_trajectory.points[i].positions[j]);
      }
      ROS_INFO("-------------------------------------------");
    }
    move_group.execute(trajectory);
    ros::Duration(2).sleep();

    ros::Duration(10).sleep();
  }
  ros::shutdown();
  return 0;
}
