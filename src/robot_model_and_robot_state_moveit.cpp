#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

void print_joint_values(const moveit::core::JointModelGroup* joint_model_group,
                        moveit::core::RobotStatePtr &kinematic_state,
                        std::vector<double> &joint_values)
{  
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]*180/3.14);
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_moveit");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //开始教程
  //开始
  //^^^^^
  //设置以开始使用 RobotModel 类非常简单。在
  //总的来说，你会发现大多数高层组件都会
  //返回指向 RobotModel 的共享指针。你应该总是使用
  //如果可能的话。在这个例子中，我们将从这样一个
  //共享指针并且只讨论基本的 API。你可以有一个
  //查看这些类的实际代码 API 以获得更多信息
  //有关如何使用这些提供的更多功能的信息
  //类。
  //
  //我们将从实例化一个开始
  //`RobotModelLoader`_
  //对象，它将查找
  //ROS参数服务器上的机器人描述并构造一个
  //:moveit_core:`RobotModel` 供我们使用。
  //
  //.. _RobotModelLoader:
  //http://docs.ros.org/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  //使用 :moveit_core:`RobotModel`，我们可以构造一个
  //:moveit_core:`RobotState` 维护配置
  //机器人的。我们将状态中的所有关节设置为它们的
  //默认值。然后我们可以得到一个
  //:moveit_core:`JointModelGroup`，代表机器人
  //特定组的模型，例如熊猫的“熊猫臂”
  //机器人。
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("gluon");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  //获取联合值
  //^^^^^^^^^^^^^^^^
  //我们可以检索存储在 Panda 手臂状态中的当前关节值集。
  std::vector<double> joint_values;
  print_joint_values(joint_model_group,kinematic_state,joint_values);

  //联合限制
  //^^^^^^^^^^^^
  //setJointGroupPositions() 本身不会强制执行关节限制，但调用 enforceBounds() 会执行此操作。
  /*将 Panda 手臂中的一个关节设置在其关节限制之外*/
  joint_values[0] = 4.88;
  joint_values[1] = 5.88;
  joint_values[2] = -3.88;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /*检查是否有任何关节超出其关节限制*/
  printf("\n------------设置关节超过其限制------------\n");
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
  
  print_joint_values(joint_model_group,kinematic_state,joint_values);

  /*对该状态执行联合限制并再次检查*/
  printf("\n------------对该状态进行强制限制------------\n");
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
	
  print_joint_values(joint_model_group,kinematic_state,joint_values);
	
  
  //正向运动学
  //^^^^^^^^^^^^^^^^^^^
  //现在，我们可以计算一组随机关节的正向运动学
  kinematic_state->setToRandomPositions(joint_model_group);
  printf("\n------------随机后的机械臂各个关节弧度------------\n");
  print_joint_values(joint_model_group,kinematic_state,joint_values);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("6_Link");
  
  Eigen::Vector3d t1;
  t1 <<  0.2,0.2,0.2;
  Eigen::Isometry3d target_state = Eigen::Isometry3d::Identity();
  target_state.pretranslate(t1);
  printf("\n------------目标姿势------------\n");
  ROS_INFO_STREAM("Translation: \n" << target_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << target_state.rotation() << "\n");

  /*打印末端执行器姿势。请记住，这是在模型框架中*/
  printf("\n------------末端执行器姿势------------\n");
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  //逆运动学
  //^^^^^^^^^^^^^^^^^^^
  //我们现在可以解决 Panda 机器人的反向运动学 (IK)。
  //要解决 IK，我们需要以下内容：
  //
  //*末端执行器的所需姿势（默认情况下，这是“panda_arm”链中的最后一个链接）：
  //我们在上面的步骤中计算的 end_effector_state。
  //*超时时间：0.1 秒
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, target_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    printf("\n------------IK后解算的机械臂各个关节弧度------------\n");
    print_joint_values(joint_model_group,kinematic_state,joint_values);
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
  #if 0

  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the :moveit_core:`RobotState`.
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
	#endif

	// END_TUTORIAL
  ros::shutdown();
  return 0;
}
