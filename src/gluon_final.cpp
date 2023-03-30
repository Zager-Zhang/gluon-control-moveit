#include <ros/ros.h>
#include <math.h>
#include <string.h>

// robot model and state
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// move group
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// visual_tools
#include <moveit_visual_tools/moveit_visual_tools.h>

#define pi 3.1415926

double dh_theta[6] = {0,0,0,0,0,0};
double dh_d[6] = {105.03 ,0 ,0 ,80.09 ,80.09 ,44.36};
double dh_a[6] = {0 ,-174.42 ,-174.42 ,0 ,0};
// double dh_d[6] = {120.5 ,0 ,0 ,79.2 ,79.2 ,8.23};
// double dh_a[6] = {0 ,-203.5 ,-173 ,0 ,0};
double dh_alfa[6] = {pi/2 ,0 ,0 ,pi/2 ,-pi/2 ,0};
double jointMat[8][6];
bool is_solution[8];
std::vector<double> jointVal[8];
double poseMat[16];
/* 末端位姿转换变换矩阵（单位：mm）*/
void poseMatrix(double *poseMat, double roll, double pitch, double yaw, double x, double y, double z)
{
	double mat[16] = {cos(pitch)*cos(yaw),	sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw), cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw), 		x,
					 cos(pitch)*sin(yaw),   sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw),			y,
					 		 -sin(pitch),  								cos(pitch)*sin(roll),  							  cos(roll)*cos(pitch),			z,
									   0,												   0,					  							 0,			1};
	memcpy (poseMat, mat, 16*sizeof(double));
}

/* 逆解->解析解（单位:rad）*/
bool invKinematics(double * poseMat,bool * is_solution)//传入位姿阵 获得解析解
{
    for(int i=0;i<8;i++)
        is_solution[i]=0;
    double px = poseMat[3];
    double py = poseMat[7];
    double pz = poseMat[11];
    double nx = poseMat[0];
    double ny = poseMat[4];
    double nz = poseMat[8];
    double ox = poseMat[1];
    double oy = poseMat[5];
    double oz = poseMat[9];
    double ax = poseMat[2];
    double ay = poseMat[6];
    double az = poseMat[10];

    double m = dh_d[5]*ay - py;
    double n = dh_d[5]*ax - px;
    double r = m*m + n*n - dh_d[3]*dh_d[3];

	double theta11, theta12, theta51, theta52, theta53, theta54;
	
	double mm, nn, mm1, mm2, mm3, mmm, nn1, nnn, hhh, s2, c2, zzz1, zzz2, s234, c234;

    if (r >= 0)
    {
        theta11 = atan2(m,n) - atan2(dh_d[3],sqrt(r));
        theta12 = atan2(m,n) - atan2(dh_d[3],-sqrt(r));

        for(int i = 0; i < 4; i++)
        {
            jointMat[i][0] = theta11;
            jointMat[i+4][0] = theta12;
        }
        theta51=acos(ax*sin(theta11)-ay*cos(theta11));
        theta52=-acos(ax*sin(theta11)-ay*cos(theta11));
        theta53=acos(ax*sin(theta12)-ay*cos(theta12));
        theta54=-acos(ax*sin(theta12)-ay*cos(theta12));
        for(int i = 0; i < 2; i++)
        {
            jointMat[i][4] = theta51;
            jointMat[i+2][4] = theta52;
            jointMat[i+4][4] = theta53;
            jointMat[i+6][4] = theta54;
        }

        for(int i = 0; i < 8; i++)
        {
            mm=nx*sin(jointMat[i][0])-ny*cos(jointMat[i][0]);                      //nxs1-nyc1
            nn=ox*sin(jointMat[i][0])-oy*cos(jointMat[i][0]);                      //oxs1-oyc1
            jointMat[i][5]=atan2(mm,nn)-atan2(sin(jointMat[i][4]),0);
            mm1=sin(jointMat[i][5])*(nx*cos(jointMat[i][0])+ny*sin(jointMat[i][0]));//s6(nxc1+nys1)
            mm2=cos(jointMat[i][5])*(ox*cos(jointMat[i][0])+oy*sin(jointMat[i][0]));//c6(oxs1+oys1)
            mm3=ax*cos(jointMat[i][0])+ay*sin(jointMat[i][0]);                     //axc1+ays1
            mmm=dh_d[4]*(mm1+mm2)+px*cos(jointMat[i][0])-dh_d[5]*mm3+py*sin(jointMat[i][0]);
            nn1=oz*cos(jointMat[i][5])+nz*sin(jointMat[i][5]);                     //ozc6+nzs6
            nnn=dh_d[4]*nn1+pz-dh_d[0]-az*dh_d[5];
            hhh=(mmm*mmm+nnn*nnn-dh_a[1]*dh_a[1]-dh_a[2]*dh_a[2])/(2*dh_a[1]*dh_a[2]);
            if (hhh > 1)
                continue;
            jointMat[i][2]=pow(-1,i)*acos((mmm*mmm+nnn*nnn-dh_a[1]*dh_a[1]-dh_a[2]*dh_a[2])/(2*dh_a[1]*dh_a[2]));
            s2=(dh_a[2]*cos(jointMat[i][2])+dh_a[1])*nnn-dh_a[2]*sin(jointMat[i][2])*mmm;
            s2=s2/(dh_a[1]*dh_a[1]+dh_a[2]*dh_a[2]+2*dh_a[1]*dh_a[2]*cos(jointMat[i][2]));
            c2=(mmm+dh_a[2]*sin(jointMat[i][2])*s2);
            c2=c2/(dh_a[2]*cos(jointMat[i][2])+dh_a[1]);
            jointMat[i][1]=atan2(s2,c2);
            zzz1=nx*cos(jointMat[i][0])+ny*sin(jointMat[i][0]);                  //nxc1+nys1
            zzz2=ox*cos(jointMat[i][0])+oy*sin(jointMat[i][0]);                    //oxc1+oys1
            s234=sin(jointMat[i][5])*zzz1+cos(jointMat[i][5])*zzz2;
            c234=oz*cos(jointMat[i][5])+nz*sin(jointMat[i][5]);                    //ozc6+nzs6
                    
            jointMat[i][3]=atan2(-s234,c234)-jointMat[i][1]-jointMat[i][2];
            
            for(int j = 0; j < 6; j++)
            {
                if (jointMat[i][j]>pi)
                    jointMat[i][j] -= 2*pi;
                if (jointMat[i][j]<-pi)
                    jointMat[i][j] += 2*pi;
            }

            //由于moveit中的DH建模与该代码本身DH建模的不同，在此作出修正
            jointMat[i][0] -= pi/2;
            jointMat[i][1] += pi/2;
            jointMat[i][3] += pi/2;
            for(int j = 0; j < 6; j++)
                if(j!=2)
                    jointMat[i][j] *= -1;
			for(int j = 0; j < 6; j++){
                if(jointMat[i][j] > pi)
                    jointMat[i][j] -= 2*pi;
                if(jointMat[i][j] < -pi)
                    jointMat[i][j] += 2*pi;
            }
            jointMat[i][5] = 0;
            is_solution[i] = 1;
            // printf("solution NO%d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",i,jointMat[i][0],jointMat[i][1],jointMat[i][2],jointMat[i][3],jointMat[i][4],jointMat[i][5]);
			printf("solution NO%d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",i,jointMat[i][0]*180/pi,jointMat[i][1]*180/pi,jointMat[i][2]*180/pi,jointMat[i][3]*180/pi,jointMat[i][4]*180/pi,jointMat[i][5]*180/pi);
	    }
		return true;
	}else{
		printf("error\r\n");
		return false;
	}
}

/* 输出各关节角度 */
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

// 执行当前计划中的轨迹（若无法达到则返回false并输出FAILED）
bool my_execute_plan(moveit::planning_interface::MoveGroupInterface &move_group,moveit::planning_interface::MoveGroupInterface::Plan &my_plan){
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if (success)
    move_group.execute(my_plan.trajectory_);
  else
    return false;
}

int main(int argc, char *argv[])
{
    // 开启线程
    ros::init(argc, argv, "gluon_final");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    /* ---------------配置--------------- */
    // move group
    static const std::string PLANNING_GROUP = "gluon";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // robot model and state
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("gluon");
    
    // visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("dummy");

    /* ---------------设置速度和加速度--------------- */
    move_group.setMaxVelocityScalingFactor(0.20);
    move_group.setMaxAccelerationScalingFactor(0.20);
    
    /* ---------------Step1: 返回home位置--------------- */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to << go to home >>");
    std::vector<double> joint_home_positions(6, 0.0);
    move_group.setJointValueTarget(joint_home_positions);
    ROS_INFO("Go to home");

    // 判断规划是否成功，成功则执行
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_execute_plan(move_group,my_plan);
    
    /*---------------Step2: 获得当前角度并输出---------------*/
    std::vector<double> joint_values;
    print_joint_values(joint_model_group,kinematic_state,joint_values);

    /*---------------Step3: 设置目标并求逆解（数值解）---------------*/ 
    // // 设置目标位姿
    // Eigen::Vector3d trans;
    // trans <<  0.2,0.2,0.2;
    // Eigen::Isometry3d target_state = Eigen::Isometry3d::Identity();
    // target_state.pretranslate(trans);

    // // 输出目标位姿
    // printf("\n------------目标姿势------------\n");
    // ROS_INFO_STREAM("Translation: \n" << target_state.translation() << "\n");
    // ROS_INFO_STREAM("Rotation: \n" << target_state.rotation() << "\n");

    // // 求逆解（数值解）
    // double timeout = 0.1;
    // bool found_ik = kinematic_state->setFromIK(joint_model_group, target_state, timeout);
    
    // // 输出逆解（数值解）
    // printf("\n------------IK后解算的机械臂各个关节弧度------------\n");
    // if (found_ik)
    //     print_joint_values(joint_model_group,kinematic_state,joint_values);
    // else
    //     ROS_INFO("Did not find IK solution");
    
    // /*---------------Step4: 执行逆解（数值解）---------------*/
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to << 数值解 >>");
    // move_group.setJointValueTarget(joint_values);

    // // 判断规划是否成功，成功则执行
    // my_execute_plan(move_group,my_plan);

    /*---------------Step5: 求解析解---------------*/
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to << 解析解 >>");

    poseMatrix(poseMat,pi/2,0,0,200,200,350);
    invKinematics(poseMat,is_solution);
    for(int i=0;i<8;i++)
        for(int j=0;j<6;j++)
            jointVal[i].push_back(jointMat[i][j]);

    for(int i=0;i<8;i++){
        if(is_solution[i]==0) continue;
        
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to << 解析解 >>");
        kinematic_state->setJointGroupPositions(joint_model_group, jointVal[i]);
        
        //判断是否超关节转动范围
        bool is_valid = kinematic_state->satisfiesBounds();
        ROS_INFO_STREAM("Current state is " << (is_valid ? "valid" : "not valid"));
        
        //如果valid再执行运动操作
        if(is_valid){
            move_group.setJointValueTarget(jointVal[i]);
            my_execute_plan(move_group,my_plan);
        }
    }
    
    ros::shutdown();
    return 0;
}
