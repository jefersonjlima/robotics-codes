#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

class Follow
{
  private:
  // master
    ros::Subscriber masterGetPose_;
    geometry_msgs::Pose2D masterPose_;
  // follower
    ros::Subscriber followerGetPose_;
    geometry_msgs::Pose2D followerPose_;
  // control
    ros::Publisher followerControl_;

  public:
    Follow(ros::NodeHandle nh)
    {
      masterGetPose_   = nh.subscribe("tb3_0/odom", 100, &Follow::getMasterPose,   this);
      followerGetPose_ = nh.subscribe("tb3_1/odom", 100, &Follow::getFollowerPose, this);
      followerControl_ = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel", 100);
    }

    void getMasterPose(const nav_msgs::Odometry::ConstPtr msg)
    {
      masterPose_.x = msg->pose.pose.position.x;
      masterPose_.y = msg->pose.pose.position.y;
      tf::Quaternion q(
	  	msg->pose.pose.orientation.x,
	  	msg->pose.pose.orientation.y,
	  	msg->pose.pose.orientation.z,
	  	msg->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      masterPose_.theta = yaw;
//      ROS_INFO("[Master] x:%.2lf, y: %.2lf, theta: %.2lf", masterPose_.x, masterPose_.y, masterPose_.theta);
    }

    void getFollowerPose(const nav_msgs::Odometry::ConstPtr msg)
    {
      followerPose_.x = msg->pose.pose.position.x;
      followerPose_.y = msg->pose.pose.position.y;
      tf::Quaternion q(
	  	msg->pose.pose.orientation.x,
	  	msg->pose.pose.orientation.y,
	  	msg->pose.pose.orientation.z,
	  	msg->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      followerPose_.theta = yaw;
//      ROS_INFO("[Follower] x:%.2lf, y: %.2lf, theta: %.2lf", followerPose_.x, followerPose_.y, followerPose_.theta);
    }

    void moveToTarget()
    {
      geometry_msgs::Twist command;
      double error, k_pl, k_pa, v_t, maxLinearVel;
      double theta_r, maxAngularVel, omega;

      k_pl = 0.3; k_pa = 0.3;
      maxLinearVel = 0.22;
      maxAngularVel = 2.84;
      // ---------------------------------------------------------------------------
      // Linear control
      error = sqrt( pow(masterPose_.x - followerPose_.x ,2) + 
                        pow(masterPose_.y - followerPose_.y ,2));
      v_t = k_pl * error;

      if (v_t > maxLinearVel)
        v_t = maxLinearVel;
      
      if (error < 0.25)
        command.linear.x = 0.0;
      else
        command.linear.x = v_t;
      
      // ---------------------------------------------------------------------------
      // Angular control

//      theta_r =
//      omega =
      // ...

      command.angular.z = omega;

      // ---------------------------------------------------------------------------
      followerControl_.publish(command);
      ROS_INFO("[Follower] Linear Control %.2f, Angular Control: %.2f", v_t, omega);
    }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "tb3_follow_node");
  ros::NodeHandle nh;
  Follow follower(nh);
  ros::Rate loop_rate(1);

  while(ros::ok())
  {
    ros::spinOnce();
    follower.moveToTarget();
    loop_rate.sleep();
  }
  return 0;
}
