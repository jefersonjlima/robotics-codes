#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#ifdef USE_MATPLOT
 #include <matplot/matplot.h>
 using namespace matplot;
#endif

#define SHOW_LOGS

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

    void getMasterPose(const sensor_msgs::LaserScan::ConstPtr msg)
    {
      auto sensor = std::vector<float>(msg->ranges.size());

      // replace inf to 3.5
      std::transform(
	  msg->ranges.begin(), msg->ranges.end(), sensor.begin(),
	  [](float it) { return (it > 360)? 3.5 : it; });

#ifdef SHOW_LOGS
      ROS_INFO("Sensor Data:");
      for(auto it = std::begin(sensor); it != std::end(sensor); ++it)
      {
	std::cout << *it << ", ";
      }
      std::cout << std::endl;
#endif

      //
      // your code here
      //
      masterPose_.x = 0;
      masterPose_.y = 0;

#ifdef USE_MATPLOT
      std::vector<double> theta = linspace(0, 2 * pi, 360);
      polarplot(theta, sensor);
#endif

#ifdef SHOW_LOGS
      ROS_INFO("[Master] x:%.2lf, y: %.2lf, theta: %.2lf", masterPose_.x, masterPose_.y, masterPose_.theta);
#endif
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
#ifdef SHOW_LOGS
      ROS_INFO("[Follower] x:%.2lf, y: %.2lf, theta: %.2lf", followerPose_.x, followerPose_.y, followerPose_.theta);
#endif
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
      // tip: use atan2 here
      // 
      // theta_r =
      // omega =

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
