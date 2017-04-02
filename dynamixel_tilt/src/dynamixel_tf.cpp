 #include <ros/ros.h>
 #include <tf/transform_broadcaster.h>
 #include <dynamixel_tilt/dynamixel_tilt.h>
 #include <dynamixel_tilt/send_angle.h>
 #include <msg/motorPosition.h>

 std::string dynamixel_motor;

 int32_t value_of_0_radian_position_      = 2048;
 int32_t value_of_min_radian_position_    = 0;
 int32_t value_of_max_radian_position_    = 4095;
 int32_t min_radian_                      = -3.14159265;
 int32_t max_radian_                      =  3.14159265;

double convertValue2Radian(int32_t value);

 void poseCallback(const msg::motorPosition::ConstPtr& pub)
 {
   static tf::TransformBroadcaster br;
   tf::Transform transform;
   tf::Quaternion q;

   convertValue2Radian(pub->motor_present_position);

   transform.setOrigin( tf::Vector3(0, 0, 0.08) );
   q.setRPY(0, convertValue2Radian(pub->motor_present_position), 0);
   transform.setRotation(q);

   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dynamixel_angle"));

   transform.setOrigin( tf::Vector3(0, 0, 0.11) );
   q.setRPY(0, convertValue2Radian(pub->motor_present_position), 0);
   transform.setRotation(q);

   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "laser_angle"));

   ROS_INFO("%d", pub->motor_present_position);
 }

 double convertValue2Radian(int32_t value)
 {
   double radian = 0.0;
   if (value > value_of_0_radian_position_)
   {
     if (max_radian_ <= 0)
       return max_radian_;

     radian = (double) (value - value_of_0_radian_position_) * max_radian_
                / (double) (value_of_max_radian_position_ - value_of_0_radian_position_);
   }
   else if (value < value_of_0_radian_position_)
   {
     if (min_radian_ >= 0)
       return min_radian_;

     radian = (double) (value - value_of_0_radian_position_) * min_radian_
                / (double) (value_of_min_radian_position_ - value_of_0_radian_position_);
   }

   if (radian > max_radian_)
     return max_radian_;
   else if (radian < min_radian_)
     return min_radian_;

   return radian;
 }

 int main(int argc, char** argv)
 {
    ros::init(argc, argv, "dynamixel_tf");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("motorPosition", 10, &poseCallback);

    ros::spin();
    return 0;
  };
