/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */
/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).
   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */
#include <string>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <kdl/frames.hpp>
#include <raiscar_msgs/ControlReport.h>
#include "raiscar_msgs/ControlCommand.h"
#include "std_msgs/Bool.h"
#include "controller.h"
using std::string;
class PurePursuit: public controller
{
public:
    //! Constructor
    PurePursuit();

    std::vector<std::double_t> ycol;
    std::vector<std::double_t> ctecol;
    double cte=0;
    double dx,dy,rx,ry;
    std::vector<std::int_fast64_t> idxcol;

    //! Run the controller.
    void run();

    //! Function to compute euclidean distance
    double distance(std::vector<tf2::Transform> pt1, tf2::Stamped<tf2::Transform> pt2, unsigned a)
    {
        return sqrt(pow(pt1.at(a).getOrigin().x() - pt2.getOrigin().x(),2) + pow(pt1.at(a).getOrigin().y() - pt2.getOrigin().y(),2) + pow(pt1.at(a).getOrigin().z() - pt2.getOrigin().z(),2));
    }

    //! Function to compute pose of the target w.r.t robot
    KDL::Frame transformToBaseLink(const std::vector<tf2::Transform> pose,
                                   const geometry_msgs::TransformStamped& map,
                                   unsigned a)
    {
        // Pose in global (map) frame
        KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.at(a).getRotation().x(),
                                                        pose.at(a).getRotation().y(),
                                                        pose.at(a).getRotation().z(),
                                                        pose.at(a).getRotation().w()),
                              KDL::Vector(pose.at(a).getOrigin().x(),
                                          pose.at(a).getOrigin().y(),
                                          pose.at(a).getOrigin().z()));
        // Robot (base_link) in global (map) frame
        KDL::Frame F_map_tf(KDL::Rotation::Quaternion(map.transform.rotation.x,
                                                      map.transform.rotation.y,
                                                      map.transform.rotation.z,
                                                      map.transform.rotation.w),
                            KDL::Vector(map.transform.translation.x,
                                        map.transform.translation.y,
                                        map.transform.translation.z));
        return F_map_tf.Inverse()*F_map_pose;
    }
private:
    void controller_step(nav_msgs::Odometry odom);
    double ld_dist_;
};
PurePursuit::PurePursuit()
{
    // Get parameters from the parameter server
    nh_private_.param<double>("lookahead_dist", ld_dist_, 0.76);
    std::cout << "Pure Pursuit controller started..." << std::endl;
    throttle_limit_ = 0.25;
    des_v_ = 0.2;
}
/*
 * Implement your controller here! The function gets called each time a new odometry is incoming.
 * The path to follow is saved in the variable "path_". Once you calculated the new control outputs you can send it with
 * the pub_acker_ publisher.
 */
void PurePursuit::controller_step(nav_msgs::Odometry odom)
{
    // Code blocks that could be useful:
    // The following code block could receive the current pose (saved in map_t_fa)
    geometry_msgs::TransformStamped tf_msg;
    geometry_msgs::TransformStamped front_axis_tf_msg;
    tf2::Stamped<tf2::Transform> map_t_fa;
    try {
        tf_msg = tf_buffer_.lookupTransform(map_frame_id_, rear_axis_frame_id_, ros::Time(0));
        front_axis_tf_msg = tf_buffer_.lookupTransform(map_frame_id_, front_axis_frame_id_, ros::Time(0));
    }catch (tf2::TransformException &ex)
    {
        ROS_WARN_STREAM(ex.what());
    }
    tf2::convert(tf_msg, map_t_fa);

    double path_size = path_.size();

    if(path_.empty() == false)
    {
        for (; idx_ < path_size-1 ; idx_++)
        {
            if (distance(path_, map_t_fa, idx_) > ld_dist_ )
            {
                KDL::Frame F_bl_ld = transformToBaseLink(path_, front_axis_tf_msg, idx_);
                target_p_.transform.translation.x = F_bl_ld.p.x();
                target_p_.transform.translation.y = F_bl_ld.p.y();
                target_p_.transform.translation.z = F_bl_ld.p.z();
                F_bl_ld.M.GetQuaternion(target_p_.transform.rotation.x,
                                        target_p_.transform.rotation.y,
                                        target_p_.transform.rotation.z,
                                        target_p_.transform.rotation.w);
                break;
            }
        }
        if(!path_.empty() && idx_ >= path_size-1)
        {
            KDL::Frame F_bl_end = transformToBaseLink(path_, tf_msg, idx_-5);
            if (fabs(F_bl_end.p.x()) <= pos_tol_)
            {
                // We have reached the goal
                goal_reached_ = true;
                sendGoalMsg(true);
                // Reset the path
                path_ = std::vector<tf2::Transform>();
            }
            else
            {
                double roll, pitch, yaw;
                F_bl_end.M.GetRPY(roll, pitch, yaw);
                double k_end = tan(yaw); // Slope of line defined by the last path pose
                double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
                double a = 1 + k_end * k_end;
                double b = 2 * l_end;
                double c = l_end * l_end - ld_dist_ * ld_dist_;
                double D = sqrt(b*b - 4*a*c);
                double x_ld = (-b + copysign(D,vmax_)) / (2*a);
                double y_ld = k_end * x_ld + l_end;
                target_p_.transform.translation.x = x_ld;
                target_p_.transform.translation.y = y_ld;
                target_p_.transform.translation.z = F_bl_end.p.z();
                F_bl_end.M.GetQuaternion(target_p_.transform.rotation.x,
                                         target_p_.transform.rotation.y,
                                         target_p_.transform.rotation.z,
                                         target_p_.transform.rotation.w);
            }
        }
        if(!goal_reached_)
        {
            KDL::Frame F_bl_ = transformToBaseLink(path_, tf_msg, idx_);
            double roll, pitch, yaw,r1,p1,y1;
            F_bl_.M.GetRPY(roll, pitch, yaw);
            double path_angle;
            if(idx_<path_size-1) {
                path_angle = atan2(path_.at(idx_ + 1).getOrigin().y() - path_.at(idx_).getOrigin().y(),
                                   path_.at(idx_ + 1).getOrigin().x() - path_.at(idx_).getOrigin().x()) * 180 /
                             M_PI;
            }

//            The following code part can be used for debugging
//
//            std::cout << "path =" << path_angle << std::endl;
//            std::cout<< "yaw=" << (yaw*180/M_PI) << std::endl;
//            std::cout << "heading_error" << fabs(path_angle) - fabs(yaw*180/M_PI) << std::endl;
//            std::cout << target_p_ << std::endl ;
//            std::cout << distance(path_, map_t_fa, idx_) <<" idx = "<< idx_ << std::endl;

            double he = fabs(path_angle) - fabs(yaw*180/M_PI);

            ycol.push_back(he);
            idxcol.push_back(idx_);

            if(idx_>0)
            {
                dx = target_p_.transform.translation.x - path_.at(idx_-1).getOrigin().x();
                dy = target_p_.transform.translation.y - path_.at(idx_-1).getOrigin().y();
                rx = tf_msg.transform.translation.x - path_.at(idx_-1).getOrigin().x();
                ry = tf_msg.transform.translation.y - path_.at(idx_-1).getOrigin().y();
                cte = (ry*dx - rx*dy)/(dx*dx - dy*dy);
                ctecol.push_back(cte);
            }
            else cte = 0;

            //!PID Controller
            float pid_vel_out = 0.0;
            float des_vel = des_v_;
            if (des_vel >= 0) {
                pid_vel_out = vel_pid.step((des_vel - odom.twist.twist.linear.x), ros::Time::now());
            } else {
                pid_vel_out = des_vel;
            }
            double yt = target_p_.transform.translation.y;
            double ld_2 = ld_dist_ * ld_dist_;
            double delta = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );
            cmd_control_.steering = delta /(70.0 * M_PI / 180.0); //  DUMMY_STEERING_ANGLE should be a value in radiant
            cmd_control_.throttle = pid_vel_out;
            cmd_control_.throttle_mode = 0;
            cmd_control_.throttle = std::min(cmd_control_.throttle, throttle_limit_);
            cmd_control_.throttle = std::max(std::min((double)cmd_control_.throttle, 1.0), 0.0);
            pub_acker_.publish(cmd_control_);
        }
        else
        {
            //!Dump data to .txt file
            std::ofstream file("/home/freicar/freicar_ws/src/freicar_ss21_exercises/01-01-control-exercise/freicar_control/data/final.txt",std::ios::out);
            target_p_.transform = geometry_msgs::Transform();
            target_p_.transform.rotation.w = 1.0;
            cmd_control_.steering = 0;
            cmd_control_.throttle = 0;
            cmd_control_.brake = 1;
            cmd_control_.hand_brake = 1;
            cmd_control_.throttle_mode = 0;
            cmd_control_.throttle = std::min(cmd_control_.throttle, throttle_limit_);
            cmd_control_.throttle = std::max(std::min((double)cmd_control_.throttle, 1.0), 0.0);
            pub_acker_.publish(cmd_control_);
            unsigned temp=idxcol.at(0);
            for(int i=1;i<ycol.size();i++)
            {
                if(idxcol.at(i)==temp)
                    continue;
                else {
                    file << "idx=" << idxcol.at(i) << " ; yaw=" << ycol.at(i) << " ; cte=" << ctecol.at(i)  << std::endl;
                    temp=idxcol.at(i);
                }
            }
            file.close();
        }
    }
    else
    {
        std::cout << "Path not received. Run pub_path.py" << std::endl;
    }
}
void PurePursuit::run()
{
    ros::spin();
}
int main(int argc, char**argv)
{
    ros::init(argc, argv, "pure_pursuit_controller");
    PurePursuit controller;
    controller.run();
    return 0;
}