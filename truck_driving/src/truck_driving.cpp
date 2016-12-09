#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <geometry_msgs/Twist.h>

#include <Eigen/Core>
#include <Eigen/Dense>


class TruckDriving {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher twist_pub_;
        tf::TransformListener listener_;
        double target_ratio_;
        double threshold_;
        double linear_vel_;
        double twist_factor_;
        double height_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
          pcl::PointCloud<pcl::PointXYZ> temp;
          pcl::fromROSMsg(*msg, temp);
          // Make sure the point cloud is in the base-frame
          listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
          pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

          unsigned int n = temp.size();
          std::vector<size_t> pidx;
          // First count the useful points
          for (unsigned int i=0;i<n;i++) {
            if (fabs(lastpc_[i].z) > max_range_) {
              continue;
            }
            pidx.push_back(i);
          }

          double lake_points = 0;
          n = pidx.size();
          ROS_INFO("n = %d", n);
          //for (unsigned int i=0;i<n;i++) {
          //  ROS_INFO("height = %f", lastpc_[pidx[i]].z);
          //}

          for (unsigned int i=0;i<n;i++) {
            if (lastpc_[pidx[i]].z < height_) {
              lake_points++;
            }
          }
          
          double ratio = lake_points/n;
          ROS_INFO("Ratio lake = %f", ratio); 
          geometry_msgs::Twist out;
          out.linear.x = linear_vel_;
          out.angular.z = 0;
          if (ratio < target_ratio_ - threshold_) {
            out.angular.z = twist_factor_;
          }
          else if (ratio > target_ratio_ + threshold_) {
            out.angular.z = -twist_factor_;
          }

          twist_pub_.publish(out);
       }

    public:
        TruckDriving() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("target_ratio",target_ratio_,1.0);
            nh_.param("threshold",threshold_,1.0);
            nh_.param("linear_vel",linear_vel_,1.0);
            nh_.param("twist_factor",twist_factor_,1.0);
            nh_.param("max_range",max_range_,5.0);
            nh_.param("height",height_,5.0);
            
            ROS_INFO("Truck steering ON");

            // Make sure TF is ready
            ros::Duration(0.5).sleep();
            scan_sub_ = nh_.subscribe("scans",1,&TruckDriving::pc_callback,this);
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist",1);
			
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"truck_driving");
    TruckDriving cd;

    ros::spin();
    return 0;
}


