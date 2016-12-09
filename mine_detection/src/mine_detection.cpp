#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
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

#include <Eigen/Core>
#include <Eigen/Dense>


class MineDetection {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;
        tf::StampedTransform transform;

        ros::NodeHandle nh_;
        int cyl_id;
        double mine_radius;
        std::vector<pcl::PointXYZ> PointList;

    protected: // ROS Callbacks

        void pc_callback(const std_msgs::Float32 msg) {
          // Make sure the point cloud is in the base-frame

          //std_msgs::fromROSMsg(*msg, temp);
          listener_.lookupTransform("/world", "VSV/Tool", ros::Time(0), transform);

          pcl::PointXYZ curr_point;
          curr_point.x = transform.getOrigin().x();
          curr_point.y = transform.getOrigin().y();
          curr_point.z = transform.getOrigin().z();

          float mine_radius = 0.5;

          //Publish the cylinder marker
          if (msg.data == 1) {
            ROS_INFO("Detected mine! Position %f, %f, %f", curr_point.x, curr_point.y, curr_point.z);
            PointList.push_back(curr_point);    
            double x, y ,z;
            if (PointList.size() < 2) {return;}
            pcl::PointXYZ point_i = PointList[PointList.size() - 2];
            if (sqrt((point_i.x-curr_point.x)*(point_i.x-curr_point.x) + (point_i.y-curr_point.y)*(point_i.y-curr_point.y) + (point_i.z-curr_point.z)*(point_i.z-curr_point.z)) < mine_radius) {
              x = (point_i.x+curr_point.x)/2;
              y = (point_i.y+curr_point.y)/2;
              z = (point_i.z+curr_point.z)/2;

              visualization_msgs::Marker m;
              //m.header.stamp = msg->header.stamp;
              m.header.frame_id = "/world";
              m.ns = "mines";
              m.id = cyl_id; 
              m.type = visualization_msgs::Marker::CYLINDER;
              m.action = visualization_msgs::Marker::ADD;
              m.pose.position.x = x;
              m.pose.position.y = y;
              m.pose.position.z = z;
              m.pose.orientation.x = 0;
              m.pose.orientation.y = 0;
              m.pose.orientation.z = 0;
              m.pose.orientation.w = 0;
              //tf::quaternionTFToMsg(Q,m.pose.orientation);
              m.scale.x = 0.5;
              m.scale.y = 0.5;
              m.scale.z = 1.5;
              m.color.a = 0.5;
              m.color.r = 1.0;
              m.color.g = 0.0;
              m.color.b = 1.0;

              marker_pub_.publish(m);
            } else {
              visualization_msgs::Marker m;
              //m.header.stamp = msg->header.stamp;
              cyl_id++;
              m.header.frame_id = "/world";
              m.ns = "mines";
              m.id = cyl_id; 
              m.type = visualization_msgs::Marker::CYLINDER;
              m.action = visualization_msgs::Marker::ADD;
              m.pose.position.x = curr_point.x;
              m.pose.position.y = curr_point.y;
              m.pose.position.z = curr_point.z;
              m.pose.orientation.x = 0;
              m.pose.orientation.y = 0;
              m.pose.orientation.z = 0;
              m.pose.orientation.w = 0;
              //tf::quaternionTFToMsg(Q,m.pose.orientation);
              m.scale.x = 0.5;
              m.scale.y = 0.5;
              m.scale.z = 1.5;
              m.color.a = 0.5;
              m.color.r = 1.0;
              m.color.g = 0.0;
              m.color.b = 1.0;

              marker_pub_.publish(m);
            }
          }
        }

    public:
        MineDetection() : nh_("~") {
            // nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("mine_radius",mine_radius,1.0);
            
            ROS_INFO("Searching for mines");

            // Make sure TF is ready
            ros::Duration(0.5).sleep();
            cyl_id = 0;
            scan_sub_ = nh_.subscribe("scans",1,&MineDetection::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("mines",1);
			
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"mine_detection");
    MineDetection cd;

    ros::spin();
    return 0;
}


