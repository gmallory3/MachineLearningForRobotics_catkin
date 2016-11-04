#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
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


class CylindersDetection {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;
        ros::Publisher slope_pub_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        double tolerance;
        int n_samples;
        int min_best;
        int cyl_id;

        double best_radius;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            //
            unsigned int n = temp.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }
            
            //
            // BEGIN TODO
            // Finding circles: 
            n = pidx.size();
            size_t best = 0;
            double X[3] = {0,0,0};
              
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            
            
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {
              // Select a random number in [0,n-1]
              size_t j_1 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
              size_t j_2 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
              size_t j_3 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				
              // Verify there are no similar points
              if (j_1 == j_2 || j_1 == j_3 || j_2 == j_3) {continue;}
              
              // Check if the 3 points are not to close
              double dist1 = sqrt(pow(lastpc_[pidx[j_1]].x - lastpc_[pidx[j_2]].x,2.0) + pow(lastpc_[pidx[j_1]].y - lastpc_[pidx[j_2]].y,2.0));
              double dist2 = sqrt(pow(lastpc_[pidx[j_2]].x - lastpc_[pidx[j_3]].x,2.0) + pow(lastpc_[pidx[j_2]].y - lastpc_[pidx[j_3]].y,2.0));
              double dist3 = sqrt(pow(lastpc_[pidx[j_1]].x - lastpc_[pidx[j_3]].x,2.0) + pow(lastpc_[pidx[j_1]].y - lastpc_[pidx[j_3]].y,2.0));
              if (dist1 < 0.05 || dist2 < 0.05 || dist3 < 0.05) {continue;}	
              
              //Finding circle's center and radius with 3 points
              Eigen::Vector3d C; 
              
              float ma = (lastpc_[pidx[j_2]].y - lastpc_[pidx[j_1]].y)/(lastpc_[pidx[j_2]].x-lastpc_[pidx[j_1]].x);
              float mb = (lastpc_[pidx[j_3]].y - lastpc_[pidx[j_2]].y)/(lastpc_[pidx[j_3]].x-lastpc_[pidx[j_2]].x);

              C[0] = (ma*mb*(lastpc_[pidx[j_1]].y-lastpc_[pidx[j_3]].y)+mb*(lastpc_[pidx[j_1]].x-lastpc_[pidx[j_2]].x)-ma*(lastpc_[pidx[j_2]].x+lastpc_[pidx[j_3]].x))/(2*(mb-ma));
              C[1] = -(1/ma)*(C[0]-(lastpc_[pidx[j_1]].x+lastpc_[pidx[j_2]].x)/2)+(lastpc_[pidx[j_1]].y+lastpc_[pidx[j_2]].y)/2;
              C[2] = 0;
			
              double radius = sqrt(pow(lastpc_[pidx[j_1]].x-C[0],2.0)+pow(lastpc_[pidx[j_1]].y-C[1],2.0));
				
              unsigned int count = 0;
				
              for (unsigned j = 0; j < n; j++) {
                Eigen::Vector3f W; W << lastpc_[pidx[j]].x, lastpc_[pidx[j]].y, lastpc_[pidx[j]].z;
                double dist = fabs(sqrt(pow(W[0] - C[0], 2.0)+pow(W[1] - C[1], 2.0)) - radius);
                if (dist <= tolerance) {count++;}
              }
				
              if (count > best) {
                best = count;
                for (unsigned int k = 0; k < 3; k++) {
                  X[k] = C[k];
                  best_radius = radius;
                }
              } 
            }
            // END OF TODO
			
            //Publish the cylinder marker
            if (best > (unsigned)min_best && best_radius < 0.4) {
              ROS_INFO("Extracted circle: (x - %.2f)^2 + (y - %.2f)^2 = %.2f^2",X[0],X[1],best_radius);			

              visualization_msgs::Marker m;
              m.header.stamp = msg->header.stamp;
              m.header.frame_id = base_frame_;
              m.ns = "cylinders";
              m.id = cyl_id++; 
              m.type = visualization_msgs::Marker::CYLINDER;
              m.action = visualization_msgs::Marker::ADD;
              m.pose.position.x = X[0];
              m.pose.position.y = X[1];
              m.pose.position.z = X[2];
              m.pose.orientation.x = 0;
              m.pose.orientation.y = 0;
              m.pose.orientation.z = 0;
              m.pose.orientation.w = 0;
              //tf::quaternionTFToMsg(Q,m.pose.orientation);
              m.scale.x = 2*best_radius;
              m.scale.y = 2*best_radius;
              m.scale.z = 1.5;
              m.color.a = 0.5;
              m.color.r = 1.0;
              m.color.g = 0.0;
              m.color.b = 1.0;

              marker_pub_.publish(m);
            }
        }

    public:
        CylindersDetection() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);
            nh_.param("min_best",min_best,7000);
            ROS_INFO("Searching for cylinders");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();
            cyl_id = 0;
            scan_sub_ = nh_.subscribe("scans",1,&CylindersDetection::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("cylinders",1);
			
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"cylinders_detection");
    CylindersDetection cd;

    ros::spin();
    return 0;
}


