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


class FloorPlaneMapping {
    protected:
        ros::Subscriber scan_sub_;
        ros::Subscriber ransac_sub_;
        ros::Publisher ransac_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        double tolerance;
        int n_samples; 
        int n_x, n_y;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
		typedef std::list<pcl::PointXYZ> PointList;
		typedef std::vector<PointList> PointListVector;
		typedef std::vector<PointListVector>PointListArray;
			
		PointListArray array(n_x,PointListVector(n_y));
		
    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
			
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
            
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(world_frame_,msg->header.stamp, temp, msg->header.frame_id, worldpc_, listener_);

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
            
            n = pidx.size();
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            
            //array[i][j].push_back(point[k]);
			//for (PointList::const_iterator it=array[i][j].begin();it != array[i][j].end(); it++) {
			//	pcl::PointXYZ & P = *it;
			//	double x = P.x, y = P.y, z = P.z;
			//}
            for (int i=0; i<n; i++) {
					PointListArray(floor((worldpc_[i].x + 5)*n_x/10),floor((worldpc_[i].y + 5)*n_y/10)).push_back(worldpc_[i]);
			}

            
        }
        
        void slope_callback(const std_msgs::Float64 slope) {
			
		}
        


    public:
        FloorPlaneMapping() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneMapping::pc_callback,this);
            ransac_sub_ = nh_.subscribe("floor_plane_ransac/floor_slope",100,&FloorPlaneMapping::slope_callback,this);
            ransac_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("ransac_mapping",100);
            array.assign(n_x,PointListVector(n_y));
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_mapping");
    FloorPlaneMapping fp;

    ros::spin();
    return 0;
}


