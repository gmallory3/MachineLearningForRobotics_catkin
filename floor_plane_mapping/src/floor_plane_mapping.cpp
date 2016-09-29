#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
        std::string world_frame_;
        double max_range_;
        double tolerance;
        int n_samples; 
        int n_x, n_y;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        pcl::PointCloud<pcl::PointXYZ> worldpc_;
        
		typedef std::list<pcl::PointXYZ> PointList;
		typedef std::vector<PointList> PointListVector;
		typedef std::vector<PointListVector> PointListArray;
		
		
		
    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
			
			PointListArray map_array(n_x,PointListVector(n_y));
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
            size_t best = 0;
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            
            //array[i][j].push_back(point[k]);
			//for (PointList::const_iterator it=array[i][j].begin();it != array[i][j].end(); it++) {
			//	pcl::PointXYZ & P = *it;
			//	double x = P.x, y = P.y, z = P.z;
			//}
            for (int i=0; i<n; i++) {
				int j = floor((worldpc_[i].x + 5)*n_x/10);
				int k = floor((worldpc_[i].y + 5)*n_y/10);
				map_array[j][k].push_back(worldpc_[i]);
			}
			
			
			for (int i=0; i<n_x; i++) {
				for (int j=0; j<n_y; i++) {
					int n_list = map_array[i][j].size();
					
					for (unsigned int i=0;i<(unsigned)n_samples;i++) {
						// Implement RANSAC here. Useful commands:
						// Select a random number in [0,n-1]
						size_t j_1 = std::min((rand() / (double)RAND_MAX) * n_list,(double)n_list-1);
						size_t j_2 = std::min((rand() / (double)RAND_MAX) * n_list,(double)n_list-1);
						size_t j_3 = std::min((rand() / (double)RAND_MAX) * n_list,(double)n_list-1);
				
						// Verify there are no similar points
						if (j_1 == j_2 || j_1 == j_3 || j_2 == j_3) {continue;}
						 
						// Create a 3D point:
						// Eigen::Vector3f P; P << x,y,z;
						// Finding the plane equation by solving AY=B
						Eigen::Matrix3d A; A << map_array[i][j][j_1].x, map_array[i][j][j_1].y, 1, 
												map_array[i][j][j_2].x, map_array[i][j][j_2].y, 1,
												map_array[i][j][j_3]].x, map_array[i][j][j_3].y, 1;
							
						Eigen::Vector3d B; B << map_array[i][j][j_1].z, map_array[i][j][j_2].z, map_array[i][j][j_3].z; 
						
						//Verify that A is inversible
						if (A.determinant() == 0) {continue;}
						
						Eigen::Vector3d Y = A.colPivHouseholderQr().solve(B);
		 
						unsigned int count = 0;
						
						for (PointList::const_iterator it=map_array[i][j].begin();it != map_array[i][j].end(); it++) {
							const pcl::PointXYZ & P = *it;
							double x = P.x, y = P.y, z = P.z;
							double dist = fabs(Y[0] * x + Y[1] * y - z + Y[2])/sqrt(pow(Y[0],2.0)+pow(Y[1],2.0)+1);
							if (dist <= tolerance) {count++;}
						}
						
						if (count > best) {
							best = count;
							for (unsigned int k = 0; k < 3; k++) {
								X[k] = Y[k];
							}
						}

					}
					cv::Mat cvMat(n_x, n_y, DataType<float>::type);
					double cos = 1 / sqrt(pow(X[0],2.0)+pow(X[1],2.0)+1);
					cvMat(i,j) = cos;
				}
			}
        }
        


    public:
        FloorPlaneMapping() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("world_frame",world_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
			nh_.param("n_x",n_x,10);
			nh_.param("n_y",n_y,10);
			
            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneMapping::pc_callback,this);
            //ransac_sub_ = nh_.subscribe("floor_plane_ransac/floor_slope",100,&FloorPlaneMapping::slope_callback,this);
            //ransac_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("ransac_mapping",100);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Mapping");
    FloorPlaneMapping fp;

    ros::spin();
    return 0;
}


