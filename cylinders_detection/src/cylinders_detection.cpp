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


class FloorPlaneRansac {
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
            // Finding planes: z = a*x + b*y + c
            // Remember to use the n_samples and the tolerance variable
            n = pidx.size();
            size_t best = 0;
            double X[3] = {0,0,0};
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            
            
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {
                // Implement RANSAC here. Useful commands:
                // Select a random number in [0,n-1]
				size_t j_1 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				size_t j_2 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				size_t j_3 = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				
				// Verify there are no similar points
				if (j_1 == j_2 || j_1 == j_3 || j_2 == j_3) {continue;}
				 
				// Create a 3D point:
				// Eigen::Vector3f P; P << x,y,z;
				// Finding the plane equation by solving AY=B
				Eigen::Matrix3d A; A << lastpc_[pidx[j_1]].x, lastpc_[pidx[j_1]].y, 1, 
										lastpc_[pidx[j_2]].x, lastpc_[pidx[j_2]].y, 1,
										lastpc_[pidx[j_3]].x, lastpc_[pidx[j_3]].y, 1;
					
				Eigen::Vector3d B; B << lastpc_[pidx[j_1]].z, lastpc_[pidx[j_2]].z, lastpc_[pidx[j_3]].z; 
				
				//Verify that A is inversible
				if (A.determinant() == 0) {continue;}
				
				Eigen::Vector3d Y = A.colPivHouseholderQr().solve(B);
 
				unsigned int count = 0;
				
				for (unsigned j = 0; j < n; j++) {
					Eigen::Vector3f W; W << lastpc_[pidx[j]].x, lastpc_[pidx[j]].y, lastpc_[pidx[j]].z;
					double dist = fabs(Y[0] * W[0] + Y[1] * W[1] - W[2] + Y[2])/sqrt(pow(Y[0],2.0)+pow(Y[1],2.0)+1);
					if (dist <= tolerance) {count++;}
				}
				
				if (count > best) {
					best = count;
					for (unsigned int k = 0; k < 3; k++) {
						X[k] = Y[k];
					}
				} 

            }
            // At the end, make sure to store the best plane estimate in X
            // X = {a,b,c}. This will be used for display
			
            // END OF TODO
            

			//Publishing angle between normal vector and z.
			std_msgs::Float64 cos_msg;
			cos_msg.data = 1 / sqrt(pow(X[0],2.0)+pow(X[1],2.0)+1);
			ROS_INFO("angle: %.2f", cos_msg.data);
			slope_pub_.publish(cos_msg);
			
			
			//Publish the floor plane
            ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X[0],X[1],X[2]);			
			
        
            Eigen::Vector3f O,u,v,w;
            w << X[0], X[1], -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf::Quaternion Q;
            R.getRotation(Q);
            
            visualization_msgs::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = base_frame_;
            m.ns = "floor_plane";
            m.id = 1;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = O(0);
            m.pose.position.y = O(1);
            m.pose.position.z = O(2);
            tf::quaternionTFToMsg(Q,m.pose.orientation);
            m.scale.x = 1.0;
            m.scale.y = 1.0;
            m.scale.z = 0.01;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;

            marker_pub_.publish(m);
            
        }

    public:
        FloorPlaneRansac() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRansac::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);
			slope_pub_ = nh_.advertise<std_msgs::Float64>("floor_slope",100);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Ransac");
    FloorPlaneRansac fp;

    ros::spin();
    return 0;
}


