#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>

class FloorPlaneRegressionDemining {
    protected:
        ros::Subscriber scan_sub_;
        ros::Subscriber hokuyo_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;
        ros::Publisher twist_pub_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        std::string world_frame_;
        double max_range_;
        double linear_factor_;
        double target_;
        double threshold_;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks


        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {

			
            // Receive the point cloud and convert it to the right format
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
              // In the sensor frame, this point would be inside the camera
              if (d < 1e-2) {
                // Bogus point, ignore
                continue;
              }
              // Measure the point distance in the base frame
              x = lastpc_[i].x;
              y = lastpc_[i].y;
              d = hypot(x,y);
              if (d > max_range_) {
                // too far, ignore
                continue;
              }
              // If we reach this stage, we have an acceptable point, so
              // let's store it
              pidx.push_back(i);
            }
            
            //
            //
            // TODO START
            // 
            // Linear regression: z = a*x + b*y + c
            // Update the code below to use Eigen to find the parameters of the
            // linear regression above. 
            //
            // n is the number of useful point in the point cloud
            n = pidx.size();
            double min = lastpc_[pidx[0]].z;
            for (unsigned int i=0;i<n;i += 100) {
              if (lastpc_[pidx[i]].z < min) {min = lastpc_[pidx[i]].z;}
            }
            
            
            ROS_INFO("min = %f", min);
            geometry_msgs::Twist out;

            if (min < target_ - threshold_){
              ROS_INFO("Up");
              out.linear.z = linear_factor_;
            }
            else if (min > target_ + threshold_){
              ROS_INFO("Down");
              out.linear.z = -linear_factor_;
            } else {
              out.linear.z = 0;
            }
            
            twist_pub_.publish(out);
        }

    public:
        FloorPlaneRegressionDemining() : nh_("~") {

            // TODO START
            // The parameter below described the frame in which the point cloud
            // must be projected to be estimated. You need to understand TF
            // enough to find the correct value to update in the launch file
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("world_frame",world_frame_,std::string("/world"));
            // This parameter defines the maximum range at which we want to
            // consider points. Experiment with the value in the launch file to
            // find something relevant.
            nh_.param("max_range",max_range_,5.0);
            nh_.param("linear_factor_",linear_factor_,2.5);
            nh_.param("threshold_",threshold_,0.05);
            nh_.param("target_",target_,0.4);
            // END OF TODO

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Subscribe to the point cloud and prepare the marker publisher
            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRegressionDemining::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist",1);
        }

};

int main(int argc, char * argv[]) 
{	

    ros::init(argc,argv,"floor_plane_regression_demining");
    FloorPlaneRegressionDemining fp;

    ros::spin();
    return 0;

}


