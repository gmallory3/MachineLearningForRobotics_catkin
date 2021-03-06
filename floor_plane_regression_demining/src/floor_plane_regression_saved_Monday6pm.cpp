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
        double threshold_;
        double thres_min_;
        double thres_max_;
        double hk_threshold_;
        bool hokuyo_up_;
        bool handler_;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks

        void hk_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            ROS_INFO("hk_callback"); 
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(world_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(world_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            unsigned int n = temp.size();
            for (unsigned int i=0;i<n;i++) {
              if (temp[i].z > hk_threshold_) {hokuyo_up_ = 1;} 
            }

            ROS_INFO("%d", hokuyo_up_);
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {

			
            // Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
            
            //
            unsigned int n = temp.size();
            ROS_INFO("%d", n);
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
            for (unsigned int i=0;i<n;i++) {
              if (lastpc_[pidx[i]].z < min) {min = lastpc_[pidx[i]].z;}
            }
            ROS_INFO("min = %f", min);
            ROS_INFO("hokuyo up: %d handler: %d", hokuyo_up_, handler_);
            // situation 1: no obstacle, no sensor. handler = 0, hokuyo = 0
            // situation 2: no obstacle, sensor. handler = 0, hokuyo = 1
            // situation 2.5: no obst, no sensor. handler = 0. hokuyo = 1
            // situation 3: obstacle, no sensor. handler = 1, hokuyo = 0
            // situation 4: no obstacle, sensor. handler = 0, hokuyo = 0
            geometry_msgs::Twist out;
            if (min > threshold_ && hokuyo_up_ == 0) {
              out.linear.z = linear_factor_;
              ROS_INFO("FIX");
            }
            else if (min > threshold_ && hokuyo_up_ == 1) {
              out.linear.z = -linear_factor_;
              handler_ = 1;
              ROS_INFO("FIX 2 ");
            }
            else if (min <= thres_min_) {
              out.linear.z = linear_factor_;
              ROS_INFO("UP");
            } 
            else if (min > thres_min_ && min < thres_max_) {
              out.linear.z = 0;
              ROS_INFO("PUT");
            } 
            else if (min >= thres_max_) {
              out.linear.z = -linear_factor_;
              if (handler_==1) { 
                handler_ = 0;
                hokuyo_up_ = 0;
              }
              ROS_INFO("DOWN");
            }
            
            twist_pub_.publish(out);
            // Eigen is a matrix library. The line below create a 3x3 matrix A,
            // and a 3x1 vector B
            /*Eigen::MatrixXf A(n,3);
            Eigen::VectorXf B(n);
            for (unsigned int i=0;i<n;i++) {
                // Assign x,y,z to the coordinates of the point we are
                // considering.
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;
                double z = lastpc_[pidx[i]].z;
                
                A(i,0) = x;
                A(i,1) = y;
                A(i,2) = 1;

                B(i) = z;
                
            }
            Eigen::MatrixXf X = A.colPivHouseholderQr().solve(B);
            
            ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X(0),X(1),X(2));
            
            double cos = 1 / sqrt(X(0)*X(0)+X(1)*X(1)+1);
            ROS_INFO("cos: %f", cos); 

            geometry_msgs::Twist out;
            //out.linear.z = linear_factor_ * (cos - 0.5);
            twist_pub_.publish(out);

            // Now build an orientation vector to display a marker in rviz
            // First we build a basis of the plane normal to its normal vector
            Eigen::Vector3f O,u,v,w;
            w << X(0), X(1), -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X(0)+0.0*X(1)+X(2);
            u << 2.0, 0.0, 2.0*X(0)+0.0*X(1)+X(2);
            u -= O;
            u /= u.norm();
            v = w.cross(u);
            // Then we build a rotation matrix out of it
            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            // And convert it to a quaternion
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
            // Finally publish the marker
            marker_pub_.publish(m);*/
         
       
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
            nh_.param("threshold_",threshold_,0.5);
            nh_.param("thres_min_",thres_min_,0.2);
            nh_.param("thres_max_",thres_max_,0.3);
            nh_.param("hk_threshold_",hk_threshold_,0.3);
            // END OF TODO
            hokuyo_up_ = 0;
            handler_ = 0;

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Subscribe to the point cloud and prepare the marker publisher
            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRegressionDemining::pc_callback,this);
            hokuyo_sub_ = nh_.subscribe("hokuyo",1,&FloorPlaneRegressionDemining::hk_callback,this);
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


