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
      ros::ServiceServer ransac_client_;
      tf::TransformListener listener_;

      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;
      std::string base_frame_;
      std::string world_frame_;
      double max_range_;
      double tolerance;
      int n_samples; 
      int n_x, n_y;
      double epsilon;
      cv::Mat_<uint8_t> cvMap;
      cv::Mat_<uint8_t> cvBool;
      cv::Mat_<float> cvProb;
      cv::Mat_<float> cvSlope;

      image_transport::Publisher im_pub;


      pcl::PointCloud<pcl::PointXYZ> lastpc_;
      pcl::PointCloud<pcl::PointXYZ> worldpc_;

      typedef std::vector<pcl::PointXYZ> PointList;
      typedef std::vector<PointList> PointListVector;
      typedef std::vector<PointListVector> PointListArray;
      PointListArray map_array;
		
    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
          //PointListArray map_array(n_x,PointListVector(n_y));
          pcl::PointCloud<pcl::PointXYZ> temp;
          pcl::fromROSMsg(*msg, temp);

          cvBool = 0;
          map_array.assign(n_x,PointListVector(n_y));
          // Make sure the point cloud is in the base-frame

          // wait for the transform from arg 1 to arg 2. Need the one at time arg3. Only wait for arg4 duration.
          listener_.waitForTransform(base_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));

          // target frame, target time,   ptc in,    fixed frame,       ptc out,   transform listener 
          pcl_ros::transformPointCloud(base_frame_, msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

          listener_.waitForTransform(world_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
          pcl_ros::transformPointCloud(world_frame_, msg->header.stamp, temp, msg->header.frame_id, worldpc_, listener_);

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
			
			// push point cloud points into our matrix at the appropriate index. 
            for (unsigned int i=0; i<n; i++) {
              int j = floor((worldpc_[pidx[i]].x + 5)*n_x/10);
              if (j>=n_x || j<0) {continue;}
              int k = floor((worldpc_[pidx[i]].y + 5)*n_y/10);
              if (k>=n_y || k<0) {continue;}
              map_array[j][k].push_back(worldpc_[pidx[i]]);
              if (map_array[j][k].size() != 0) {cvBool(j,k) = 1;}
            }
			
            for (int i=0; i<n_x; i++) {
              for (int j=0; j<n_y; j++) {
                //double cos;
                double X[3]={0,0,0};				
                size_t best = 0;
                int n_list = map_array[i][j].size();
                for (unsigned int k=0; k<(unsigned)n_samples; k++) {
                  // Select a random number in [0,n-1]
                  size_t j_1 = std::min((rand() / (double)RAND_MAX) * n_list,(double)n_list-1);
                  size_t j_2 = std::min((rand() / (double)RAND_MAX) * n_list,(double)n_list-1);
                  size_t j_3 = std::min((rand() / (double)RAND_MAX) * n_list,(double)n_list-1);

                  // Verifying there are no similar points
                  if (j_1 == j_2 || j_1 == j_3 || j_2 == j_3) {continue;}

                  // Finding the plane equation by solving AY=B
                  Eigen::Matrix3d A; A << map_array[i][j][j_1].x, map_array[i][j][j_1].y, 1, 
                    map_array[i][j][j_2].x, map_array[i][j][j_2].y, 1,
                    map_array[i][j][j_3].x, map_array[i][j][j_3].y, 1;

                  Eigen::Vector3d B; B << map_array[i][j][j_1].z, map_array[i][j][j_2].z, map_array[i][j][j_3].z; 
                  //Verify that A is inversible
                  if (A.determinant() == 0) {continue;}

                  Eigen::Vector3d Y = A.colPivHouseholderQr().solve(B);

                  unsigned int count = 0;

                  for (PointList::const_iterator it=map_array[i][j].begin();it != map_array[i][j].end(); it++) {
                    const pcl::PointXYZ & P = *it;

                    double x = P.x, y = P.y, z = P.z;
                    double dist = fabs(Y[0] * x + Y[1] * y - z + Y[2])/sqrt(Y[0]*Y[0]+Y[1]*Y[1]+1);
                    if (dist <= tolerance) {count++;}
                  }

                  if (count > best) {
                    best = count;
                    for (unsigned int q = 0; q < 3; q++) {
                      X[q] = Y[q];
                    }
                  }
                }
                cvSlope(i,j) = 1 / sqrt(X[0]*X[0]+X[1]*X[1]+1);
                double P0 = cvProb(i,j);
                double P1 = 1 - cvProb(i,j);
                //if (n_list == 0) {cvMap(i,j) = 0;}
                if(cvSlope(i,j) >= 0.75 && cvBool(i,j) == 1) {
                  P0 = (1-epsilon)*P0;
                  P1 = epsilon*P1;
                  double P0n = P0/(P0+P1);
                  cvProb(i,j) = P0n;
                  cvMap(i,j) = floor(15 + P0n*255);
                }
                else if (cvBool(i,j) == 1) {
                  P0 = epsilon*P0;
                  P1 = (1-epsilon)*P1;
                  double P1n = P1/(P0+P1); 
                  cvProb(i,j) = P1n;
                  cvMap(i,j) = floor(15 + P1n*127);
                }
                else {continue;}
                sensor_msgs::ImagePtr imMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cvMap).toImageMsg();
                im_pub.publish(imMsg);
              }
            } // end iterating through matrix



        } // end callback 
        


    public:
        FloorPlaneMapping() : nh_("~"), it_(nh_) {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("world_frame",world_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);
            nh_.param("n_x",n_x,10);
            nh_.param("n_y",n_y,10);
            nh_.param("epsilon",epsilon,0.2);
			
            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);
			
            int dims[2] = {n_x,n_y};
            cvMap = cv::Mat_<uint8_t>(2,dims);
            cvMap = 0;
            cvProb = cv::Mat_<float>(2,dims);
            cvProb = 0.5;			
            cvSlope = cv::Mat_<float>(2,dims);
            cvSlope = 0;
            cvBool = cv::Mat_<uint8_t>(2,dims);
			
            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans", 1, &FloorPlaneMapping::pc_callback, this);
            im_pub = it_.advertise("camera/image", 1);
            //ransac_sub_ = nh_.subscribe("floor_plane_ransac/floor_slope", 100, &FloorPlaneMapping::slope_callback, this);
            //ros::ServiceClient client = nh_.serviceClient<floor_plan_mapping::GetSlope>("Get_Slope");
            //floor_plane_mapping::GetSlope srv;
            //ransac_client_ = nh_.advertiseService("ransac_mapping", add);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Mapping");
    FloorPlaneMapping fp;

    ros::spin();
    return 0;
}

