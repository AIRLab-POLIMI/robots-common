//This node convert gps fix from LLH to ENU
//It is possible to define the reference point as a parameter
//Topic published: /enu
//Message: geometry_msgs/PoseStamped
//Topic subscribed: /fix
//Message: sensor_msgs/NavSatFix
//Parameters: latitude, longitude, height - reference point
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define NO_FIX -1
#define COVARIANCE_TYPE_UNKNOWN 0

#define NAME_OF_THIS_NODE "nmea_to_enu"
#define PI 3.14159265
#define F 1.0/298.257223563
#define A 6378137
#define E2 F*(2 - F)

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Subscriber Subscriber;
  ros::Publisher Publisher;
  double X, Y, Z;
  double senphi, cosphi;
  double senlambda, coslambda;
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);  
public:
  bool Prepare();
};

bool ROSnode::Prepare() {
  double lat, lon, h;
  //Retrieve parameters
  if (Handle.getParam("/nmea_to_enu/latitude", lat)) {
    ROS_INFO("Node %s: retrieved parameter latitude.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter latitude.", ros::this_node::getName().c_str());
    return false;
  }
  if (Handle.getParam("/nmea_to_enu/longitude", lon)) {
    ROS_INFO("Node %s: retrieved parameter longitude.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter longitude.", ros::this_node::getName().c_str());
    return false;
  }
  if (Handle.getParam("/nmea_to_enu/height", h)) {
    ROS_INFO("Node %s: retrieved parameter height.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter height.", ros::this_node::getName().c_str());
    return false;
  }

  //Init publisher and subscriber
  Subscriber = Handle.subscribe("fix", 10, &ROSnode::gpsCallback, this);
  Publisher = Handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("enu", 10);
  
  //some math, used multiple times
  senphi = sin(lat * PI / 180.0);
  cosphi = cos(lat * PI / 180.0);
  senlambda = sin(lon * PI / 180.0);
  coslambda = cos(lon * PI / 180.0);

  double v = A / sqrt(1 - E2 * senphi * senphi);
  
  // reference coordinates in ECEF
  X = (v + h) * cosphi * coslambda;
  Y = (v + h) * cosphi * senlambda;
  Z = v*(1 - E2) * senphi;
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  
  return true;
}

void ROSnode::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if(msg->status.status == NO_FIX)
    return;

  double x, y, z;
  double clat, slat, clon, slon;

  slat = sin(msg->latitude * PI / 180.0);
  clat = cos(msg->latitude * PI / 180.0);
  slon = sin(msg->longitude * PI / 180.0);
  clon = cos(msg->longitude * PI / 180.0);
  
  double v = A / sqrt(1 - E2 * slat * slat);

  // fix in ECEF coordinates
  x = (v + msg->altitude) * clat * clon;
  y = (v + msg->altitude) * clat * slon;
  z = v*(1 - E2) * slat;

  geometry_msgs::PoseWithCovarianceStamped enu;

  //from ECEF to ENU
  enu.pose.pose.position.x = -senlambda * (x-X) + coslambda * (y-Y);
  enu.pose.pose.position.y = -senphi * coslambda * (x-X) - senphi * senlambda * (y-Y) + cosphi * (z-Z);
  enu.pose.pose.position.z = cosphi * coslambda * (x-X) + cosphi * senlambda * (y-Y) + senphi * (z-Z);
  
  enu.header = msg->header;
  boost::array<double, 36> covariance = {0};
  if(msg->position_covariance_type != COVARIANCE_TYPE_UNKNOWN) {
    int j = 0, k = 0;
    while(j < 13) {
      for(int i=0;i < 3; i++) {
	covariance[i+j] = msg->position_covariance[k];
	k++;
      }
      j = j + 6;
    }
  }
  enu.pose.covariance = covariance;
  Publisher.publish(enu);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  
  if(!node.Prepare())
    return 1;
  
  ros::spin(); 
  
  return (0);
}
