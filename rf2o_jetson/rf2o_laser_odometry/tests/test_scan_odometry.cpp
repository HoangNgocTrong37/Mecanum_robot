#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

using rf2o::Pose3d;
const double pi=std::acos(-1.0);
Pose3d pose(double x,double y,double yaw) {
  Pose3d p=Pose3d::Identity(); p=rf2o::matrixYaw(yaw);
  p.translation()(0)=x;p.translation()(1)=y;return p;
}
sensor_msgs::msg::LaserScan scan(const Pose3d& laser,int tick) {
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id="laser";
  s.header.stamp=rclcpp::Time(1000000000LL+tick*100000000LL);
  s.angle_min=-pi;s.angle_max=pi;s.angle_increment=2*pi/1079;
  s.range_min=.15;s.range_max=12;s.scan_time=.1;
  const double x=laser.translation()(0),y=laser.translation()(1);
  const double yaw=rf2o::getYaw(laser.rotation());
  for(int i=0;i<1080;++i) {
    double a=yaw+s.angle_min+i*s.angle_increment,dx=std::cos(a),dy=std::sin(a);
    double best=12;
    // An asymmetric rectangular room with static boundaries.
    for(double wall : {-2.0,3.0}) if(std::abs(dx)>1e-10) {
      double t=(wall-x)/dx,hit=y+t*dy;
      if(t>0 && hit>=-1.5 && hit<=2.5)best=std::min(best,t);
    }
    for(double wall : {-1.5,2.5}) if(std::abs(dy)>1e-10) {
      double t=(wall-y)/dy,hit=x+t*dx;
      if(t>0 && hit>=-2 && hit<=3)best=std::min(best,t);
    }
    s.ranges.push_back(best);
  }
  return s;
}
void run(const char* label,double x,double y,double yaw) {
  rf2o::CLaserOdometry2D odom;
  const Pose3d mount=pose(.06,0,pi);
  odom.setLaserPose(mount);
  geometry_msgs::msg::Pose initial;initial.orientation.w=1;
  odom.init(scan(mount,0),initial);
  int tick=0;
  auto process=[&](const Pose3d& base) {
    auto s=scan(base*mount,++tick);
    odom.current_scan_time=s.header.stamp;
    if(!odom.odometryCalculation(s))throw std::runtime_error("Alignment rejected");
  };
  for(int i=0;i<4;++i)process(pose(0,0,0));
  for(int i=1;i<=20;++i)process(pose(x*i/20,y*i/20,yaw*i/20));
  auto estimated=odom.getPose();
  if(!estimated.matrix().allFinite() ||
     std::hypot(estimated.translation()(0)-x,estimated.translation()(1)-y)>.03 ||
     std::abs(rf2o::getYaw(estimated.rotation())-yaw)>.03)
    throw std::runtime_error(std::string(label)+": pose mismatch");
  if(x>.1 && odom.lin_speed<=0)throw std::runtime_error("Forward velocity sign");
  if(y>.1 && odom.lat_speed<=0)throw std::runtime_error("Lateral velocity sign");
  if(yaw>.1 && odom.ang_speed<=0)throw std::runtime_error("Yaw velocity sign");
  std::cout<<"PASS "<<label<<": pose="<<estimated.translation().transpose()
           <<" yaw="<<rf2o::getYaw(estimated.rotation())
           <<" twist="<<odom.lin_speed<<","<<odom.lat_speed<<","<<odom.ang_speed<<'\n';
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    run("stationary",0,0,0);run("forward",.2,0,0);
    run("lateral",0,.2,0);run("rotation",0,0,.2);
    rclcpp::shutdown();return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;
  }
}
