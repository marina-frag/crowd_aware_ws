#include "crowd_aware_simulation/dynamic_dwal_fixed_core.hpp"
#include <iostream>
#include <stdexcept>

namespace cas = crowd_aware_simulation;
namespace f = cas::fixed_dwal;
int checks = 0;
void check(bool condition, const char * message)
{
  ++checks;
  if (!condition) {throw std::runtime_error(message);}
}
bool near(double a, double b, double eps=1.e-9) {return std::abs(a-b)<eps;}

int main()
{
  try {
    for (double k : {-4.0,-1.0,-0.2,0.0,0.2,1.0,4.0}) {
      const auto curve=f::completeCurve(k,1.8,0.05);
      check(curve.size()>2,"full far-level geometry");
      check(near(std::hypot(curve.back().pose.x,curve.back().pose.y),1.8),
        "far radius must not be interpreted as length");
      check(near(curve.back().s,f::lengthAtRadius(k,1.8)),"complete arc length");
      for (std::size_t i=1;i<curve.size();++i) {
        check(curve[i].s>curve[i-1].s,"strictly ordered samples");
      }
      const auto mirror=f::completeCurve(-k,1.8,0.05);
      check(near(curve.back().pose.x,mirror.back().pose.x),"mirror x");
      check(near(curve.back().pose.y,-mirror.back().pose.y),"mirror y");
    }
    const auto sharp=f::completeCurve(4,1.8,0.05);
    check(near(sharp.back().pose.theta,cas::kPi/2),"tangent-line heading");
    bool junction=false;
    for (const auto & p : sharp) {junction=junction||near(p.s,cas::arcLineTransition(4));}
    check(junction,"exact arc-line junction included");
    const cas::Pose2 origin{3,-2,0.7},p{0.2,0.9,-0.3};
    const auto roundtrip=f::toLocal(f::toWorld(p,origin),origin);
    check(near(p.x,roundtrip.x)&&near(p.y,roundtrip.y)&&near(p.theta,roundtrip.theta),
      "sample pose0/current odometry transform");

    const double slow=cas::stoppingDistance(.1,.4,.5,.05);
    const double fast=cas::stoppingDistance(.3,.4,.5,.05);
    check(cas::hasStaticBrakingClearance(slow,.15),"slow path accepted");
    check(!cas::hasStaticBrakingClearance(fast,.15),"same path rejected at higher speed");
    check(!cas::hasStaticBrakingClearance(fast,fast),"strict braking boundary");
    const auto overspeed=f::evaluateSpeed(.1,.6,.3,.5,.5,.1);
    check(near(overspeed.test,.6),"actual overspeed must not be hidden");
    const auto accelerate=f::evaluateSpeed(.3,0,.3,.5,.5,.1);
    check(near(accelerate.test,.05),"prospective acceleration included");

    const std::vector<f::ClusterMember> members{
      {0,-1,true},{1,-.8,false},{2,-.6,true},{3,-.4,true},
      {4,-.2,false},{5,0,false},{6,.2,true},{7,.4,true}};
    const auto joined=f::formClusters(members,3,0,1.8);
    check(joined.size()==1,"gap threshold joins surviving members");
    check(joined[0].members.size()==5,"rejected holes never become members");
    const auto separated=f::formClusters(members,2,0,1.8);
    check(separated.size()==2,"gap threshold splits clusters");
    const std::vector<f::ClusterMember> narrow{{0,-.01,true},{1,.01,true}};
    check(f::formClusters(narrow,5,.2,1.8).empty(),"minimum cluster chord span");
    const std::vector<f::ClusterMember> single{{0,0,true}};
    check(f::formClusters(single,5,.2,1.8).empty(),"singleton has zero span");

    const auto motion=f::brakingMotion(.3,0,.4,.5,1,.02);
    check(near(motion.back().pose.x,.21,1.e-8),"reaction plus braking distance");
    check(near(motion.back().time,1.0),"complete stopping time");
    check(near(motion.back().linear,0)&&near(motion.back().angular,0),"zero terminal speeds");
    const auto rotating=f::brakingMotion(0,.8,.4,.5,1,.02);
    check(near(rotating.back().pose.theta,.64,1.e-8),"angular stopping sweep");
    check(near(rotating.back().pose.x,0)&&near(rotating.back().pose.y,0),"in-place rotation");
    const f::Polygon footprint{{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
    check(!f::humanIntersectsMotion(motion,footprint,.25,.5,0,-1,0,false,.04,0,0),
      "OFF human frozen at current position");
    check(f::humanIntersectsMotion(motion,footprint,.25,.5,0,-1,0,true,.04,0,0),
      "ON crossing human rejected in simultaneous time");
    check(f::humanIntersectsMotion(motion,footprint,.30,0,1.5,0,0,false,.04,0,0),
      "OFF path meets static person");
    check(!f::humanIntersectsMotion(motion,footprint,.30,0,1.5,0,0,true,.04,0,0),
      "ON departing person permits motion");
    const auto aged=f::humanPosition(1,2,.5,-.2,.4,.6,true);
    check(near(aged.first,1.5)&&near(aged.second,1.8),"track age plus prediction time");
    const auto frozen=f::humanPosition(1,2,.5,-.2,.4,.6,false);
    check(near(frozen.first,1)&&near(frozen.second,2),"OFF ignores velocity and age extrapolation");
    check(f::polygonIntersectsRectangle(footprint,.09,-.2,.2,.2),"cell edge contact");
    check(!f::polygonIntersectsRectangle(footprint,.2,.2,.3,.3),"separate cell");
    std::cout<<"PASS: "<<checks<<" core checks\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr<<"FAIL: "<<e.what()<<"\n"; return 1;
  }
}
