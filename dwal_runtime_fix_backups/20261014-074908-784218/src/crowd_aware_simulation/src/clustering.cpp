#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/agent_type.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <nav2_costmap_2d/footprint.hpp>

#include <dwal_planner/libdwal_cluster.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <dwal_planner/msg/cluster_group.hpp>
#include <dwal_planner/msg/sampled_cluster.hpp>
#include <dwal_planner/msg/path_cluster.hpp>
#include <dwal_planner/msg/path.hpp>
#include <dwal_planner/msg/pose2_d32.hpp>

#include <dwal_planner/srv/toggle_clusters.hpp>
#include <dwal_planner/srv/toggle_slice.hpp>

using std::placeholders::_1;

double colors[] = {
  1, 0, 0,  0, 1, 0,  0, 0, 1,  1, 1, 0,
  0, 1, 1,  1, 0, 1,  0.5, 0.5, 0.2,  0.8, 0.1, 0.4
};

void setColor(visualization_msgs::msg::Marker & path_marker, int c, double cost)
{
  c = (c % 8 + 8) % 8;
  path_marker.color.r = colors[3 * c] * cost;
  path_marker.color.g = colors[3 * c + 1] * cost;
  path_marker.color.b = colors[3 * c + 2] * cost;
}

// Original DWAL candidates, clipped at the first collision before clustering.
class BrakingFilter
{
public:
  explicit BrakingFilter(const std::shared_ptr<rclcpp::Node> & node) : node_(node)
  {
    enabled = node_->declare_parameter<bool>("dwal_clustering/braking_filter", false);
    if (!enabled) throw std::runtime_error("Variable-radius DWAL requires braking_filter=true");
    mode_ = node_->declare_parameter<std::string>("mode", "off");
    a_ = node_->declare_parameter<double>("dwal_clustering/a_brake", 0.5);
    reaction_ = node_->declare_parameter<double>("dwal_clustering/reaction_time", 0.4);
    margin_ = node_->declare_parameter<double>("dwal_clustering/stop_margin", 0.05);
    vmax_ = node_->declare_parameter<double>("dwal_clustering/max_linear", 0.3);
    wmax_ = node_->declare_parameter<double>("dwal_clustering/max_angular", 0.8);
    timeout_ = node_->declare_parameter<double>("dwal_clustering/input_timeout", 0.5);
    reference_timeout_ = node_->declare_parameter<double>("dwal_clustering/reference_timeout", 0.35);
    human_radius_ = node_->declare_parameter<double>("dwal_clustering/human_radius", 0.4);
    uncertainty_ = node_->declare_parameter<double>("dwal_clustering/human_uncertainty", 0.1);
    uncertainty_growth_ = node_->declare_parameter<double>("dwal_clustering/human_uncertainty_growth", 0.1);
    horizon_ = node_->declare_parameter<double>("dwal_clustering/prediction_horizon", 2.0);
    const auto xy = node_->declare_parameter<std::vector<double>>("dwal_clustering/footprint", {});
    if ((mode_!="on" && mode_!="off") || xy.size()<6 || xy.size()%2 ||
      !std::isfinite(a_) || a_<=0 || !std::isfinite(reaction_) || reaction_<0 ||
      !std::isfinite(margin_) || margin_<0 || !std::isfinite(vmax_) || vmax_<=0 ||
      !std::isfinite(wmax_) || wmax_<=0 || !std::isfinite(timeout_) || timeout_<=0 ||
      !std::isfinite(reference_timeout_) || reference_timeout_<=0 ||
      !std::isfinite(human_radius_) || human_radius_<0 ||
      !std::isfinite(uncertainty_) || uncertainty_<0 ||
      !std::isfinite(uncertainty_growth_) || uncertainty_growth_<0 ||
      !std::isfinite(horizon_) || horizon_<=0)
      throw std::runtime_error("Invalid fixed DWAL braking settings");
    for (size_t i=0;i<xy.size();i+=2) {
      if (!std::isfinite(xy[i]) || !std::isfinite(xy[i+1]))
        throw std::runtime_error("Non-finite footprint");
      geometry_msgs::msg::Point p; p.x=xy[i]; p.y=xy[i+1]; footprint_.push_back(p);
    }
    nav2_costmap_2d::padFootprint(footprint_,0.02);
    for (const auto & p : footprint_) footprint_radius_=std::max(footprint_radius_,std::hypot(p.x,p.y));
    map_=std::make_unique<costmap_2d::Costmap2DROS>(node_,"/local_costmap/costmap");
    odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr m){odom_=m; odom_rx_=node_->now().seconds();});
    reference_sub_=node_->create_subscription<geometry_msgs::msg::Twist>("/reference_cmd",rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr m){reference_=m; reference_rx_=node_->now().seconds();});
    tracks_sub_=node_->create_subscription<cohan_msgs::msg::TrackedAgents>("/tracked_agents",rclcpp::QoS(10),
      [this](cohan_msgs::msg::TrackedAgents::SharedPtr m){tracks_=m; tracks_rx_=node_->now().seconds();});
    RCLCPP_INFO(node_->get_logger(),"DWAL_VARIABLE_R_V2: original DWAL candidates, obstacle-limited radii, mode=%s",mode_.c_str());
  }

  bool prepare()
  {
    const double now=node_->now().seconds();
    if (!odom_ || !reference_ || !fresh(odom_rx_,timeout_) ||
      !fresh(reference_rx_,reference_timeout_) || !fresh(map_->lastUpdate(),timeout_) ||
      !sourceFresh(odom_->header.stamp,timeout_) || odom_->header.frame_id!="odom" ||
      !std::isfinite(reference_->linear.x) || !std::isfinite(reference_->angular.z) ||
      !std::isfinite(odom_->twist.twist.linear.x) ||
      reference_->linear.x<0 || odom_->twist.twist.linear.x < -1.e-3) return false;
    speed_=std::max(std::abs(odom_->twist.twist.linear.x),
      std::clamp(reference_->linear.x,0.0,vmax_));
    stop_distance_=speed_*reaction_+speed_*speed_/(2*a_);
    required_=stop_distance_+margin_;
    humans_.clear();
    if (mode_=="on") {
      if (!tracks_ || !fresh(tracks_rx_,timeout_) ||
        !sourceFresh(tracks_->header.stamp,timeout_) || tracks_->header.frame_id!="odom" ||
        reaction_+speed_/a_>horizon_) return false;
      const double stamp=rclcpp::Time(tracks_->header.stamp).seconds();
      const double age=stamp>0 ? std::max(0.0,now-stamp) : now-tracks_rx_;
      for (const auto & agent : tracks_->agents) {
        if (agent.type!=cohan_msgs::msg::AgentType::HUMAN) continue;
        const cohan_msgs::msg::TrackedSegment * torso=nullptr;
        for (const auto & segment : agent.segments)
          if (segment.type==cohan_msgs::msg::TrackedSegmentType::TORSO) {torso=&segment; break;}
        if (!torso) return false;
        const double x=torso->pose.pose.position.x,y=torso->pose.pose.position.y;
        const double vx=torso->twist.twist.linear.x,vy=torso->twist.twist.linear.y;
        if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(vx)||!std::isfinite(vy)) return false;
        humans_.push_back({x+vx*age,y+vy*age,vx,vy});
      }
    }
    auto * raw=map_->getCostmap();
    if (!raw || raw->getSizeInCellsX()==0 || raw->getSizeInCellsY()==0) return false;
    collision_map_=raw;
    if (mode_=="on") {
      // Mask only local human disks, and reuse the snapshot until map/tracks change.
      if (!checked_map_ || cached_map_rx_!=map_->lastUpdate() || cached_tracks_rx_!=tracks_rx_) {
        checked_map_=std::make_unique<nav2_costmap_2d::Costmap2D>(*raw);
        const double map_age=std::max(0.0,now-map_->lastUpdate());
        const double res=raw->getResolution();
        const double extent=human_radius_+uncertainty_+0.5*std::sqrt(2.0)*res;
        double ox,oy; raw->mapToWorld(0,0,ox,oy); ox-=res/2; oy-=res/2;
        for (const auto & h : humans_) {
          const double hx=h.x-h.vx*map_age,hy=h.y-h.vy*map_age;
          const int x0=std::max(0,static_cast<int>(std::floor((hx-extent-ox)/res)));
          const int y0=std::max(0,static_cast<int>(std::floor((hy-extent-oy)/res)));
          const int x1=std::min(static_cast<int>(raw->getSizeInCellsX())-1,
            static_cast<int>(std::floor((hx+extent-ox)/res)));
          const int y1=std::min(static_cast<int>(raw->getSizeInCellsY())-1,
            static_cast<int>(std::floor((hy+extent-oy)/res)));
          for (int y=y0;y<=y1;++y) for (int x=x0;x<=x1;++x) {
            if (raw->getCost(x,y)!=nav2_costmap_2d::LETHAL_OBSTACLE) continue;
            double wx,wy;raw->mapToWorld(x,y,wx,wy);
            if (std::hypot(wx-hx,wy-hy)<=extent)
              checked_map_->setCost(x,y,nav2_costmap_2d::FREE_SPACE);
          }
        }
        cached_map_rx_=map_->lastUpdate(); cached_tracks_rx_=tracks_rx_;
      }
      collision_map_=checked_map_.get();
    }
    return true;
  }

  bool accepts(dwal_planner::msg::SampledPath & path,int & prefix_cost)
  {
    prefix_cost=0;
    if (path.poses.empty() || !std::isfinite(path.curvature) || !collision_map_) return false;
    const bool steering_ok=std::abs(path.curvature*speed_)<=wmax_+1.e-9;
    const auto candidates=std::move(path.poses);
    path.poses.clear(); path.poses.reserve(candidates.size()+1); path.colission_r=10000;
    base_local_planner::CostmapModel model(collision_map_);
    double distance=0,available=0;
    bool hit=false;
    dwal_planner::msg::Pose2D32 last_free;
    bool have_free=false;
    for (size_t i=0;i<candidates.size() && !hit;++i) {
      const auto & b=candidates[i]; const auto & a=i ? candidates[i-1] : b;
      if (!std::isfinite(b.x)||!std::isfinite(b.y)||!std::isfinite(b.theta)) {hit=true;break;}
      const double ds=std::hypot(b.x-a.x,b.y-a.y);
      const int steps=std::max(1,static_cast<int>(std::ceil(ds/0.02)));
      const double dtheta=std::atan2(std::sin(b.theta-a.theta),std::cos(b.theta-a.theta));
      for (int j=(i ? 1 : 0);j<=(i ? steps : 0);++j) {
        const double u=static_cast<double>(j)/steps,s=distance+u*ds;
        const double x=a.x+u*(b.x-a.x),y=a.y+u*(b.y-a.y),theta=a.theta+u*dtheta;
        const int cost=model.footprintCost(x,y,theta,footprint_);
        bool collision=cost<0 || cost>nav2_costmap_2d::MAX_NON_OBSTACLE;
        if (!collision && mode_=="on") {
          // Constant-velocity prediction for the available path. Also test the
          // slower braking motion on the prefix actually needed to stop.
          const double v=std::max(speed_,0.05),t=s/v,dt=ds/steps/v;
          collision=humanCollision(x,y,theta,t,dt,ds/steps,path.curvature);
          if (!collision && s<=required_+ds/steps) {
            const double bt=timeAtDistance(std::min(s,stop_distance_));
            const double bdt=std::max(0.0,timeAtDistance(std::min(s+ds/steps,stop_distance_))-bt);
            collision=humanCollision(x,y,theta,bt,bdt,ds/steps,path.curvature);
          }
        }
        if (collision) {
          path.colission_r=std::hypot(x-candidates.front().x,y-candidates.front().y);
          hit=true;break;
        }
        if (s<=required_+ds/steps) prefix_cost=std::max(prefix_cost,cost);
        available=s;have_free=true;
        last_free.x=static_cast<float>(x);last_free.y=static_cast<float>(y);
        last_free.theta=static_cast<float>(theta);
      }
      if (!hit) path.poses.push_back(b);
      distance+=ds;
    }
    if (hit && have_free && (path.poses.empty() ||
      std::hypot(path.poses.back().x-last_free.x,path.poses.back().y-last_free.y)>1.e-6))
      path.poses.push_back(last_free);
    const int last=path.poses.empty() ? -1 : static_cast<int>(path.poses.size())-1;
    std::fill(path.level_inds.begin(),path.level_inds.end(),last);
    std::fill(path.costs.begin(),path.costs.end(),prefix_cost);
    return have_free && steering_ok && available+1.e-9>=required_;
  }

  bool enabled{false};
private:
  struct Human {double x,y,vx,vy;};
  bool humanCollision(double x,double y,double theta,double t,double dt,double ds,double k) const {
    for (const auto & h : humans_) {
      const double extent=human_radius_+uncertainty_+uncertainty_growth_*t+
        ds*(1.0+footprint_radius_*std::abs(k))+std::hypot(h.vx,h.vy)*dt;
      const double hx=h.x+h.vx*t,hy=h.y+h.vy*t;
      const double outer=extent+footprint_radius_;
      if ((hx-x)*(hx-x)+(hy-y)*(hy-y)>outer*outer) continue;
      if (distanceToFootprint(hx,hy,x,y,theta)<=extent) return true;
    }
    return false;
  }
  bool fresh(double rx,double timeout) const {
    const double age=node_->now().seconds()-rx;
    return std::isfinite(age) && age>=0 && age<=timeout;
  }
  bool sourceFresh(const builtin_interfaces::msg::Time & stamp,double timeout) const {
    const double source=rclcpp::Time(stamp).seconds();
    return source==0 || fresh(source,timeout);
  }
  double timeAtDistance(double s) const {
    if (speed_<=1.e-9) return 0;
    if (s<=speed_*reaction_) return s/speed_;
    return reaction_+(speed_-std::sqrt(std::max(0.0,
      speed_*speed_-2*a_*(s-speed_*reaction_))))/a_;
  }
  double distanceToFootprint(double hx,double hy,double x,double y,double theta) const {
    const double c=std::cos(theta),s=std::sin(theta);
    const double px=c*(hx-x)+s*(hy-y),py=-s*(hx-x)+c*(hy-y);
    double best=std::numeric_limits<double>::infinity(); int sign=0; bool inside=true;
    for (size_t i=0;i<footprint_.size();++i) {
      const auto & a=footprint_[i]; const auto & b=footprint_[(i+1)%footprint_.size()];
      const double dx=b.x-a.x,dy=b.y-a.y,cross=dx*(py-a.y)-dy*(px-a.x);
      const int side=cross>1.e-10 ? 1 : (cross < -1.e-10 ? -1 : 0);
      if (side && sign && side!=sign) inside=false;
      if (side) sign=side;
      const double denominator=dx*dx+dy*dy;
      const double u=denominator>0 ? std::clamp(((px-a.x)*dx+(py-a.y)*dy)/denominator,0.0,1.0) : 0;
      best=std::min(best,std::hypot(px-a.x-u*dx,py-a.y-u*dy));
    }
    return inside ? 0 : best;
  }
  std::shared_ptr<rclcpp::Node> node_;
  std::string mode_;
  double a_{.5},reaction_{.4},margin_{.05},vmax_{.3},wmax_{.8},timeout_{.5},reference_timeout_{.35};
  double human_radius_{.4},uncertainty_{.1},uncertainty_growth_{.1},horizon_{2},speed_{0},stop_distance_{0},required_{0};
  double odom_rx_{-1.e10},reference_rx_{-1.e10},tracks_rx_{-1.e10};
  std::vector<geometry_msgs::msg::Point> footprint_;
  std::vector<Human> humans_;
  std::unique_ptr<costmap_2d::Costmap2DROS> map_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> checked_map_;
  nav2_costmap_2d::Costmap2D * collision_map_{nullptr};
  double footprint_radius_{0},cached_map_rx_{-1.e10},cached_tracks_rx_{-1.e10};
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::Twist::SharedPtr reference_;
  cohan_msgs::msg::TrackedAgents::SharedPtr tracks_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reference_sub_;
  rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr tracks_sub_;
};

class Clustering
{
public:
  explicit Clustering(const std::shared_ptr<rclcpp::Node>& node)
  : node_(node)
  {
    prev_cluster_num = 0;
    source_slice_group_ptr = nullptr;
    source_slice_cluster_index = 1;
    sliced_path_indexes.clear();
    do_slice = false;
    direction = false;

    node_->get_parameter("dwal_clustering/min_cluster_span", min_cluster_span);
    node_->get_parameter("dwal_clustering/cluster_separation", cluster_separation);
    node_->get_parameter("dwal_clustering/subsample_step", subsample_step);

    std::string odom_frame;
    node_->get_parameter("common/odom_frame", odom_frame);
    RCLCPP_INFO(node_->get_logger(), "Using odometry frame '%s'", odom_frame.c_str());

    // Wait until the generator sets a non-negative value
    marker_num = -1;
    while (rclcpp::ok() && marker_num < 0) {
      (void)node_->get_parameter("dwal_clustering/Marker_num", marker_num);
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "Waiting for parameter 'dwal_clustering/Marker_num' to be set...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(node_);
    }

    RCLCPP_INFO(node_->get_logger(), "Marker_num received: %d", marker_num);
    // Init marker template
    path_marker.header.frame_id = odom_frame;
    path_marker.header.stamp = node_->now();
    path_marker.ns = "dwal_planner";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.x = 0.0;
    path_marker.pose.orientation.y = 0.0;
    path_marker.pose.orientation.z = 0.0;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.01;
    path_marker.color.r = 0.7;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 0.8;

    path_array_marker.markers.reserve(static_cast<size_t>(marker_num));
    for (int k = 0; k < marker_num; k++) {
      path_marker.id = k;
      path_array_marker.markers.push_back(path_marker);
    }
  }

  void set_publishers(const std::string& marker_topic, const std::string& cluster_topic)
  {
    markers_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(2));
    group_publisher   = node_->create_publisher<dwal_planner::msg::ClusterGroup>(cluster_topic, rclcpp::QoS(2));
  }

  int clusterIdtoIndex(const int& id, const dwal_planner::msg::ClusterGroup* cl_group)
  {
    for (int k = 0; k < static_cast<int>(cl_group->clusters.size()); k++)
      if (cl_group->clusters[k].id == id)
        return k;
    return -1;
  }

  void findCorrespondence()
  {
    corresp_matrix.clear();
    if (cluster_num == 0) //current frame empty
    {
      prev_ids.clear();
      prev_mean_phi.clear();
      prev_cluster_num = cluster_num;
      return;
    }

    if (prev_cluster_num == 0) //previous frame empty
    {
      prev_ids.clear();
      prev_mean_phi.clear();
      prev_cluster_num = cluster_num;
      unsigned int i = 0;
      for (auto &cl : group.clusters)
      {
        cl.id = static_cast<int>(i);
        prev_mean_phi.push_back((cluster_lib::curv2phi(cl.paths.front().curvature,levels[level_index]) +
        cluster_lib::curv2phi(cl.paths.back().curvature,levels[level_index])) / 2);
        prev_ids.push_back(i);
        i++;
      }
      return;
    }

    cur_mean_phi.clear();
    phi_distances.clear();
    temp.clear();

    //calculate mean phis for current clusters
    for (auto &cl : group.clusters)
      cur_mean_phi.push_back((cluster_lib::curv2phi(cl.paths.front().curvature,levels[level_index]) +
        cluster_lib::curv2phi(cl.paths.back().curvature,levels[level_index])) / 2);

    temp.resize(static_cast<size_t>(cluster_num)); //calculate mean phi distance between clusters
    for (int k = 0; k < prev_cluster_num; k++)
    {
      phi_distances.push_back(temp);
      for (int i = 0; i < cluster_num; i++)
        phi_distances[static_cast<size_t>(k)][static_cast<size_t>(i)] = std::fabs(cur_mean_phi[static_cast<size_t>(i)] - prev_mean_phi[static_cast<size_t>(k)]);
    }

    int row, col; //find minimum phi distance
    double minimum;
    for (int j = 0; j < std::min(prev_cluster_num, cluster_num); j++)
    {
      row = col = 0;
      minimum = 1000;
      for (int k = 0; k < prev_cluster_num; k++)
      {
        for (int i = 0; i < cluster_num; i++)
        {
          if (phi_distances[static_cast<size_t>(k)][static_cast<size_t>(i)] < minimum)
          {
            minimum = phi_distances[static_cast<size_t>(k)][static_cast<size_t>(i)];
            row = k;
            col = i;
          }
        }
      }
      group.clusters[static_cast<size_t>(col)].id = static_cast<int>(prev_ids[static_cast<size_t>(row)]); //assign same id to min distance
      corresp_matrix[prev_ids[static_cast<size_t>(row)]] = static_cast<unsigned int>(col); // prev ID -> current INDEX

      //invalidate selected row and col by assigning distance=1000
      for (int k = 0; k < prev_cluster_num; k++)
        phi_distances[static_cast<size_t>(k)][static_cast<size_t>(col)] = 1000;
      std::fill(phi_distances[static_cast<size_t>(row)].begin(), phi_distances[static_cast<size_t>(row)].end(), 1000);
    }

    if (cluster_num > prev_cluster_num)
    {
      unsigned int j = 0; //for the rest of the clusters assign appropriate id
      std::vector<unsigned int>::iterator it;
      it = std::find(prev_ids.begin(), prev_ids.end(), j);
      for (auto &cl : group.clusters)
      {
        j = 0;
        if (cl.id == 100)
        {
          while (it != prev_ids.end())
          {
            j++;
            it = std::find(prev_ids.begin(), prev_ids.end(), j);
          }
          cl.id = static_cast<int>(j);
          prev_ids.push_back(j);
        }
      }
    }
    prev_ids.clear();
    prev_mean_phi.clear();
    prev_cluster_num = cluster_num;
    prev_mean_phi = cur_mean_phi;
    for (int i = 0; i < cluster_num; i++)
      prev_ids.push_back(static_cast<unsigned int>(group.clusters[static_cast<size_t>(i)].id));
  }

  void doClustering(const dwal_planner::msg::SampledCluster::SharedPtr msg)
  {
    // Use the obstacle-limited poses for both radius and cluster geometry.
    cluster_num = 0;
    Rcl = levels[static_cast<size_t>(level_index)];
    path_num = static_cast<int>(msg->paths.size());
    free_path_indexes.clear();
    cluster_indexes.clear();
    cluster_vecs.clear();
    group.clusters.clear();
    for (int k = 0; k < marker_num; k++)
      path_array_marker.markers[static_cast<size_t>(k)].points.clear(); //clear all path markers

    for (int k = 0; k < path_num; k++)
    { //get free paths
      if (use_braking_filter ? (static_cast<size_t>(k)<admissible.size() && admissible[static_cast<size_t>(k)]) :
          msg->paths[static_cast<size_t>(k)].colission_r > Rcl)
        free_path_indexes.push_back(k);
    }

    if (free_path_indexes.empty() || msg->pose0.size()<2) // at least one usable path?
    {
      group_publisher->publish(group); //publish empty clusters
      markers_publisher->publish(path_array_marker); //publish empty markers and exit
      return;
    }

    // do cluster slicing with source group, if flag is raised
    //////--- start slicing -----------------------------------------------------------
    if (do_slice && source_slice_group_ptr && !source_slice_group_ptr->clusters.empty())
    {
      source_slice_cluster_index = clusterIdtoIndex(source_slice_cluster_id, source_slice_group_ptr);
      if (source_slice_cluster_index == -1)
      {
        RCLCPP_INFO(node_->get_logger(), "--- Source cluster ID not found. Skipping slicing...");
        return;
      }
      switch (direction)
      {
        case false: //false = right
          max_curv_slice = source_slice_group_ptr->clusters[static_cast<size_t>(source_slice_cluster_index)].paths.back().curvature;
          if (source_slice_cluster_index > 0)
            min_curv_slice = source_slice_group_ptr->clusters[static_cast<size_t>(source_slice_cluster_index - 1)].paths.back().curvature;
          else
            min_curv_slice = msg->paths.front().curvature;
          break;

        case true: //true = left
          min_curv_slice = source_slice_group_ptr->clusters[static_cast<size_t>(source_slice_cluster_index)].paths[0].curvature;
          if (source_slice_cluster_index < static_cast<int>(source_slice_group_ptr->clusters.size()) - 1)
            max_curv_slice = source_slice_group_ptr->clusters[static_cast<size_t>(source_slice_cluster_index + 1)].paths[0].curvature;
          else
            max_curv_slice = msg->paths.back().curvature;
          break;
      }

      for (int &k : free_path_indexes)
        if (msg->paths[static_cast<size_t>(k)].curvature >= min_curv_slice &&
            msg->paths[static_cast<size_t>(k)].curvature <= max_curv_slice)
          sliced_path_indexes.push_back(k);

      free_path_indexes = sliced_path_indexes;
      sliced_path_indexes.clear();
    }
    //////--- end slicing ----------------------------------------------------------------

    if (free_path_indexes.empty()) {
      group_publisher->publish(group); markers_publisher->publish(path_array_marker); return;
    }
    const auto pathRadius=[&](int index) {
      const auto & poses=msg->paths[static_cast<size_t>(index)].poses;
      return poses.empty() ? 0.0 : std::hypot(poses.back().x-msg->pose0[0],poses.back().y-msg->pose0[1]);
    };
    const auto clusterRadius=[&](const std::vector<int> & indices) {
      double r=Rcl;
      if (use_braking_filter) for (int index : indices) r=std::min(r,pathRadius(index));
      return r;
    };
    const auto finishCluster=[&]() {
      if (cluster_indexes.empty()) return;
      const double r=clusterRadius(cluster_indexes);
      const double span=cluster_lib::curv2phi(msg->paths[cluster_indexes.back()].curvature,r)-
        cluster_lib::curv2phi(msg->paths[cluster_indexes.front()].curvature,r);
      if (r>1.e-6 && cluster_lib::clusterChord(r,span)>=min_cluster_span)
        cluster_vecs.push_back(cluster_indexes);
      cluster_indexes.clear();
    };
    // Preserve fixed DWAL adjacency and the two existing clustering parameters.
    for (int index : free_path_indexes) {
      if (!cluster_indexes.empty() && index-cluster_indexes.back()>cluster_separation) finishCluster();
      cluster_indexes.push_back(index);
    }
    finishCluster();

    cluster_num = static_cast<int>(cluster_vecs.size());
    //////--- end clustering -----------------------------------------------------------

    //populate cluster_group msg and path markers
    //int j;
    for (std::vector<int> &clvec : cluster_vecs)
    {
      tmp_cluster.paths.clear(); //clear temp cluster
      tmp_cluster.id = 100; //start with id=100. Will change in correspondence matching later
      const double available_r=clusterRadius(clvec);
      tmp_cluster.r = available_r;
      tmp_cluster.pose0 = msg->pose0;
      
      for (int &l : clvec)
      {
        if (*spin != 0)
        {
          const auto & path = msg->paths[static_cast<size_t>(l)];
          const int last = std::min(
            static_cast<int>(path.poses.size()) - 1,
            static_cast<int>(path.level_inds[static_cast<size_t>(level_index)]));

          for (int j = 0; j <= last; j += subsample_step)
          {
            p0.x = path.poses[static_cast<size_t>(j)].x;
            p0.y = path.poses[static_cast<size_t>(j)].y;
            path_array_marker.markers[static_cast<size_t>(l)].points.push_back(p0);
          }
          if (last >= 0 && (last % subsample_step) != 0)
          {
            p0.x = path.poses[static_cast<size_t>(last)].x;
            p0.y = path.poses[static_cast<size_t>(last)].y;
            path_array_marker.markers[static_cast<size_t>(l)].points.push_back(p0);
          }
        }
        tmp_path.curvature = msg->paths[static_cast<size_t>(l)].curvature;
        tmp_path.phi = cluster_lib::curv2phi(msg->paths[static_cast<size_t>(l)].curvature, available_r);
        tmp_path.cost = msg->paths[static_cast<size_t>(l)].costs[static_cast<size_t>(level_index)];
        tmp_cluster.paths.push_back(tmp_path); //add path to current cluster    
      }
      group.clusters.push_back(tmp_cluster);
    }

    findCorrespondence(); //set cluster ids according to correspondence matching

    if (*spin != 0)
    {
      //draw clusters with different color
      for (int k = 0; k < cluster_num; k++)
        for (int l : cluster_vecs[static_cast<size_t>(k)])
          setColor(path_array_marker.markers[static_cast<size_t>(l)],
                   group.clusters[static_cast<size_t>(k)].id,
                   0.3 + 0.7 * (static_cast<double>(msg->paths[static_cast<size_t>(l)].costs[static_cast<size_t>(level_index)]) / 128.0));
    }

    //publish markers & groups
    group_publisher->publish(group);
    markers_publisher->publish(path_array_marker);
  }

  bool use_braking_filter{false};
  std::vector<bool> admissible;
  std::vector<int>::iterator spin;
  int level_index{0};
  std::vector<double> levels;

  std::map<unsigned int, unsigned int> corresp_matrix;
  dwal_planner::msg::ClusterGroup group;
  dwal_planner::msg::ClusterGroup* source_slice_group_ptr{nullptr};
  int source_slice_cluster_id{0};
  int source_slice_cluster_index{1};
  bool do_slice{false};
  bool direction{false};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
  rclcpp::Publisher<dwal_planner::msg::ClusterGroup>::SharedPtr group_publisher;

private:
  std::shared_ptr<rclcpp::Node> node_;

  geometry_msgs::msg::Point p0;
  visualization_msgs::msg::Marker path_marker;
  visualization_msgs::msg::MarkerArray path_array_marker;
  
  double Dphip{0.0}, min_cluster_span{0.5}, Rcl{0.0}, curv_mob{0.0}, phi0{0.0};
  std::vector<double> temp, prev_mean_phi, cur_mean_phi;

  std::vector<int> free_path_indexes, sliced_path_indexes, cluster_indexes;
  std::vector<std::vector<int>> cluster_vecs;
  std::vector<unsigned int> prev_ids;
  std::vector<std::vector<double>> phi_distances;
  int marker_num{0}, cluster_separation{5}, cluster_num{0}, path_num{0}, prev_cluster_num{0}, Dind{0};
  int subsample_step{3};

  dwal_planner::msg::PathCluster tmp_cluster;
  dwal_planner::msg::Path tmp_path;
  double min_curv_slice{0.0}, max_curv_slice{0.0};
};

/* main class that handles the sampled paths. It creates Clustering objects for each level (group)
   and performs the clustering by calling their appropriate method */
class ControlClass
{
public:
  std::shared_ptr<rclcpp::Node> ros_node;
  explicit ControlClass(const std::shared_ptr<rclcpp::Node>& node)
  : ros_node(node), brake_filter(node)
  {
    markerTopic = "cluster_markers_";
    clusterTopic = "clusters_";

    ros_node->declare_parameter<std::vector<std::string>>("dwal_clustering/postfix", {});
    ros_node->declare_parameter<std::vector<double>>("common/levels", {});
    ros_node->declare_parameter<std::vector<int64_t>>("dwal_clustering/spin", {});
    ros_node->declare_parameter<double>("dwal_clustering/min_cluster_span", 0.5);
    ros_node->declare_parameter<int>("dwal_clustering/cluster_separation", 5);
    ros_node->declare_parameter<int>("dwal_clustering/Marker_num", -1);
    ros_node->declare_parameter<std::string>("common/odom_frame", "odom");
    ros_node->declare_parameter<int>("dwal_clustering/subsample_step", 3);

    ros_node->get_parameter("dwal_clustering/postfix", postfixes);
    ros_node->get_parameter("common/levels", levels);
    ros_node->get_parameter("dwal_clustering/spin", spin64);


    spin.clear();
    spin.reserve(spin64.size());
    for (auto v : spin64) spin.push_back(static_cast<int>(v));

    group_num = postfixes.size();
    if (group_num != levels.size())
    {
      RCLCPP_ERROR(ros_node->get_logger(),
                   "\n\n----ERROR : 'postfix' and 'levels' should have the same number of elements...\n"
                   "----aborting launch of clustering node...\n");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(ros_node->get_logger(), "Cluster groups to be created: %zu", group_num);
    for (size_t k=0;k<group_num;++k)
      RCLCPP_INFO(ros_node->get_logger(), "Cluster postfix: %s (%.2f m)", postfixes[k].c_str(),levels[k]);
    if (spin.size()!=group_num) throw std::runtime_error("spin and levels must have equal size");

    do_slice = false;
    group_objects.reserve(group_num);
    for (size_t k = 0; k < group_num; k++) {
      group_objects.emplace_back(ros_node);  
      clustering_order.push_back(static_cast<int>(k));
      group_objects[k].level_index = static_cast<int>(k);
      group_objects[k].levels = levels;
      group_objects[k].spin = spin.begin() + static_cast<long>(k);
      const std::string marker_topic  = markerTopic  + postfixes[k];
      const std::string clusters_topic = clusterTopic + postfixes[k];
      group_objects[k].set_publishers(marker_topic, clusters_topic);
    }

    available_paths_publisher=ros_node->create_publisher<dwal_planner::msg::SampledCluster>(
      "available_paths",rclcpp::QoS(1));
    available_markers_publisher=ros_node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "available_pathMarkers",rclcpp::QoS(1));

    // Subscriber
    sampled_cluster_subscriber = ros_node->create_subscription<dwal_planner::msg::SampledCluster>(
      "sampled_paths", rclcpp::QoS(1),
      std::bind(&ControlClass::sampledClusterHandler, this, _1));

    // Services
    toggleSpinService = ros_node->create_service<dwal_planner::srv::ToggleClusters>(
      "toggle_cluster_spin",
      [this](const std::shared_ptr<dwal_planner::srv::ToggleClusters::Request> req,
             std::shared_ptr<dwal_planner::srv::ToggleClusters::Response> res)
      {
        for (size_t k = 0; k < req->flags.size() && k < spin.size(); k++)
          spin[k] = req->flags[k];
        res->success = true;
      });

    toggleSliceService = ros_node->create_service<dwal_planner::srv::ToggleSlice>(
      "toggle_cluster_slice",
      [this](const std::shared_ptr<dwal_planner::srv::ToggleSlice::Request> req,
             std::shared_ptr<dwal_planner::srv::ToggleSlice::Response> res)
      {
        if (req->source_group_index<0 || req->target_group_index<0 ||
          static_cast<size_t>(req->source_group_index)>=group_num ||
          static_cast<size_t>(req->target_group_index)>=group_num ||
          req->source_group_index==req->target_group_index) {
          res->success=false; return;
        }
        clustering_order.clear();
        clustering_order.push_back(req->source_group_index);
        clustering_order.push_back(req->target_group_index);
        for (int k = 0; k < static_cast<int>(group_num); k++)
        {
          group_objects[static_cast<size_t>(k)].do_slice = false; // reset slicing flags
          do_slice = false;
          if ((k != req->source_group_index) && (k != req->target_group_index))
            clustering_order.push_back(k);
        }

        if (req->do_slice)
        {
          group_objects[static_cast<size_t>(req->target_group_index)].do_slice = true;
          group_objects[static_cast<size_t>(req->target_group_index)].direction = req->direction;
          group_objects[static_cast<size_t>(req->target_group_index)].source_slice_cluster_id = req->source_cluster_id;
          do_slice = true;
        }
        res->success = true;
      });
  }

  void sampledClusterHandler(const dwal_planner::msg::SampledCluster::SharedPtr input)
  {
    const auto started=std::chrono::steady_clock::now();
    auto msg=std::make_shared<dwal_planner::msg::SampledCluster>(*input);
    if (brake_filter.enabled) {
      const bool ready=brake_filter.prepare();
      std::vector<bool> accepted(msg->paths.size(),false);
      for (size_t i=0;i<msg->paths.size();++i) {
        int prefix_cost=0;
        accepted[i]=ready && brake_filter.accepts(msg->paths[i],prefix_cost);
        if (!ready) {
          msg->paths[i].poses.clear();
          std::fill(msg->paths[i].level_inds.begin(),msg->paths[i].level_inds.end(),-1);
        }
        if (accepted[i]) std::fill(msg->paths[i].costs.begin(),msg->paths[i].costs.end(),prefix_cost);
      }
      for (auto & group : group_objects) {
        group.use_braking_filter=true; group.admissible=accepted;
      }
      if (!ready) RCLCPP_WARN_THROTTLE(ros_node->get_logger(),*ros_node->get_clock(),2000,
        "DWAL_VARIABLE_R_V2: missing/stale odom/reference/map/tracks; empty output");
    }

    available_paths_publisher->publish(*msg);
    visualization_msgs::msg::MarkerArray available_markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id="odom"; marker.header.stamp=ros_node->now();
    marker.ns="dwal_available"; marker.action=visualization_msgs::msg::Marker::DELETEALL;
    available_markers.markers.push_back(marker);
    marker.action=visualization_msgs::msg::Marker::ADD;
    marker.type=visualization_msgs::msg::Marker::LINE_STRIP;
    marker.pose.orientation.w=1.0; marker.scale.x=0.012;
    marker.color.r=marker.color.g=marker.color.b=0.6; marker.color.a=0.6;
    for (size_t i=0;i<msg->paths.size();++i) {
      marker.id=static_cast<int>(i);marker.points.clear();
      for (const auto & pose : msg->paths[i].poses) {
        geometry_msgs::msg::Point point;point.x=pose.x;point.y=pose.y;marker.points.push_back(point);
      }
      available_markers.markers.push_back(marker);
    }
    available_markers_publisher->publish(available_markers);

    if (do_slice)
    {
      Clustering & src_cluster = group_objects[static_cast<size_t>(clustering_order[0])];
      Clustering & trg_cluster = group_objects[static_cast<size_t>(clustering_order[1])];

      src_cluster.doClustering(msg); //do src group clustering
      trg_cluster.source_slice_group_ptr = &src_cluster.group;

      // check to see if prev src frame was empty
      if (!src_cluster.corresp_matrix.empty())
      {
        trg_cluster.source_slice_cluster_index =
          static_cast<int>(src_cluster.corresp_matrix[static_cast<unsigned int>(trg_cluster.source_slice_cluster_id)]);
        trg_cluster.source_slice_cluster_id =
          src_cluster.group.clusters[static_cast<size_t>(trg_cluster.source_slice_cluster_index)].id;
      }
      else
        trg_cluster.source_slice_cluster_index = -1;

      trg_cluster.doClustering(msg); //do target group clustering

      for (size_t k = 2; k < group_num; k++)
        group_objects[static_cast<size_t>(clustering_order[static_cast<size_t>(k)])].doClustering(msg);
    }
    else
    {
      for (size_t k = 0; k < group_num; k++)
        group_objects[static_cast<size_t>(clustering_order[k])].doClustering(msg);
    }
    double rmin=std::numeric_limits<double>::infinity(),rmax=0;
    size_t clusters=0;
    for (const auto & group : group_objects) for (const auto & cluster : group.group.clusters) {
      rmin=std::min(rmin,cluster.r); rmax=std::max(rmax,cluster.r); ++clusters;
    }
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    RCLCPP_INFO_THROTTLE(ros_node->get_logger(),*ros_node->get_clock(),2000,
      "DWAL_VARIABLE_R_V2: candidates=%zu clusters=%zu r=[%.3f, %.3f] cycle=%.1f ms",
      msg->paths.size(),clusters,clusters ? rmin : 0.0,rmax,ms);
  }


  // ROS2 entities
  rclcpp::Subscription<dwal_planner::msg::SampledCluster>::SharedPtr sampled_cluster_subscriber;
  rclcpp::Service<dwal_planner::srv::ToggleClusters>::SharedPtr toggleSpinService;
  rclcpp::Service<dwal_planner::srv::ToggleSlice>::SharedPtr toggleSliceService;

  rclcpp::Publisher<dwal_planner::msg::SampledCluster>::SharedPtr available_paths_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr available_markers_publisher;
  BrakingFilter brake_filter;

  // State kept the same
  std::vector<int> spin, clustering_order;
  std::vector<int64_t> spin64;
  std::vector<Clustering> group_objects;
  bool do_slice{false};
  size_t group_num{0};
  std::vector<std::string> postfixes;
  std::vector<double> levels;
  std::string markerTopic, clusterTopic;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dwal_clustering");

  ControlClass ctrlObj(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
