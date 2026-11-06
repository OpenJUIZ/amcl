// End-to-end check of the library: a robot drives around a synthetic room, the laser is
// simulated with map_calc_range, the odometry drifts, and the filter (started near the true
// pose) must track the robot. This mirrors what the ROS amcl node does with these APIs.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_laser.h"
#include "amcl/sensors/amcl_odom.h"

namespace {

const double kPi = 3.14159265358979323846;

// 10 m x 8 m room with an L-shaped block so that it is not symmetric.
map_t* make_room() {
  map_t* map = map_alloc();
  map->scale = 0.05;
  map->size_x = 200;
  map->size_y = 160;
  // origin_x/origin_y is the world position of the map centre (as the ROS node sets it).
  map->origin_x = 5.0;
  map->origin_y = 4.0;
  map->cells = static_cast<map_cell_t*>(calloc(map->size_x * map->size_y, sizeof(map_cell_t)));
  for (int j = 0; j < map->size_y; ++j) {
    for (int i = 0; i < map->size_x; ++i) {
      const double x = MAP_WXGX(map, i);
      const double y = MAP_WYGY(map, j);
      const bool wall = i < 2 || j < 2 || i >= map->size_x - 2 || j >= map->size_y - 2;
      const bool block = (x > 6.0 && x < 7.0 && y > 2.0 && y < 5.0) || (x > 6.0 && x < 8.5 && y > 4.0 && y < 5.0);
      const bool pillar = std::hypot(x - 2.5, y - 6.0) < 0.3;
      map->cells[MAP_INDEX(map, i, j)].occ_state = (wall || block || pillar) ? +1 : -1;
    }
  }
  return map;
}

struct Pose {
  double x, y, a;
};

double angle_diff(double a, double b) {
  double d = std::fmod(a - b + kPi, 2 * kPi);
  if (d < 0) d += 2 * kPi;
  return d - kPi;
}

}  // namespace

int main() {
  map_t* map = make_room();
  map_update_cspace(map, 2.0);

  pf_t* pf = pf_alloc(500, 5000, 0.0, 0.0, nullptr, nullptr);
  pf->pop_err = 0.01;
  pf->pop_z = 0.99;

  amcl::AMCLOdom odom;
  odom.SetModel(amcl::ODOM_MODEL_DIFF, 0.2, 0.2, 0.2, 0.2);
  amcl::AMCLLaser laser(30, map);
  laser.SetModelLikelihoodField(0.95, 0.05, 0.2, 2.0);
  pf_vector_t laser_pose = pf_vector_zero();
  laser.SetLaserPose(laser_pose);

  // True start pose and an initial guess 0.3 m / 0.2 rad off.
  Pose truth{2.0, 2.0, 0.0};
  pf_vector_t mean = pf_vector_zero();
  mean.v[0] = truth.x + 0.3;
  mean.v[1] = truth.y - 0.2;
  mean.v[2] = truth.a + 0.2;
  pf_matrix_t cov = pf_matrix_zero();
  cov.m[0][0] = 0.25;
  cov.m[1][1] = 0.25;
  cov.m[2][2] = (kPi / 12) * (kPi / 12);
  pf_init(pf, mean, cov);

  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 1.0);

  // Waypoints of a loop through the free space of the room.
  const std::vector<Pose> route = {{2.0, 2.0, 0}, {5.0, 1.5, 0}, {8.5, 2.0, 0}, {8.5, 6.5, 0},
                                   {4.0, 6.8, 0}, {1.2, 5.0, 0}, {1.5, 2.0, 0}};
  Pose odom_pose = truth;
  Pose last_update_odom = odom_pose;
  bool first = true;
  int updates = 0;
  double worst = 0.0, last_err = 0.0, odom_err = 0.0;

  const int n_beams = 181;
  for (size_t w = 1; w < route.size(); ++w) {
    for (int k = 0; k < 400; ++k) {
      // Drive towards the next waypoint: turn then go.
      const double dx = route[w].x - truth.x, dy = route[w].y - truth.y;
      if (std::hypot(dx, dy) < 0.05) break;
      const double heading = std::atan2(dy, dx);
      const double turn = angle_diff(heading, truth.a);
      double v = 0.0, om = 0.0;
      if (std::fabs(turn) > 0.05) om = std::copysign(std::min(0.15, std::fabs(turn)), turn);
      else v = std::min(0.05, std::hypot(dx, dy));
      truth.a += om;
      truth.x += v * std::cos(truth.a);
      truth.y += v * std::sin(truth.a);
      // Odometry with 5 % multiplicative noise and a small heading bias.
      const double v_o = v * (1.0 + 0.05 * noise(rng));
      const double om_o = om * (1.0 + 0.05 * noise(rng)) + 0.002 * v;
      odom_pose.a += om_o;
      odom_pose.x += v_o * std::cos(odom_pose.a);
      odom_pose.y += v_o * std::sin(odom_pose.a);

      // Update only after moving 0.2 m or 30 deg, like the ROS node.
      pf_vector_t delta = pf_vector_zero();
      delta.v[0] = odom_pose.x - last_update_odom.x;
      delta.v[1] = odom_pose.y - last_update_odom.y;
      delta.v[2] = angle_diff(odom_pose.a, last_update_odom.a);
      if (!first && std::hypot(delta.v[0], delta.v[1]) < 0.2 && std::fabs(delta.v[2]) < kPi / 6) continue;

      amcl::AMCLOdomData odata;
      odata.pose.v[0] = odom_pose.x;
      odata.pose.v[1] = odom_pose.y;
      odata.pose.v[2] = odom_pose.a;
      odata.delta = delta;
      if (!first) odom.UpdateAction(pf, &odata);
      first = false;
      last_update_odom = odom_pose;

      amcl::AMCLLaserData ldata;
      ldata.sensor = &laser;
      ldata.range_count = n_beams;
      ldata.range_max = 8.0;
      ldata.ranges = new double[n_beams][2];
      for (int i = 0; i < n_beams; ++i) {
        const double bearing = -kPi / 2 + kPi * i / (n_beams - 1);
        double r = map_calc_range(map, truth.x, truth.y, truth.a + bearing, ldata.range_max);
        if (r < ldata.range_max) r += 0.01 * noise(rng);
        ldata.ranges[i][0] = r;
        ldata.ranges[i][1] = bearing;
      }
      laser.UpdateSensor(pf, &ldata);
      pf_update_resample(pf);
      ++updates;

      // Best cluster, as the ROS node reports it.
      double best_w = -1.0;
      pf_vector_t best = pf_vector_zero();
      pf_sample_set_t* set = pf->sets + pf->current_set;
      for (int c = 0; c < set->cluster_count; ++c) {
        double weight;
        pf_vector_t m;
        pf_matrix_t cv;
        if (!pf_get_cluster_stats(pf, c, &weight, &m, &cv)) break;
        if (weight > best_w) {
          best_w = weight;
          best = m;
        }
      }
      last_err = std::hypot(best.v[0] - truth.x, best.v[1] - truth.y);
      odom_err = std::hypot(odom_pose.x - truth.x, odom_pose.y - truth.y);
      if (updates > 10 && last_err > worst) worst = last_err;
    }
  }

  std::printf("updates %d, samples %d, final error %.3f m (odometry %.3f m), worst after 10 updates %.3f m\n", updates,
              pf->sets[pf->current_set].sample_count, last_err, odom_err, worst);
  pf_free(pf);
  map_free(map);

  bool ok = updates > 50 && last_err < 0.15 && worst < 0.3;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
