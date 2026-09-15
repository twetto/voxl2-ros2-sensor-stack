// Offline, single-threaded OpenVINS runner for decoded VOXL2 bags.
//
// Feeds ov_msckf::VioManager (the Humble build in ~/openvins_ws_foxy/install_humble)
// exactly as ROS2Visualizer does in run_subscribe_msckf, but serially and ordered by
// header stamp, so a run is repeatable:
//   - IMU: feed_measurement_imu, then process every queued camera frame whose stamp is
//     older than (imu stamp - calib_dt_CAMtoIMU), in stamp order.
//   - Camera: dropped if closer than 1/track_frequency to the last accepted frame
//     (ROS2Visualizer::callback_monocular), else queued with an all-zero mask.
// Inputs come from decode_bag.py (frames.u8, cam_t.txt, imu.csv); stamps are the
// original bag header stamps, never re-stamped.
//
// Output: TUM lines "t x y z qx qy qz qw" after every camera update once initialised:
//   t = state->_timestamp + calib_dt_CAMtoIMU (IMU clock, as publish_state),
//   p = p_IinG, q = Hamilton q(I->G), which is numerically OpenVINS' JPL q_GtoI.
// A second file (<out>.state) adds velocity, biases and track counts for diagnosis.
//
// Usage: ov_serial <estimator_config.yaml> <decoded_dir> <out.tum>
#include <algorithm>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "types/Vec.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

struct Ev {
  long long t_ns;
  int kind; // 0 imu, 1 cam
  size_t idx;
};

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "usage: ov_serial <estimator_config.yaml> <decoded_dir> <out.tum>\n";
    return 2;
  }
  const std::string config_path = argv[1], dir = argv[2], out_path = argv[3];

  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false; // initialiser runs inline (as ros1_serial_msckf)
  params.use_multi_threading_pubs = false;
  if (!parser->successful()) {
    std::cerr << "[SERIAL] unable to parse all parameters\n";
    return 1;
  }
  auto sys = std::make_shared<VioManager>(params);

  // ---- load IMU
  std::vector<long long> imu_t;
  std::vector<ov_core::ImuData> imu;
  {
    std::ifstream f(dir + "/imu.csv");
    std::string line;
    while (std::getline(f, line)) {
      long long s, ns;
      double v[6];
      if (std::sscanf(line.c_str(), "%lld,%lld,%lf,%lf,%lf,%lf,%lf,%lf", &s, &ns, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 8)
        continue;
      ov_core::ImuData m;
      m.timestamp = s + ns * 1e-9;
      m.wm << v[0], v[1], v[2];
      m.am << v[3], v[4], v[5];
      imu_t.push_back(s * 1000000000LL + ns);
      imu.push_back(m);
    }
  }
  // ---- load camera stamps; frames are read lazily from frames.u8
  std::vector<long long> cam_t;
  std::vector<std::pair<long long, long long>> cam_sn;
  {
    std::ifstream f(dir + "/cam_t.txt");
    long long s, ns;
    while (f >> s >> ns) {
      cam_t.push_back(s * 1000000000LL + ns);
      cam_sn.push_back({s, ns});
    }
  }
  const int W = 1280, H = 800;
  std::ifstream frames(dir + "/frames.u8", std::ios::binary);
  if (!frames || imu.empty() || cam_t.empty()) {
    std::cerr << "[SERIAL] missing input in " << dir << "\n";
    return 1;
  }
  std::printf("[SERIAL] %zu imu, %zu frames\n", imu.size(), cam_t.size());

  // ---- merge by header stamp (IMU first on ties, like a sensor arriving before the frame)
  std::vector<Ev> ev;
  ev.reserve(imu.size() + cam_t.size());
  for (size_t i = 0; i < imu.size(); i++)
    ev.push_back({imu_t[i], 0, i});
  for (size_t i = 0; i < cam_t.size(); i++)
    ev.push_back({cam_t[i], 1, i});
  std::stable_sort(ev.begin(), ev.end(), [](const Ev &a, const Ev &b) { return a.t_ns != b.t_ns ? a.t_ns < b.t_ns : a.kind < b.kind; });

  std::ofstream tum(out_path), stf(out_path + ".state");
  tum.setf(std::ios::fixed);
  stf.setf(std::ios::fixed);
  stf << "# t px py pz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz n_slam dt_cam_imu\n";

  std::deque<ov_core::CameraData> queue;
  const double time_delta = 1.0 / params.track_frequency;
  double cam_last = -1;
  size_t n_dropped_rate = 0, n_updates = 0, n_fed = 0;
  double last_written = -1, t_init = -1;
  bool was_init = false;

  auto emit = [&]() {
    auto state = sys->get_state();
    double dt = state->_calib_dt_CAMtoIMU->value()(0);
    double t = state->_timestamp + dt;
    if (!sys->initialized() || t == last_written)
      return;
    last_written = t;
    Eigen::Vector3d p = state->_imu->pos();
    Eigen::Vector4d q = state->_imu->quat(); // JPL q_GtoI (x y z w) == Hamilton q_ItoG
    Eigen::Vector3d v = state->_imu->vel(), bg = state->_imu->bias_g(), ba = state->_imu->bias_a();
    tum << std::setprecision(9) << t << " " << std::setprecision(6) << p(0) << " " << p(1) << " " << p(2) << " " << std::setprecision(9)
        << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << "\n";
    stf << std::setprecision(9) << t << std::setprecision(6);
    for (int k = 0; k < 3; k++)
      stf << " " << p(k);
    for (int k = 0; k < 4; k++)
      stf << " " << q(k);
    for (int k = 0; k < 3; k++)
      stf << " " << v(k);
    for (int k = 0; k < 3; k++)
      stf << " " << bg(k);
    for (int k = 0; k < 3; k++)
      stf << " " << ba(k);
    stf << " " << state->_features_SLAM.size() << " " << std::setprecision(7) << dt << "\n";
    n_updates++;
  };

  std::vector<unsigned char> buf((size_t)W * H);
  for (const Ev &e : ev) {
    if (e.kind == 0) {
      sys->feed_measurement_imu(imu[e.idx]);
      double t_imu_inC = imu[e.idx].timestamp - sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
      while (!queue.empty() && queue.front().timestamp < t_imu_inC) {
        sys->feed_measurement_camera(queue.front());
        queue.pop_front();
        n_fed++;
        if (sys->initialized() && !was_init) {
          was_init = true;
          t_init = sys->get_state()->_timestamp;
          std::printf("[SERIAL] initialised at t=%.6f (frame %zu fed)\n", t_init, n_fed);
        }
        emit();
      }
    } else {
      double t = cam_sn[e.idx].first + cam_sn[e.idx].second * 1e-9;
      if (cam_last >= 0 && t < cam_last + time_delta) {
        n_dropped_rate++;
        continue;
      }
      cam_last = t;
      frames.seekg((std::streamoff)e.idx * W * H);
      frames.read((char *)buf.data(), (std::streamsize)buf.size());
      ov_core::CameraData msg;
      msg.timestamp = t;
      msg.sensor_ids.push_back(0);
      msg.images.push_back(cv::Mat(H, W, CV_8UC1, buf.data()).clone());
      msg.masks.push_back(cv::Mat::zeros(H, W, CV_8UC1));
      queue.push_back(msg);
    }
  }
  std::printf("[SERIAL] done: frames %zu, dropped by track_frequency %zu, fed %zu, unprocessed %zu, poses written %zu, init t=%.6f\n",
              cam_t.size(), n_dropped_rate, n_fed, queue.size(), n_updates, t_init);
  return 0;
}
