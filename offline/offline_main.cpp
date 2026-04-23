#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;

struct ImuSample {
  double t = 0.0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
};

struct Pose {
  double t = 0.0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

static void PrintUsage(const char* bin) {
  std::cout << "Usage:\n"
            << "  " << bin
            << " --scan_dir <dir> --imu_csv <file> --output_dir <dir> [--voxel 0.2] [--min_range 1.0]\n\n"
            << "Input scan files must be *.pcd and use filename stem as timestamp (seconds), e.g. 1686032450.123.pcd\n"
            << "IMU csv format: timestamp,ax,ay,az,gx,gy,gz\n";
}

static bool ParseArgs(int argc, char** argv, std::string& scan_dir, std::string& imu_csv,
                      std::string& output_dir, double& voxel_size, double& min_range) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto read_value = [&](std::string& out) -> bool {
      if (i + 1 >= argc) return false;
      out = argv[++i];
      return true;
    };
    if (arg == "--scan_dir") {
      if (!read_value(scan_dir)) return false;
    } else if (arg == "--imu_csv") {
      if (!read_value(imu_csv)) return false;
    } else if (arg == "--output_dir") {
      if (!read_value(output_dir)) return false;
    } else if (arg == "--voxel") {
      std::string val;
      if (!read_value(val)) return false;
      voxel_size = std::stod(val);
    } else if (arg == "--min_range") {
      std::string val;
      if (!read_value(val)) return false;
      min_range = std::stod(val);
    } else if (arg == "--help" || arg == "-h") {
      return false;
    }
  }

  return !scan_dir.empty() && !imu_csv.empty() && !output_dir.empty();
}

static std::vector<ImuSample> LoadImuCsv(const std::string& path) {
  std::vector<ImuSample> data;
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Failed to open imu csv: " + path);
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    if (!std::isdigit(line.front()) && line.front() != '-' && line.front() != '+') {
      continue;
    }

    std::stringstream ss(line);
    std::string token;
    std::vector<double> vals;
    while (std::getline(ss, token, ',')) {
      vals.push_back(std::stod(token));
    }
    if (vals.size() < 7) continue;

    ImuSample s;
    s.t = vals[0];
    s.acc = Eigen::Vector3d(vals[1], vals[2], vals[3]);
    s.gyr = Eigen::Vector3d(vals[4], vals[5], vals[6]);
    data.push_back(s);
  }

  std::sort(data.begin(), data.end(), [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });
  return data;
}

static std::vector<std::pair<double, fs::path>> LoadScans(const std::string& scan_dir) {
  std::vector<std::pair<double, fs::path>> scans;
  for (const auto& e : fs::directory_iterator(scan_dir)) {
    if (!e.is_regular_file() || e.path().extension() != ".pcd") continue;

    try {
      const double t = std::stod(e.path().stem().string());
      scans.emplace_back(t, e.path());
    } catch (...) {
      std::cerr << "[WARN] Skip non-timestamp file: " << e.path() << std::endl;
    }
  }
  std::sort(scans.begin(), scans.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  return scans;
}

static Eigen::Quaterniond DeltaQ(const Eigen::Vector3d& w, double dt) {
  const Eigen::Vector3d theta = w * dt;
  const double angle = theta.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, theta / angle));
}

static Pose Integrate(const Pose& start, const std::vector<ImuSample>& imu, size_t& imu_idx, double t_end) {
  Pose pose = start;
  const Eigen::Vector3d g(0, 0, -9.81);

  while (imu_idx + 1 < imu.size() && imu[imu_idx + 1].t <= t_end) {
    const ImuSample& s0 = imu[imu_idx];
    const ImuSample& s1 = imu[imu_idx + 1];
    const double dt = s1.t - std::max(pose.t, s0.t);
    if (dt > 0) {
      pose.q = (pose.q * DeltaQ(s0.gyr, dt)).normalized();
      const Eigen::Vector3d a_world = pose.q * s0.acc + g;
      pose.p += pose.v * dt + 0.5 * a_world * dt * dt;
      pose.v += a_world * dt;
      pose.t += dt;
    }
    ++imu_idx;
  }

  if (imu_idx < imu.size()) {
    const double dt = t_end - pose.t;
    if (dt > 0) {
      pose.q = (pose.q * DeltaQ(imu[imu_idx].gyr, dt)).normalized();
      const Eigen::Vector3d a_world = pose.q * imu[imu_idx].acc + g;
      pose.p += pose.v * dt + 0.5 * a_world * dt * dt;
      pose.v += a_world * dt;
      pose.t += dt;
    }
  }

  return pose;
}

int main(int argc, char** argv) {
  std::string scan_dir;
  std::string imu_csv;
  std::string output_dir;
  double voxel_size = 0.2;
  double min_range = 1.0;

  if (!ParseArgs(argc, argv, scan_dir, imu_csv, output_dir, voxel_size, min_range)) {
    PrintUsage(argv[0]);
    return 1;
  }

  fs::create_directories(output_dir);

  const auto imu = LoadImuCsv(imu_csv);
  const auto scans = LoadScans(scan_dir);
  if (imu.empty() || scans.empty()) {
    std::cerr << "No valid imu/scans found." << std::endl;
    return 2;
  }

  Pose pose;
  pose.t = scans.front().first;

  PointCloud::Ptr global_map(new PointCloud());
  std::ofstream traj(fs::path(output_dir) / "trajectory.txt");
  traj << std::fixed << std::setprecision(9);

  size_t imu_idx = 0;
  while (imu_idx + 1 < imu.size() && imu[imu_idx + 1].t <= pose.t) {
    ++imu_idx;
  }

  for (size_t i = 0; i < scans.size(); ++i) {
    const double t = scans[i].first;
    pose = Integrate(pose, imu, imu_idx, t);

    PointCloud::Ptr scan(new PointCloud());
    if (pcl::io::loadPCDFile<PointType>(scans[i].second.string(), *scan) != 0) {
      std::cerr << "[WARN] Failed to load scan: " << scans[i].second << std::endl;
      continue;
    }

    PointCloud::Ptr filtered(new PointCloud());
    filtered->reserve(scan->size());
    const double min_range_sq = min_range * min_range;
    for (const auto& p : scan->points) {
      const double r2 = p.x * p.x + p.y * p.y + p.z * p.z;
      if (r2 >= min_range_sq) filtered->push_back(p);
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = pose.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = pose.p.cast<float>();

    PointCloud::Ptr transformed(new PointCloud());
    pcl::transformPointCloud(*filtered, *transformed, T);
    *global_map += *transformed;

    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.setInputCloud(global_map);
    PointCloud::Ptr down(new PointCloud());
    vg.filter(*down);
    global_map.swap(down);

    traj << t << ' ' << pose.p.x() << ' ' << pose.p.y() << ' ' << pose.p.z() << ' '
         << pose.q.x() << ' ' << pose.q.y() << ' ' << pose.q.z() << ' ' << pose.q.w() << '\n';

    std::cout << "Processed " << (i + 1) << "/" << scans.size() << " scans. map points=" << global_map->size() << std::endl;
  }

  const std::string map_path = (fs::path(output_dir) / "map.pcd").string();
  pcl::io::savePCDFileBinary(map_path, *global_map);
  std::cout << "Saved map: " << map_path << std::endl;
  std::cout << "Saved trajectory: " << (fs::path(output_dir) / "trajectory.txt") << std::endl;
  return 0;
}
