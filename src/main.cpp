#include "lobb_weld/demo_cloud.hpp"
#include "lobb_weld/extractor.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
  std::string input;
  std::string output = "output";
  std::string demo;
  lobb::ExtractParams params;
};

void printUsage() {
  std::cout
      << "用法:\n"
      << "  lobb_weld --input 工件.pcd --output out\n"
      << "  lobb_weld --demo sine --output out/sine\n"
      << "  lobb_weld --demo pipe --output out/pipe\n"
      << "\n"
      << "按局部有向包围盒（LOBB）特征、非线性激活和分层 K-means 提取曲面焊缝。\n"
      << "可选参数:\n"
      << "  --radius <值>    LOBB 邻域半径，默认按点间距自动估计\n"
      << "  --voxel <值>     体素降采样边长，默认不降采样\n"
      << "  --no-sor         关闭统计离群点滤波\n";
}

bool parseArgs(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const std::string& name) {
      if (i + 1 >= argc) {
        throw std::runtime_error("缺少参数值: " + name);
      }
      return std::string(argv[++i]);
    };
    if (arg == "--help" || arg == "-h") {
      printUsage();
      return false;
    } else if (arg == "--input") {
      options.input = need(arg);
    } else if (arg == "--output") {
      options.output = need(arg);
    } else if (arg == "--demo") {
      options.demo = need(arg);
    } else if (arg == "--radius") {
      options.params.feature_radius = std::stof(need(arg));
    } else if (arg == "--voxel") {
      options.params.voxel = std::stof(need(arg));
    } else if (arg == "--no-sor") {
      options.params.statistical_filter = false;
    } else {
      throw std::runtime_error("无法识别的参数: " + arg);
    }
  }
  if (options.input.empty() && options.demo.empty()) {
    printUsage();
    return false;
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCloud(const std::string& path) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const std::string ext = std::filesystem::path(path).extension().string();
  int code = -1;
  if (ext == ".ply" || ext == ".PLY") {
    code = pcl::io::loadPLYFile(path, *cloud);
  } else {
    code = pcl::io::loadPCDFile(path, *cloud);
  }
  if (code < 0 || cloud->empty()) {
    throw std::runtime_error("无法读取点云: " + path);
  }
  return cloud;
}

void saveCloud(const std::string& path, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  if (cloud.empty()) {
    std::ofstream out(path);
    if (!out) {
      throw std::runtime_error("无法写入点云: " + path);
    }
    out << "# .PCD v0.7 - Point Cloud Data file format\n"
           "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
           "WIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA binary\n";
    return;
  }
  if (pcl::io::savePCDFileBinary(path, cloud) < 0) {
    throw std::runtime_error("无法写入点云: " + path);
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorByLabel(const lobb::ExtractResult& result) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->reserve(result.cloud->size());
  for (size_t i = 0; i < result.cloud->size(); ++i) {
    pcl::PointXYZRGB point;
    point.x = result.cloud->points[i].x;
    point.y = result.cloud->points[i].y;
    point.z = result.cloud->points[i].z;
    switch (result.labels[i]) {
      case lobb::PointLabel::Boundary:
        point.r = 40;
        point.g = 120;
        point.b = 240;
        break;
      case lobb::PointLabel::Crease:
        point.r = 220;
        point.g = 40;
        point.b = 40;
        break;
      case lobb::PointLabel::Corner:
        point.r = 250;
        point.g = 190;
        point.b = 20;
        break;
      case lobb::PointLabel::Plane:
      default:
        point.r = 185;
        point.g = 185;
        point.b = 185;
        break;
    }
    colored->push_back(point);
  }
  colored->width = static_cast<uint32_t>(colored->size());
  colored->height = 1;
  return colored;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr collect(const lobb::ExtractResult& result, lobb::PointLabel label) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < result.labels.size(); ++i) {
    if (result.labels[i] == label) {
      cloud->push_back(result.cloud->points[i]);
    }
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  return cloud;
}

void saveSeam(const std::filesystem::path& path, const lobb::SeamCurve& seam) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("无法写入轨迹: " + path.string());
  }
  out << "x,y,z,tangent_x,tangent_y,tangent_z,approach_x,approach_y,approach_z,"
         "binormal_x,binormal_y,binormal_z\n";
  out.setf(std::ios::fixed);
  out.precision(6);
  for (const auto& pose : seam.poses) {
    out << pose.position.x() << ',' << pose.position.y() << ',' << pose.position.z() << ','
        << pose.tangent.x() << ',' << pose.tangent.y() << ',' << pose.tangent.z() << ','
        << pose.approach.x() << ',' << pose.approach.y() << ',' << pose.approach.z() << ','
        << pose.binormal.x() << ',' << pose.binormal.y() << ',' << pose.binormal.z() << '\n';
  }
}

void writePreview(const std::filesystem::path& path, const lobb::ExtractResult& result) {
  if (!result.cloud || result.cloud->empty()) {
    return;
  }
  float min_x = result.cloud->points.front().x;
  float max_x = min_x;
  float min_y = result.cloud->points.front().y;
  float max_y = min_y;
  float min_z = result.cloud->points.front().z;
  float max_z = min_z;
  for (const auto& p : result.cloud->points) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
    min_z = std::min(min_z, p.z);
    max_z = std::max(max_z, p.z);
  }
  const int width = 720;
  const int panel = 520;
  const int height = panel * 2 + 16;
  std::vector<unsigned char> image(static_cast<size_t>(width * height * 3), 255);

  auto paint = [&](int x, int y, unsigned char r, unsigned char g, unsigned char b, int radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int xx = x + dx;
        const int yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= width || yy >= height) {
          continue;
        }
        const size_t offset = static_cast<size_t>((yy * width + xx) * 3);
        image[offset] = r;
        image[offset + 1] = g;
        image[offset + 2] = b;
      }
    }
  };

  auto map_point = [&](float a, float b, float a0, float a1, float b0, float b1, int y_origin) {
    const float sa = std::max(1e-6f, a1 - a0);
    const float sb = std::max(1e-6f, b1 - b0);
    const int x = static_cast<int>((a - a0) / sa * (width - 40) + 20);
    const int y = y_origin + static_cast<int>((1.f - (b - b0) / sb) * (panel - 40) + 20);
    return std::pair<int, int>(x, y);
  };

  for (size_t i = 0; i < result.cloud->size(); ++i) {
    unsigned char r = 185, g = 185, b = 185;
    switch (result.labels[i]) {
      case lobb::PointLabel::Boundary:
        r = 40;
        g = 120;
        b = 240;
        break;
      case lobb::PointLabel::Crease:
        r = 210;
        g = 40;
        b = 40;
        break;
      case lobb::PointLabel::Corner:
        r = 240;
        g = 170;
        b = 20;
        break;
      default:
        break;
    }
    const auto& p = result.cloud->points[i];
    const auto xy = map_point(p.x, p.y, min_x, max_x, min_y, max_y, 0);
    paint(xy.first, xy.second, r, g, b, 1);
    const auto xz = map_point(p.x, p.z, min_x, max_x, min_z, max_z, panel + 16);
    paint(xz.first, xz.second, r, g, b, 1);
  }
  for (const auto& seam : result.seams) {
    for (const auto& pose : seam.poses) {
      const auto xy = map_point(static_cast<float>(pose.position.x()), static_cast<float>(pose.position.y()),
                                min_x, max_x, min_y, max_y, 0);
      paint(xy.first, xy.second, 20, 170, 60, 2);
      const auto xz = map_point(static_cast<float>(pose.position.x()), static_cast<float>(pose.position.z()),
                                min_x, max_x, min_z, max_z, panel + 16);
      paint(xz.first, xz.second, 20, 170, 60, 2);
    }
  }

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    throw std::runtime_error("无法写入预览图: " + path.string());
  }
  out << "P6\n" << width << " " << height << "\n255\n";
  out.write(reinterpret_cast<const char*>(image.data()), static_cast<std::streamsize>(image.size()));
}

void saveOutputs(const std::filesystem::path& directory, const lobb::ExtractResult& result) {
  std::filesystem::create_directories(directory);
  saveCloud((directory / "preprocessed.pcd").string(), *result.cloud);
  pcl::io::savePCDFileBinary((directory / "classes.pcd").string(), *colorByLabel(result));
  saveCloud((directory / "crease.pcd").string(), *collect(result, lobb::PointLabel::Crease));
  saveCloud((directory / "boundary.pcd").string(), *collect(result, lobb::PointLabel::Boundary));
  saveCloud((directory / "corners.pcd").string(), *collect(result, lobb::PointLabel::Corner));

  pcl::PointCloud<pcl::PointXYZ>::Ptr seam_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t s = 0; s < result.seams.size(); ++s) {
    const auto file = directory / (s == 0 ? "seam.csv" : "seam_" + std::to_string(s) + ".csv");
    saveSeam(file, result.seams[s]);
    for (const auto& pose : result.seams[s].poses) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(pose.position.x());
      point.y = static_cast<float>(pose.position.y());
      point.z = static_cast<float>(pose.position.z());
      seam_cloud->push_back(point);
    }
  }
  seam_cloud->width = static_cast<uint32_t>(seam_cloud->size());
  seam_cloud->height = 1;
  saveCloud((directory / "seam.pcd").string(), *seam_cloud);
  writePreview(directory / "preview.ppm", result);
}

int runDemo(const Options& options) {
  lobb::DemoCloud demo;
  if (options.demo == "sine") {
    demo = lobb::makeSineFilletCloud();
  } else if (options.demo == "pipe") {
    demo = lobb::makePipePlateCloud();
  } else {
    throw std::runtime_error("未知演示: " + options.demo + "（可选 sine 或 pipe）");
  }
  std::cout << "演示 " << demo.name << "，点数 " << demo.cloud->size() << "\n";
  const lobb::ExtractResult result = lobb::extractCurvedSeam(demo.cloud, options.params);
  saveOutputs(options.output, result);
  saveCloud((std::filesystem::path(options.output) / "input.pcd").string(), *demo.cloud);

  std::vector<Eigen::Vector3d> seam_points;
  if (!result.seams.empty()) {
    for (const auto& pose : result.seams.front().poses) {
      seam_points.push_back(pose.position);
    }
  }
  const lobb::CurveScore score = lobb::scoreCurve(seam_points, demo.ground_truth, demo.coverage_distance);
  std::cout << "相对真值: 平均 " << score.mean << "，P90 " << score.p90 << "，覆盖率 " << score.coverage << "\n";
  if (score.mean > demo.max_mean_error || score.p90 > demo.max_p90_error || score.coverage < demo.min_coverage) {
    std::cout << "演示未达到精度要求（平均 <= " << demo.max_mean_error << "，P90 <= " << demo.max_p90_error
              << "，覆盖率 >= " << demo.min_coverage << "）。\n";
    return 1;
  }
  std::cout << "演示通过。结果写在 " << options.output << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options options;
    if (!parseArgs(argc, argv, options)) {
      return 2;
    }
    if (!options.demo.empty()) {
      return runDemo(options);
    }
    const auto cloud = loadCloud(options.input);
    std::cout << "读取 " << options.input << "，点数 " << cloud->size() << "\n";
    const lobb::ExtractResult result = lobb::extractCurvedSeam(cloud, options.params);
    saveOutputs(options.output, result);
    if (result.seams.empty()) {
      std::cout << "没有提取到焊缝。可以增大 --radius，使邻域同时覆盖焊缝两侧。\n";
      return 1;
    }
    std::cout << "结果写在 " << options.output << "\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
