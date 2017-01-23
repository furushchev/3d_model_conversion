/*
 * main.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <pcl/filters/crop_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include <random>

namespace po = boost::program_options;

void print_usage(std::string prog) {
  std::cout << prog << " [options] [mesh input file] [pcd output file]"
            << std::endl << std::endl
            << "Options:" << std::endl
            << "  -h        Print this help" << std::endl
            << "  -d        Density of interpolation" << std::endl
            << "  -v        Show Viewer" << std::endl;
}

bool check_ext(const std::string &filename, const std::string &ext) {
  return filename.substr(filename.find_last_of(".") + 1) == ext;
}

std::string replace_ext(const std::string &filename, const std::string &ext) {
  return filename.substr(0, filename.find_last_of(".")) + "." + ext;
}

int main(int argc, char** argv) {

  std::string program_name(argv[0]);

  auto opt = po::options_description{"Options"};
  opt.add_options()
    ("help,h", "Show this help")
    ("viewer,v", "Show viewer")
    ("interpolate,i", "Enable interpolation")
    ("points,p", po::value<int>()->default_value(2000), "Interpolated point set (default: 2000)")
    ("input", po::value<std::string>(), "input mesh file path")
    ("output", po::value<std::string>(), "output pcd file path");
  auto vm = po::variables_map{};
  po::store(po::command_line_parser{argc, argv}
             .options(opt)
             .positional(po::positional_options_description{}
                          .add("input", 1)
                          .add("output", 1))
             .run(), vm);
  po::notify(vm);

  if (vm.count("help") ||
      !vm.count("input") ||
      !vm.count("output")) {
    print_usage(program_name);
    return 1;
  }

  auto visualize = vm["viewer"].as<bool>();
  auto interpolate = vm["interpolate"].as<bool>();
  auto points_num = vm["points"].as<int>();
  auto input_name = vm["input"].as<std::string>();
  auto output_name = vm["output"].as<std::string>();

  std::cout << "Converting from " << input_name << " to " << output_name << std::endl;

  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPolygonFile(input_name, *mesh) == -1) {
    std::cerr << "Error: failed to load polygon file" << std::endl;
    return 1;
  }

  pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

  if (interpolate) {
    pcl::PointXYZ min, max;
    pcl::PointCloud<pcl::PointXYZ>::Ptr interpolated(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &p : *cloud) {
      interpolated->push_back(p);
    }
    pcl::getMinMax3D(*cloud, min, max);

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double>
      rx(min.x, max.x),
      ry(min.y, max.y),
      rz(min.z, max.z);

    int interpolated_num = 0;
    auto chull = pcl::CropHull<pcl::PointXYZ>();
    while (interpolated_num < points_num) {
      pcl::PointXYZ p(rx(mt), ry(mt), rz(mt));
      for (auto& v : mesh->polygons) {
        if (chull.isPointIn2DPolyWithVertIndices(p,v,*cloud)){
          interpolated->push_back(p);
          interpolated_num++;
          break;
        }
      }
    }
    cloud = interpolate;
  }

  if (visualize) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Model Converter"));
    viewer->addCoordinateSystem();
    bool ok = false;
    viewer->registerKeyboardCallback([&](const pcl::visualization::KeyboardEvent& e) {
        if (e.keyDown()) {
          if (e.getKeySym() == "y") {
            std::cout << "Save Done!" << std::endl;
            viewer->close();
          } else if (e.getKeySym() == "n") {
            std::cout << "Exited without save" << std::endl;
            viewer->close();
          } else {
            std::cout << "Answer with [y/n] ? " << std::endl;
          }
        }
      });
    viewer->addPointCloud(cloud);

    std::cout << "Save this cloud to " << output_name << "? [y/n] " << std::endl;
    viewer->spin();
  } else {
    if (pcl::io::savePCDFile(output_name, *cloud) == -1) {
      std::cerr << "Error: failed to save pcd file" << std::endl;
      return 1;
    }
    std::cout << "Done." << std::endl;
  }

  return 0;
}
