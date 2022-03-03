/*
 * @Author: Jianheng Liu
 * @Date: 2021-10-24 16:46:18
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2021-12-12 14:21:09
 * @Description: Description
 */
#include "map.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

//// Colmap
//#include <colmap/util/option_manager.h>
//#include <colmap/util/string.h>

#include <boost/filesystem.hpp>

using namespace std;

struct Label
{
  /* data */
  std::string name;
  std::string category;
  cv::Scalar color;
};
map<int, Label> labels = {
    // https://github.com/mcordts/cityscapesScripts/blob/master/cityscapesscripts/helpers/labels.py
    //trainId   name    category   color
    {255, {"void", "void", {0, 0, 0}}},
    {0, {"road", "flat", {128, 64, 128}}},
    {1, {"sidewalk", "flat", {232, 35, 244}}},
    {2, {"building", "construction", {70, 70, 70}}},
    {3, {"wall", "construction", {156, 102, 102}}},
    {4, {"fence", "construction", {153, 153, 190}}},
    {5, {"pole", "object", {153, 153, 153}}},
    {6, {"traffic light", "object", {30, 170, 250}}},
    {7, {"traffic sign", "object", {0, 220, 220}}},
    {8, {"vegetation", "nature", {35, 142, 107}}},
    {9, {"terrain", "nature", {152, 251, 152}}},
    {10, {"sky", "sky", {180, 130, 70}}},
    {11, {"person", "human", {60, 20, 220}}},
    {12, {"rider", "human", {0, 0, 255}}},
    {13, {"car", "vehicle", {142, 0, 0}}},
    {14, {"truck", "vehicle", {70, 0, 0}}},
    {15, {"bus", "vehicle", {100, 60, 0}}},
    {16, {"train", "vehicle", {100, 80, 0}}},
    {17, {"motorcycle", "vehicle", {230, 0, 0}}},
    {18, {"bicycle", "vehicle", {32, 11, 119}}},
    {-1, {"license plate", "vehicle", {142, 0, 0}}}};

Map::Map(ros::NodeHandle &nh, const std::string &sensor_config_file)
{
  node_ = nh;

  line_map_pub_ = node_.advertise<visualization_msgs::Marker>("line_map", 10);

  readParameters(sensor_config_file);

  constructSemanticLineMapfromColmap();
  // pubLineMapfromTXT("/home/chrisliu/Datasets/kitti/2011_09_26_drive_0101_sync/2011_09_26/2011_09_26_drive_0101_sync/image_02/colmap/Line3D++/Line3D++__W_FULL__N_10__sigmaP_2.5__sigmaA_10__epiOverlap_0.25__kNN_10__OPTIMIZED__vis_3.txt");
}

void Map::readParameters(const std::string &sensor_config_file)
{
  cv::FileStorage fsSettings(sensor_config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  cv::FileNode fn = fsSettings["map_config"];

  HAS_MAP = static_cast<int>(fn["has_map"]);
  if (HAS_MAP)
  {
    fn["map_path"] >> MAP_PATH;
    cout << "Loading PCD file from: " << endl
         << MAP_PATH << endl;
    PCL_ASSERT_ERROR_PRINT_CHECK(
        pcl::io::loadPCDFile<pcl::PointXYZ>(MAP_PATH, *global_map_) == 0,
        "Couldn't read file\n");
    cout << "Map Loaded!" << endl;
    OCTREE_RESOLUTION = static_cast<float>(fn["octree_resolution"]);

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree =
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
    octree.setInputCloud(global_map_);
    octree.addPointsFromInputCloud();
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(global_map_);
    while (!viewer.wasStopped())
    {
    }
  }

  fsSettings.release();
}

void Map::constructSemanticLineMapfromColmap()
{
  std::string inputFolder = "/home/chrisliu/Datasets/kitti/2011_09_26_drive_0101_sync/2011_09_26/2011_09_26_drive_0101_sync/image_02/data";
  std::string sfmFolder = "/home/chrisliu/Datasets/kitti/2011_09_26_drive_0101_sync/2011_09_26/2011_09_26_drive_0101_sync/image_02/colmap";

  if (sfmFolder.length() == 0)
    sfmFolder = inputFolder;

  std::string outputFolder = sfmFolder + "/Line3D++/";

  // check if colmap result folder
  boost::filesystem::path sfm(sfmFolder);
  if (!boost::filesystem::exists(sfm))
  {
    std::cerr << "colmap result folder " << sfm << " does not exist!" << std::endl;
    return;
  }

  // create output directory
  boost::filesystem::path dir(outputFolder);
  boost::filesystem::create_directory(dir);

  L3DPP::Line3D *Line3D = new L3DPP::Line3D(outputFolder, L3D_DEF_LOAD_AND_STORE_SEGMENTS, L3D_DEF_MAX_IMG_WIDTH,
                                            L3D_DEF_MAX_NUM_SEGMENTS, true, true);

  // check if result files exist
  boost::filesystem::path sfm_cameras(sfmFolder + "/cameras.txt");
  boost::filesystem::path sfm_images(sfmFolder + "/images.txt");
  boost::filesystem::path sfm_points3D(sfmFolder + "/points3D.txt");
  if (!boost::filesystem::exists(sfm_cameras) || !boost::filesystem::exists(sfm_images) ||
      !boost::filesystem::exists(sfm_points3D))
  {
    std::cerr << "at least one of the colmap result files does not exist in sfm folder: " << sfm << std::endl;
    return;
  }

  std::cout << std::endl
            << "reading colmap result..." << std::endl;

  // read cameras.txt
  std::ifstream cameras_file;
  cameras_file.open(sfm_cameras.c_str());
  std::string cameras_line;

  std::map<unsigned int, Eigen::Matrix3d> cams_K;
  std::map<unsigned int, Eigen::Vector3d> cams_radial;
  std::map<unsigned int, Eigen::Vector2d> cams_tangential;

  while (std::getline(cameras_file, cameras_line))
  {
    // check first character for a comment (#)
    if (cameras_line.substr(0, 1).compare("#") != 0)
    {
      std::stringstream cameras_stream(cameras_line);

      unsigned int camID, width, height;
      std::string model;

      // parse essential data
      cameras_stream >> camID >> model >> width >> height;

      double fx, fy, cx, cy, k1, k2, k3, p1, p2;

      // check camera model
      if (model.compare("SIMPLE_PINHOLE") == 0)
      {
        // f,cx,cy
        cameras_stream >> fx >> cx >> cy;
        fy = fx;
        k1 = 0;
        k2 = 0;
        k3 = 0;
        p1 = 0;
        p2 = 0;
      }
      else if (model.compare("PINHOLE") == 0)
      {
        // fx,fy,cx,cy
        cameras_stream >> fx >> fy >> cx >> cy;
        k1 = 0;
        k2 = 0;
        k3 = 0;
        p1 = 0;
        p2 = 0;
      }
      else if (model.compare("SIMPLE_RADIAL") == 0)
      {
        // f,cx,cy,k
        cameras_stream >> fx >> cx >> cy >> k1;
        fy = fx;
        k2 = 0;
        k3 = 0;
        p1 = 0;
        p2 = 0;
      }
      else if (model.compare("RADIAL") == 0)
      {
        // f,cx,cy,k1,k2
        cameras_stream >> fx >> cx >> cy >> k1 >> k2;
        fy = fx;
        k3 = 0;
        p1 = 0;
        p2 = 0;
      }
      else if (model.compare("OPENCV") == 0)
      {
        // fx,fy,cx,cy,k1,k2,p1,p2
        cameras_stream >> fx >> fy >> cx >> cy >> k1 >> k2 >> p1 >> p2;
        k3 = 0;
      }
      else if (model.compare("FULL_OPENCV") == 0)
      {
        // fx,fy,cx,cy,k1,k2,p1,p2,k3[,k4,k5,k6]
        cameras_stream >> fx >> fy >> cx >> cy >> k1 >> k2 >> p1 >> p2 >> k3;
      }
      else
      {
        std::cerr << "camera model " << model << " unknown!" << std::endl;
        std::cerr << "please specify its parameters in the main_colmap.cpp in order to proceed..." << std::endl;
        return;
      }

      Eigen::Matrix3d K;
      K(0, 0) = fx;
      K(0, 1) = 0;
      K(0, 2) = cx;
      K(1, 0) = 0;
      K(1, 1) = fy;
      K(1, 2) = cy;
      K(2, 0) = 0;
      K(2, 1) = 0;
      K(2, 2) = 1;

      cams_K[camID] = K;
      cams_radial[camID] = Eigen::Vector3d(k1, k2, k3);
      cams_tangential[camID] = Eigen::Vector2d(p1, p2);
    }
  }
  cameras_file.close();

  std::cout << "found " << cams_K.size() << " cameras in [cameras.txt]" << std::endl;

  // read images.txt
  std::ifstream images_file;
  images_file.open(sfm_images.c_str());
  std::string images_line;

  std::map<unsigned int, Eigen::Matrix3d> cams_R;
  std::map<unsigned int, Eigen::Vector3d> cams_t;
  std::map<unsigned int, Eigen::Vector3d> cams_C;
  std::map<unsigned int, unsigned int> img2cam;
  std::map<unsigned int, std::string> cams_images;
  std::map<unsigned int, std::list<unsigned int>> cams_worldpoints;
  std::map<unsigned int, Eigen::Vector3d> wps_coords;
  std::vector<unsigned int> img_seq;
  unsigned int imgID, camID;

  bool first_line = true;
  while (std::getline(images_file, images_line))
  {
    // check first character for a comment (#)
    if (images_line.substr(0, 1).compare("#") != 0)
    {
      std::stringstream images_stream(images_line);
      if (first_line)
      {
        // image data
        double qw, qx, qy, qz, tx, ty, tz;
        std::string img_name;

        images_stream >> imgID >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> camID >> img_name;

        // convert rotation
        if (cams_K.find(camID) != cams_K.end())
        {
          Eigen::Matrix3d R = Line3D->rotationFromQ(qw, qx, qy, qz);
          Eigen::Vector3d t(tx, ty, tz);
          Eigen::Vector3d C = (R.transpose()) * (-1.0 * t);

          cams_R[imgID] = R;
          cams_t[imgID] = t;
          cams_C[imgID] = C;
          cams_images[imgID] = img_name;
          img2cam[imgID] = camID;
          img_seq.push_back(imgID);
        }

        first_line = false;
      }
      else
      {
        if (cams_K.find(camID) != cams_K.end())
        {
          // 2D points
          double x, y;
          std::string wpID;
          std::list<unsigned int> wps;
          bool process = true;

          while (process)
          {
            wpID = "";
            images_stream >> x >> y >> wpID;

            if (wpID.length() > 0)
            {
              int wp = atoi(wpID.c_str());

              if (wp >= 0)
              {
                wps.push_back(wp);
                wps_coords[wp] = Eigen::Vector3d(0, 0, 0);
              }
            }
            else
            {
              // end reached
              process = false;
            }
          }

          cams_worldpoints[imgID] = wps;
        }

        first_line = true;
      }
    }
  }
  images_file.close();

  std::cout << "found " << cams_R.size() << " images and " << wps_coords.size() << " worldpoints in [images.txt]" << std::endl;

  // read points3D.txt
  std::ifstream points3D_file;
  points3D_file.open(sfm_points3D.c_str());
  std::string points3D_line;

  while (std::getline(points3D_file, points3D_line))
  {
    // check first character for a comment (#)
    if (images_line.substr(0, 1).compare("#") != 0)
    {
      std::stringstream points3D_stream(points3D_line);

      // read id and coords
      double X, Y, Z;
      unsigned int pID;

      points3D_stream >> pID >> X >> Y >> Z;

      if (wps_coords.find(pID) != wps_coords.end())
      {
        wps_coords[pID] = Eigen::Vector3d(X, Y, Z);
      }
    }
  }
  points3D_file.close();

  // load images (parallel)
#ifdef L3DPP_OPENMP
#pragma omp parallel for
#endif //L3DPP_OPENMP
  for (int i = 0; i < img_seq.size(); ++i)
  {
    // get camera params
    unsigned int imgID = img_seq[i];
    unsigned int camID = img2cam[imgID];

    // intrinsics
    Eigen::Matrix3d K = cams_K[camID];
    Eigen::Vector3d radial = cams_radial[camID];
    Eigen::Vector2d tangential = cams_tangential[camID];

    if (cams_R.find(imgID) != cams_R.end())
    {
      // extrinsics
      Eigen::Matrix3d R = cams_R[imgID];
      Eigen::Vector3d t = cams_t[imgID];
      Eigen::Vector3d C = cams_C[imgID];

      // read image
      std::cout << inputFolder + "/" + cams_images[imgID] << std::endl;
      cv::Mat image = cv::imread(inputFolder + "/" + cams_images[imgID], CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat mask = cv::imread(inputFolder + "/seg_encode_results/" + cams_images[imgID], CV_LOAD_IMAGE_GRAYSCALE);

      // undistort image
      cv::Mat img_undist;
      if (fabs(radial(0)) > L3D_EPS || fabs(radial(1)) > L3D_EPS || fabs(radial(2)) > L3D_EPS ||
          fabs(tangential(0)) > L3D_EPS || fabs(tangential(1)) > L3D_EPS)
      {
        // undistorting
        Line3D->undistortImage(image, img_undist, radial, tangential, K);
      }
      else
      {
        // already undistorted
        img_undist = image;
      }

      // compute depths
      if (cams_worldpoints.find(imgID) != cams_worldpoints.end())
      {
        std::list<unsigned int> wps_list = cams_worldpoints[imgID];
        std::vector<float> depths;

        std::list<unsigned int>::iterator it = wps_list.begin();
        for (; it != wps_list.end(); ++it)
        {
          depths.push_back((C - wps_coords[*it]).norm());
        }

        // median depth
        if (depths.size() > 0)
        {
          std::sort(depths.begin(), depths.end());
          float med_depth = depths[depths.size() / 2];

          // add image
          Line3D->addImage(imgID, img_undist, mask, K, R, t, med_depth, wps_list);
        }
      }
    }
  }
  std::cout << "Matching ..." << std::endl;

  // match images
  Line3D->matchImages(L3D_DEF_SCORING_POS_REGULARIZER, L3D_DEF_SCORING_ANG_REGULARIZER, L3D_DEF_MATCHING_NEIGHBORS, L3D_DEF_EPIPOLAR_OVERLAP,
                      L3D_DEF_KNN, -1.0f);

  // compute result
  Line3D->reconstruct3Dlines(L3D_DEF_MIN_VISIBILITY_T, L3D_DEF_PERFORM_RDD, L3D_DEF_COLLINEARITY_T, L3D_DEF_USE_CERES);

  // save end result
  Line3D->get3Dlines(L3DPP_result_);

  // // save as STL
  // Line3D->saveResultAsSTL(outputFolder);
  // // save as OBJ
  // Line3D->saveResultAsOBJ(outputFolder);
  // // save as TXT
  // Line3D->save3DLinesAsTXT(outputFolder);
  // // save as BIN
  // Line3D->save3DLinesAsBIN(outputFolder);

  line_map_pub_.publish(getLineMap(Line3D, cams_images));

  // cleanup
  delete Line3D;
}

visualization_msgs::Marker Map::getLineMap(L3DPP::Line3D *Line3D, std::map<unsigned int, std::string> &cams_images)
{

  std::string inputFolder = "/home/chrisliu/Datasets/kitti/2011_09_26_drive_0101_sync/2011_09_26/2011_09_26_drive_0101_sync/image_02/data";

  visualization_msgs::Marker line_map;
  line_map.header.stamp = ros::Time::now();
  line_map.header.frame_id = "line_map";
  line_map.ns = "lineMap";
  line_map.action = visualization_msgs::Marker::ADD;
  line_map.pose.orientation.w = 1;
  line_map.id = 0;

  line_map.type = visualization_msgs::Marker::LINE_LIST;
  line_map.scale.x = 0.001;
  line_map.color.r = 1.0;
  line_map.color.a = 1.0;

  if (L3DPP_result_.empty())
  {
    return line_map;
  }

  for (int i = 0; i < L3DPP_result_.size(); ++i)
  {
    L3DPP::FinalLine3D current = L3DPP_result_[i];
    if (current.collinear3Dsegments_.size() == 0)
      continue;

    L3DPP::Segment2D segment2D = *current.underlyingCluster_.residuals()->begin();
    int camID = segment2D.camID();
    Eigen::Vector4f coords2D = Line3D->getSegmentCoords2D(segment2D);

    cv::Mat mask = cv::imread(inputFolder + "/seg_encode_results/" + cams_images[camID], CV_LOAD_IMAGE_GRAYSCALE);

    std_msgs::ColorRGBA color;
    cv::Scalar label_color = labels[mask.at<uchar>(coords2D(1), coords2D(0))].color;
    // cout << label_color << endl;
    color.a = 1.0;
    color.b = label_color(0)/255.0;
    color.g = label_color(1)/255.0;
    color.r = label_color(2)/255.0;

    // write 3D segments
    // file << current.collinear3Dsegments_.size() << " ";
    std::list<L3DPP::Segment3D>::const_iterator it2 = current.collinear3Dsegments_.begin();
    for (; it2 != current.collinear3Dsegments_.end(); ++it2)
    {
      // Eigen::Vector3d P1 = (*it2).P1();
      // Eigen::Vector3d P2 = (*it2).P2();

      geometry_msgs::Point P1, P2;
      P1.x = (*it2).P1().x();
      P1.y = (*it2).P1().y();
      P1.z = (*it2).P1().z();

      P2.x = (*it2).P2().x();
      P2.y = (*it2).P2().y();
      P2.z = (*it2).P2().z();

      line_map.points.push_back(P1);
      line_map.points.push_back(P2);

      line_map.colors.push_back(color);
      line_map.colors.push_back(color);
    }
  }
  return line_map;
}

void Map::pubLineMapfromTXT(const std::string &line3D_file_path)
{

  visualization_msgs::Marker line_map;
  line_map.header.stamp = ros::Time::now();
  line_map.header.frame_id = "line_map";
  line_map.ns = "lineMap";
  line_map.action = visualization_msgs::Marker::ADD;
  line_map.pose.orientation.w = 1;
  line_map.id = 0;

  line_map.type = visualization_msgs::Marker::LINE_LIST;
  line_map.scale.x = 0.001;
  line_map.color.r = 1.0;
  line_map.color.a = 1.0;

  // read points3D.txt
  std::ifstream line3D_file;
  line3D_file.open(line3D_file_path.c_str());

  std::string line3D_line;

  while (std::getline(line3D_file, line3D_line))
  {

    std::stringstream line3D_stream(line3D_line);

    // read id and coords
    int collinear3Dsegments_size;

    geometry_msgs::Point P1, P2;

    line3D_stream >> collinear3Dsegments_size >> P1.x >> P1.y >> P1.z >> P2.x >> P2.y >> P2.z;

    line_map.points.push_back(P1);
    line_map.points.push_back(P2);
  }
  line3D_file.close();
  // for (int i = 0; i < 100; ++i)
  {
    std::cout << "Publish LineMap!" << std::endl;
    line_map_pub_.publish(line_map);
  }
}
