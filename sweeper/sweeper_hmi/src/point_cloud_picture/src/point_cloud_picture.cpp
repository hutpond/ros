#include <functional>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>

#include "point_cloud_picture.h"
#include "point_cloud_picture_private.h"

PointCloudPicture::PointCloudPicture()
{
  picture_private_ptr_ = new PointCloudPicturePrivate();
}

PointCloudPicture::~PointCloudPicture()
{
  delete picture_private_ptr_;
}

void PointCloudPicture::startSubscirbe()
{
  picture_private_ptr_->subscriber_point_ = picture_private_ptr_->node_handle_.subscribe(
        "/ads_lidar_data", 1,
        &PointCloudPicturePrivate::onSubscirbePointCloud, picture_private_ptr_);

  picture_private_ptr_->subscriber_targets_ = picture_private_ptr_->node_handle_.subscribe(
        "/fusion_box_results", 1,
        &PointCloudPicturePrivate::onSubscirbeTargets, picture_private_ptr_);

  picture_private_ptr_->run();
}

void PointCloudPicture::stopSubscirbe()
{
  picture_private_ptr_->flag_subscribe_ = false;
  picture_private_ptr_->thread_spin_->join();
}

void PointCloudPicture::setSendFunction(std::function<void(const std::string &)> fun)
{
  picture_private_ptr_->send_function_ = fun;
}

void PointCloudPicture::setIntervalMSecond(int ms)
{
  picture_private_ptr_->interval_ms_ = ms;
}


PointCloudPicturePrivate::PointCloudPicturePrivate()
{
}

void PointCloudPicturePrivate::run()
{
  flag_subscribe_ = true;
  std::thread *thread = new std::thread([&]() {
    while (flag_subscribe_) {
      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    }
  });
  thread_spin_.reset(thread);
}

void PointCloudPicturePrivate::onSubscirbePointCloud(const sensor_msgs::PointCloud2 &point_cloud)
{
  point_cloud_ = point_cloud;

  this->createImage();
}

void PointCloudPicturePrivate::onSubscirbeTargets(const ads_msgs::ads_TrackTargetColl &targets)
{
  track_targets_ = targets;
}

void PointCloudPicturePrivate::createImage()
{
  if (point_cloud_.width == 0) {
    return;
  }
  // init Mat
  double pixel_w = 800;
  double pixel_h = 800;
  cv::Mat image(pixel_w, pixel_h, CV_8UC3);
  for (int i = 0; i < image.rows; i++) {
    uchar *p = image.ptr<uchar>(i);
    for (int j = 0; j < image.cols; j++) {
      p[3 * j] = 0;
      p[3 * j + 1] = 0;
      p[3 * j + 2] = 0;
    }
  }

  // decode point cloud
  std::vector<geometry_msgs::PointPtr> points;
  const int size_points = point_cloud_.width;
  uint8_t *data = point_cloud_.data.data();
  double min_x = 10000, max_x = -10000;
  double min_y = 10000, max_y = -10000;
  double min_z = 10000, max_z = -10000;

  for (int i = 0; i < size_points; ++i) {
    float x, y, z;

    memcpy(&x, data, sizeof(float));
    data += sizeof(float);

    memcpy(&y, data, sizeof(float));
    data += sizeof(float);

    memcpy(&z, data, sizeof(float));
    data += sizeof(float);

    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
      continue;
    }
    if (std::abs(x) > 30 || std::abs(y) > 30 || std::abs(z) > 30) {
      continue;
    }

    boost::shared_ptr<geometry_msgs::Point> point(new geometry_msgs::Point);
    point->x = x;
    point->y = y;
    point->z = z;
    points.push_back(point);

    if (min_x > point->x) {
      min_x = point->x;
    }
    if (max_x < point->x) {
      max_x = point->x;
    }
    if (min_y > point->y) {
      min_y = point->y;
    }
    if (max_y < point->y) {
      max_y = point->y;
    }
    if (min_z > point->z) {
      min_z = point->z;
    }
    if (max_z < point->z) {
      max_z = point->z;
    }

    data += sizeof(float) * (point_cloud_.fields.size() - 3);
  }

  // draw point cloud
  double start_x, start_y, mm_per_pixel;
  if ( (max_x - min_x) / (max_y - min_y) > pixel_h / pixel_w ) {
    start_x = min_x;
    mm_per_pixel = (max_x - min_x) / pixel_h;
    start_y = min_y - (mm_per_pixel * pixel_w - (max_y - min_y)) / 2.0;
  }
  else {
    start_y = min_y;
    mm_per_pixel = (max_y - min_y) / pixel_w;
    start_x = min_x - (mm_per_pixel * pixel_h - (max_x - min_x)) / 2.0;
  }

  for (const auto &point : points) {
    int i = pixel_h - (point->x - start_x) / mm_per_pixel;
    int j = pixel_w - (point->y - start_y) / mm_per_pixel;
    if (i < 0 || i >= pixel_h || j < 0 || j >= pixel_w) {
      continue;
    }

    uchar *p = image.ptr<uchar>(i);
    p[3 * j] = 255;
    p[3 * j + 1] = 255;
    p[3 * j + 2] = 255;
  }

  // draw targets
  for (const auto &target : track_targets_.track_objects) {
    const int size_edge_pts = target.edge_points.size();
    for (int i = 0; i < size_edge_pts; ++i) {
      double pt1_x, pt1_y, pt2_x, pt2_y;  // 起点、终点坐标
      pt1_x = target.edge_points[i].x;
      pt1_y = target.edge_points[i].y;
      if (i == size_edge_pts - 1) {
        pt2_x = target.edge_points[0].x;
        pt2_y = target.edge_points[0].y;
      }
      else {
        pt2_x = target.edge_points[i + 1].x;
        pt2_y = target.edge_points[i + 1].y;
      }

      if (std::abs(pt1_x) > 30 || std::abs(pt1_y) > 30 ||
          std::abs(pt2_x) > 30 || std::abs(pt2_y) > 30) {
        continue;
      }

      if (std::abs<double>(pt2_y - pt1_y) > std::abs<double>(pt2_x - pt1_x)) {
        double k = (pt2_x - pt1_x) / (pt2_y - pt1_y);
        double b = pt1_x - k * pt1_y;
        auto fun = [&k, &b](double y) {return k * y + b;};

        double pt_start_y, pt_end_y;  // y轴起始、终点值
        if (pt2_y > pt1_y) {
          pt_start_y = pt1_y;
          pt_end_y = pt2_y;
        }
        else {
          pt_start_y = pt2_y;
          pt_end_y = pt1_y;
        }
        double pt_y = pt_start_y;
        while (pt_y < pt_end_y) {
          double pt_x = fun(pt_y);
          int i = (pt_x - start_x) / mm_per_pixel;
          int j = (pt_y - start_y) / mm_per_pixel;
          if (i < 0 || i >= pixel_h || j < 0 || j >= pixel_w) {
            continue;
          }

          uchar *p = image.ptr<uchar>(i);
          p[3 * j] = 0;
          p[3 * j + 1] = 255;
          p[3 * j + 2] = 0;

          pt_y += mm_per_pixel;
        }
      }
      else {
        double k = (pt2_y - pt1_y) / (pt2_x - pt1_x);
        double b = pt1_y - k * pt1_x;
        auto fun = [&k, &b](double x) {return k * x + b;};

        double pt_start_x, pt_end_x;  // x轴起始、终点值
        if (pt2_x > pt1_x) {
          pt_start_x = pt1_x;
          pt_end_x = pt2_x;
        }
        else {
          pt_start_x = pt2_x;
          pt_end_x = pt1_x;
        }
        double pt_x = pt_start_x;
        while (pt_x < pt_end_x) {
          double pt_y = fun(pt_x);
          int i = pixel_h - (pt_x - start_x) / mm_per_pixel;
          int j = pixel_w - (pt_y - start_y) / mm_per_pixel;

          uchar *p = image.ptr<uchar>(i);
          p[3 * j] = 0;
          p[3 * j + 1] = 255;
          p[3 * j + 2] = 0;

          pt_x += mm_per_pixel;
        }
      }
    }
  }

  std::vector<unsigned char> img_jpg;
  cv::imencode(".jpg", image, img_jpg);

  std::string name = "/tmp/1.jpg";
  std::ofstream out(name, std::ios::binary);
  out.write(reinterpret_cast<const char*>(img_jpg.data()), img_jpg.size());
  out.close();

  send_function_(name);
}

