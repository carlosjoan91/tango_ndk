/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdlib>

#include <tango_support_api.h>

#include "hello_depth_perception/hello_depth_perception_app.h"


namespace {
// The minimum Tango Core version required from this application.
constexpr int kTangoCoreMinimumVersion = 9377;

// This function logs XYZij data from onXYZijAvailable callbacks.
//
// @param context, this will be a pointer to a HelloDepthPerceptionApp
//        instance on which to call callbacks. This parameter is hidden
//        since it is not used.
// @param *point_cloud, XYZij data to log.
void onPointCloudAvailable(void* context, const TangoXYZij* point_cloud) {
  // Number of points in the point cloud.
  int point_cloud_size;
  float average_depth;
  int ret;
  hello_depth_perception::tango_context* ctxt = (hello_depth_perception::tango_context*)context;
  ret = TangoSupport_updatePointCloud(ctxt->pc_manager, point_cloud);
  if (ret != TANGO_SUCCESS)
  {
    LOGE("ERROR UPDATING TANGO MANAGER");
  }
  // Calculate the average depth.
  /*point_cloud_size = point_cloud->xyz_count;
  average_depth = 0;
  // Each xyz point has 3 coordinates.
  size_t iteration_size = point_cloud_size * 3;
  size_t vertices_count = 0;
  for (size_t i = 2; i < iteration_size; i += 3) {
    average_depth += point_cloud->xyz[0][i];
    vertices_count++;
  }
  if (vertices_count) {
    average_depth /= vertices_count;
  }

  // Log the number of points and average depth.
  LOGI("HelloDepthPerceptionApp: Point count: %d. Average depth (m): %.3f",
       point_cloud_size, average_depth);*/
}

void* pub_thread_method(void* arg)
{
    hello_depth_perception::HelloDepthPerceptionApp* app;
    app = (hello_depth_perception::HelloDepthPerceptionApp*) arg;
    ros::Rate rate(10);
    TangoErrorType ret;
    TangoXYZij* pc_ptr;
    bool new_available;
    while (ros::ok())
    {
        ret = TangoSupport_getLatestPointCloudAndNewDataFlag((app->ctxt).pc_manager, &pc_ptr, &new_available);
        if (ret != TANGO_SUCCESS)
        {
            LOGE("Error retrieving latest pointcloud");
        }
        if (new_available)
        {
            app->pc_msg.width = app->pc_msg.row_step = pc_ptr->xyz_count;
            app->pc_msg.header.seq = app->seq++;
            app->pc_msg.header.stamp = ros::Time::now();
            LOGI("Header stamp: %f", app->pc_msg.header.stamp.toSec());
            app->pc_msg.data.resize(3* sizeof(float) * pc_ptr->xyz_count);
            memcpy(&app->pc_msg.data[0], (void*)pc_ptr->xyz,  pc_ptr->xyz_count * 3 * sizeof(float));
            app->pc_pub.publish(app->pc_msg);
        }
        rate.sleep();
    }
}
}  // anonymous namespace

namespace hello_depth_perception {

void HelloDepthPerceptionApp::OnCreate(JNIEnv* env, jobject caller_activity) {
  int seq = 0;
  sensor_msgs::PointField x, y, z;
  x.name = "x";
  y.name = "y";
  z.name = "z";
  x.offset = 0;
  y.offset = sizeof(float);
  z.offset = 2.0*sizeof(float);
  x.count = y.count = z.count = 1;
  x.datatype = y.datatype = z.datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.header.frame_id = "tango_camera_depth";
  pc_msg.height = 1;
  pc_msg.is_bigendian = false;
  pc_msg.is_dense = true;
  pc_msg.point_step = 3 * sizeof(float);
  pc_msg.fields.push_back(x);
  pc_msg.fields.push_back(y);
  pc_msg.fields.push_back(z);
  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  // Check the installed version of the TangoCore. If it is too old, then
  // it will not support the most up to date features.
  int32_t max_point_cloud_elements;
  int ret = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements",
                                       &max_point_cloud_elements);
  if(ret != TANGO_SUCCESS) {
    LOGE("Failed to query maximum number of point cloud elements.");
  }

  ret = TangoSupport_createPointCloudManager(max_point_cloud_elements, &(ctxt.pc_manager));
  if(ret != TANGO_SUCCESS) {
      LOGE("Failed to create support point cloud manager");
  }

  ret = TangoSupport_createXYZij(max_point_cloud_elements, &local_pc);
  if(ret != TANGO_SUCCESS) {
        LOGE("Failed to create support point cloud");
  }

  int argc = 3;
  char *argv[] = {"nothing_important" , "__master:=http://10.185.0.249:11311", "__ip:=10.185.0.224"};
  LOGI("GOING TO ROS INIT");

  for(int i = 0; i < argc; i++){
      LOGI("%s", argv[i]);
  }

  // %Tag(ROS_INIT)%
  ros::init(argc, &argv[0], "android_ndk_native_cpp");
  // %EndTag(ROS_INIT)%

  LOGI("GOING TO NODEHANDLE");

  // %Tag(ROS_MASTER)%
  std::string master_uri = ros::master::getURI();

  if(ros::master::check()){
      LOGI("ROS MASTER IS UP!");
  } else {
      LOGI("NO ROS MASTER.");
  }
  LOGI("%s", master_uri.c_str());

  ctxt.nh = new ros::NodeHandle();

  pc_pub = (ctxt.nh)->advertise<sensor_msgs::PointCloud2>("tango_image_depth", 1);

  int version = 0;
  TangoErrorType err =
      TangoSupport_GetTangoVersion(env, caller_activity, &version);
  if (err != TANGO_SUCCESS || version < kTangoCoreMinimumVersion) {
    LOGE(
        "HelloDepthPerceptionApp::OnCreate, Tango Core version is out"
        " of date.");
    std::exit(EXIT_SUCCESS);
  }
}

void HelloDepthPerceptionApp::OnTangoServiceConnected(JNIEnv* env,
                                                      jobject binder) {
  // Associate the service binder to the Tango service.
  if (TangoService_setBinder(env, binder) != TANGO_SUCCESS) {
    LOGE(
        "HelloDepthPerceptionApp::OnTangoServiceConnected,"
        "TangoService_setBinder error");
    std::exit(EXIT_SUCCESS);
  }

  // Here, we'll configure the service to run in the way we want. For this
  // application, we'll start from the default configuration.
  // TANGO_CONFIG_DEFAULT is disabling Depth Perception.
  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tango_config_ == nullptr) {
    LOGE(
        "HelloDepthPerceptionApp::OnTangoServiceConnected,"
        "TangoService_getConfig error.");
    std::exit(EXIT_SUCCESS);
  }

  // Enable Depth Perception.
  TangoErrorType err =
      TangoConfig_setBool(tango_config_, "config_enable_depth", true);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "HelloDepthPerceptionApp::OnTangoServiceConnected,"
        "config_enable_depth() failed with error code: %d.",
        err);
    std::exit(EXIT_SUCCESS);
  }

  // Attach the OnXYZijAvailable callback to the onPointCloudAvailable
  // function defined above. The callback will be called every time a new
  // point cloud is acquired, after the service is connected.
  err = TangoService_connectOnXYZijAvailable(onPointCloudAvailable);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "HelloDepthPerceptionApp::OnTangoServiceConnected,"
        "Failed to connect to point cloud callback with error code: %d",
        err);
    std::exit(EXIT_SUCCESS);
  }

  // Connect to the Tango Service, the service will start running:
  // point clouds can be queried and callbacks will be called.
  err = TangoService_connect((void*)&ctxt, tango_config_);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "HelloDepthPerceptionApp::OnTangoServiceConnected,"
        "Failed to connect to the Tango service with error code: %d",
        err);
    std::exit(EXIT_SUCCESS);
  }

  pthread_create(&pub_thread, NULL, pub_thread_method, (void*)this);
}

void HelloDepthPerceptionApp::OnPause() {
  // When disconnecting from the Tango Service, it is important to make sure to
  // free your configuration object. Note that disconnecting from the service,
  // resets all configuration, and disconnects all callbacks. If an application
  // resumes after disconnecting, it must re-register configuration and
  // callbacks with the service.
  TangoConfig_free(tango_config_);
  tango_config_ = nullptr;
  TangoService_disconnect();
  LOGI("Shutting down ros");
  ros::shutdown();
  pthread_join(pub_thread, NULL);
  LOGI("Done");
}
}  // namespace hello_depth_perception
