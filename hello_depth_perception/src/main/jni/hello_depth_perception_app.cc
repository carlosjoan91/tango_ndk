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

#include <tf/transform_datatypes.h>


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
}

void onPoseAvailable(void* context, const TangoPoseData *pose) {
  hello_depth_perception::tango_context* ctxt = (hello_depth_perception::tango_context*)context;
  if (pthread_mutex_trylock(ctxt->pose_mutex_ptr) == 0)
  {
    ctxt->odom_to_base_ptr->transform.translation.x = pose->translation[0];
    ctxt->odom_to_base_ptr->transform.translation.y = pose->translation[1];
    ctxt->odom_to_base_ptr->transform.translation.z = pose->translation[2];
    ctxt->odom_to_base_ptr->transform.rotation.x = pose->orientation[0];
    ctxt->odom_to_base_ptr->transform.rotation.y = pose->orientation[1];
    ctxt->odom_to_base_ptr->transform.rotation.z = pose->orientation[2];
    ctxt->odom_to_base_ptr->transform.rotation.w = pose->orientation[3];
    pthread_mutex_unlock(ctxt->pose_mutex_ptr);
  }
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
            //LOGI("Header stamp: %f", app->pc_msg.header.stamp.toSec());
            app->pc_msg.data.resize(3* sizeof(float) * pc_ptr->xyz_count);
            memcpy(&app->pc_msg.data[0], (void*)pc_ptr->xyz,  pc_ptr->xyz_count * 3 * sizeof(float));
            app->pc_pub.publish(app->pc_msg);
        }
        pthread_mutex_lock(&(app->pose_mutex));
        app->map_to_odom.header.seq = app->map_to_odom_seq++;
        app->odom_to_base.header.seq = app->odom_to_base_seq++;
        app->map_to_odom.header.stamp = app->odom_to_base.header.stamp = ros::Time::now();
        app->tf_bcaster->sendTransform(app->map_to_odom);
        app->tf_bcaster->sendTransform(app->odom_to_base);
        pthread_mutex_unlock(&(app->pose_mutex));
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
  pthread_mutex_init(&pose_mutex, NULL);
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

  int argc = 3;
  //char *argv[] = {"nothing_important" , "__master:=http://10.185.0.249:11311", "__ip:=10.185.0.224"};
  char *argv[] = {"nothing_important" , "__master:=http://192.168.1.110:11311", "__ip:=192.168.1.220"};
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
  ctxt.pose_mutex_ptr = &pose_mutex;
  ctxt.odom_to_base_ptr = &odom_to_base;
  ctxt.nh = new ros::NodeHandle();
  tf_bcaster = new tf2_ros::TransformBroadcaster;
  static_tf_bcaster = new tf2_ros::StaticTransformBroadcaster;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.translation.x = 0;
  map_to_odom.transform.translation.y = 0;
  map_to_odom.transform.translation.z = 0;
  map_to_odom.transform.rotation.x = 0;
  map_to_odom.transform.rotation.y = 0;
  map_to_odom.transform.rotation.z = 0;
  map_to_odom.transform.rotation.w = 1;
  odom_to_base.header.frame_id = "odom";
  odom_to_base.child_frame_id = "tango_base_link";
  odom_to_base.transform.translation.x = 0;
  odom_to_base.transform.translation.y = 0;
  odom_to_base.transform.translation.z = 0;
  odom_to_base.transform.rotation.x = 0;
  odom_to_base.transform.rotation.y = 0;
  odom_to_base.transform.rotation.z = 0;
  odom_to_base.transform.rotation.w = 1;
  base_to_depth.header.frame_id = "tango_base_link";
  base_to_depth.child_frame_id = "tango_camera_depth";
  base_to_color.header.frame_id = "tango_base_link";
  base_to_color.child_frame_id = "tango_camera_color";
  pc_pub = (ctxt.nh)->advertise<sensor_msgs::PointCloud2>("tango_image_depth", 1);
  known_pose_sub = (ctxt.nh)->subscribe("initial_pose", 1, &HelloDepthPerceptionApp::SetCurrentPoseCallback, this);
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

void HelloDepthPerceptionApp::SetCurrentPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& known_pose)
{

  //maptoodom = known * invert(odomtobase) 
  tf::Pose known, maptoodom;
  tf::Vector3 maptoodom_vect;
  tf::Quaternion maptoodom_quat;
  tf::poseMsgToTF(known_pose->pose.pose, known);
  pthread_mutex_lock(&pose_mutex);
  tf::Quaternion odom_to_base_quat(odom_to_base.transform.rotation.x, odom_to_base.transform.rotation.y, odom_to_base.transform.rotation.z, odom_to_base.transform.rotation.w);
  tf::Vector3 odom_to_base_vect(odom_to_base.transform.translation.x, odom_to_base.transform.translation.y, odom_to_base.transform.translation.z);
  tf::Pose odom_to_base_pose(odom_to_base_quat, odom_to_base_vect);
  maptoodom = known * (odom_to_base_pose.inverse());
  maptoodom_quat = maptoodom.getRotation();
  maptoodom_vect = maptoodom.getOrigin();
  map_to_odom.transform.translation.x = maptoodom_vect.getX();
  map_to_odom.transform.translation.y = maptoodom_vect.getY();
  map_to_odom.transform.translation.z = maptoodom_vect.getZ();
  map_to_odom.transform.rotation.x = maptoodom_quat.getX();
  map_to_odom.transform.rotation.y = maptoodom_quat.getY();
  map_to_odom.transform.rotation.z = maptoodom_quat.getZ();
  map_to_odom.transform.rotation.w = maptoodom_quat.getW();
  pthread_mutex_unlock(&pose_mutex);
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

  TangoCoordinateFramePair pair;
  pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pair.target = TANGO_COORDINATE_FRAME_DEVICE;


  err = TangoService_connectOnPoseAvailable(1, &pair, onPoseAvailable);
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

  TangoPoseData cToIMUPose;
  TangoCoordinateFramePair cToIMUPair;
  cToIMUPair.base = TANGO_COORDINATE_FRAME_IMU;
  cToIMUPair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  TangoService_getPoseAtTime(0.0, cToIMUPair, &cToIMUPose);
  base_to_color.transform.translation.x = cToIMUPose.translation[0];
  base_to_color.transform.translation.y = cToIMUPose.translation[1];
  base_to_color.transform.translation.z = cToIMUPose.translation[2];
  base_to_color.transform.rotation.x = cToIMUPose.orientation[0];
  base_to_color.transform.rotation.y = cToIMUPose.orientation[1];
  base_to_color.transform.rotation.z = cToIMUPose.orientation[2];
  base_to_color.transform.rotation.w = cToIMUPose.orientation[3];

  cToIMUPair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
  TangoService_getPoseAtTime(0.0, cToIMUPair, &cToIMUPose);
  base_to_depth.transform.translation.x = cToIMUPose.translation[0];
  base_to_depth.transform.translation.y = cToIMUPose.translation[1];
  base_to_depth.transform.translation.z = cToIMUPose.translation[2];
  base_to_depth.transform.rotation.x = cToIMUPose.orientation[0];
  base_to_depth.transform.rotation.y = cToIMUPose.orientation[1];
  base_to_depth.transform.rotation.z = cToIMUPose.orientation[2];
  base_to_depth.transform.rotation.w = cToIMUPose.orientation[3];
  base_to_depth.header.stamp = base_to_color.header.stamp = ros::Time::now();
  static_tf_bcaster->sendTransform(base_to_depth);
  static_tf_bcaster->sendTransform(base_to_color);
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
