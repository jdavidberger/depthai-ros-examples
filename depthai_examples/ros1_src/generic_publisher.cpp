
#include <cstdio>
#include <iostream>

#include "ros/ros.h"
// #include "utility.hpp"
#include <camera_info_manager/camera_info_manager.h>

#include <functional>

#include "sensor_msgs/Image.h"

// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/GenericPipelinePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

#include "depthai/depthai.hpp"

template <typename T>
static inline void getParamWithWarning(ros::NodeHandle& pnh, const char* key, T& val) {
    bool gotParam = pnh.getParam(key, val);
    std::stringstream ss;
    ss << val;
    if(!gotParam) {
        ROS_WARN("Could not find param '%s' on node '%s'. Defaulting to '%s'", key, pnh.getNamespace().c_str(), ss.str().c_str());
    } else {
        ROS_INFO("Node %32s param %20s: %s", pnh.getNamespace().c_str(), key, ss.str().c_str());
    }
}

dai::Pipeline createPipeline(ros::NodeHandle& pnh) {
    dai::Pipeline pipeline;

    std::string camera_param_uri;
    std::string resolution = "720p";
    std::string mode = "depth";

    bool lrcheck = true, extended = false,
            subpixel = true, rectify = true, depth_aligned = true;
    int stereo_fps = 30;
    int confidence = 200;
    int LRchecktresh = 5;
    int bilateralSigma = 5;
    int decimation = 1;
    int fixedFocus = 135;

    getParamWithWarning(pnh, "camera_param_uri", camera_param_uri);
    getParamWithWarning(pnh, "lrcheck", lrcheck);
    getParamWithWarning(pnh, "extended", extended);
    getParamWithWarning(pnh, "subpixel", subpixel);
    getParamWithWarning(pnh, "confidence", confidence);
    getParamWithWarning(pnh, "LRchecktresh", LRchecktresh);
    getParamWithWarning(pnh, "mode", mode);
    getParamWithWarning(pnh, "rectify",  rectify);
    getParamWithWarning(pnh, "depth_aligned",  depth_aligned);
    getParamWithWarning(pnh, "stereo_fps",  stereo_fps);
    getParamWithWarning(pnh, "bilateral_sigma",  bilateralSigma);
    getParamWithWarning(pnh, "mono_resolution",   resolution);
    getParamWithWarning(pnh, "decimation",   decimation);
    getParamWithWarning(pnh, "fixed_focus",   fixedFocus);

    bool enableDepth = mode == "depth";

    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    if(enableDepth) {
        xoutDepth->setStreamName("depth");
    } else {
        xoutDepth->setStreamName("disparity");
    }

    xoutImu->setStreamName("imu");

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(stereo_fps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(stereo_fps);

    // StereoDepth
    auto rawCfg = stereo->initialConfig.get();
    rawCfg.postProcessing.decimationFilter.decimationFactor = decimation;
    stereo->initialConfig.set(rawCfg);
    stereo->initialConfig.setConfidenceThreshold(confidence);        // Known to be best
    stereo->setRectifyEdgeFillColor(0);                              // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);  // Known to be best
    stereo->initialConfig.setBilateralFilterSigma(bilateralSigma);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    stereo->setRuntimeModeSwitch(true);
    if(enableDepth && depth_aligned) stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    // Imu
    imu->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR, dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 400);
    imu->setMaxBatchReports(1);  // Get one message only for now.

    if(enableDepth && depth_aligned) {
        // RGB image
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
        xoutRgb->setStreamName("rgb");
        camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        // the ColorCamera is downscaled from 1080p to 720p.
        // Otherwise, the aligned depth is automatically upscaled to 1080p
        camRgb->setIspScale(2, 3);

        if(fixedFocus < 0) {
            camRgb->initialControl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::CONTINUOUS_VIDEO);
        } else {
            camRgb->initialControl.setManualFocus(fixedFocus);
        }
        camRgb->isp.link(xoutRgb->input);
    } else {
        // Stereo imges
        auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight = pipeline.create<dai::node::XLinkOut>();
        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if(rectify) {
            stereo->rectifiedLeft.link(xoutLeft->input);
            stereo->rectifiedRight.link(xoutRight->input);
        } else {
            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
        }
    }

    // Link plugins CAM -> STEREO -> XLINK

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    if(enableDepth) {
        stereo->depth.link(xoutDepth->input);
    } else {
        stereo->disparity.link(xoutDepth->input);
    }

    imu->out.link(xoutImu->input);

    return pipeline;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dai_generic_publisher", ros::InitOption::AnonymousName);

    dai::Device device;
    ros::NodeHandle pnh("~");

    std::string tfPrefix = "dai_" + device.getMxId();
    std::string topicPrefix = tfPrefix;

    getParamWithWarning(pnh, "tf_prefix", tfPrefix);
    getParamWithWarning(pnh, "topic_prefix", topicPrefix);

    dai::Pipeline pipeline = createPipeline(pnh);

    ros::NodeHandle n(topicPrefix);
    auto publisher = dai::ros::GenericPipelinePublisher(n, device, pipeline, tfPrefix);

    while(ros::ok() && !device.isClosed()) {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    return 0;
}
