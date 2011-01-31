// <nicolas.burrus@uc3m.es>
// <jgbueno@ing.uc3m.es>


#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <sstream>
#include <iomanip>

#include <ntk/image/sift_gpu.h>
#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#ifdef USE_FREENECT
# include <ntk/camera/kinect_grabber.h>
#endif
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/surfels_rgbd_modeler.h>
#include "GuiController.h"
#include "ObjectDetector.h"
#include "ModelAcquisitionController.h"

#include <QApplication>
#include <QMetaType>

using namespace ntk;
using namespace cv;

namespace opt
{
  ntk::arg<const char*> dir_prefix("--prefix", "Directory prefix for output", "grab1");
  ntk::arg<int> first_index("--istart", "First image index", 0);
  ntk::arg<const char*> calibration_file("--calibration", "Calibration file (yml)", 0);
  ntk::arg<const char*> image("--image", "Fake mode, use given still image", 0);
  ntk::arg<const char*> directory("--directory", "Fake mode, use all view???? images in dir.", 0);
  ntk::arg<int> camera_id("--camera-id", "Camera id for opencv", 0);
  ntk::arg<bool> sync("--sync", "Synchronization mode", 0);
  ntk::arg<bool> pose_controller("--pose-controller", "Pose acquisition trajectory", 0);
  ntk::arg<const char*> pose_estimator("--pose-estimator",
                                       "Relative pose estimator (file|delta|image)",
                                       "image");
  ntk::arg<const char*> model_initial_pose("--pose-initial", "Model initial pose", 0);
  ntk::arg<const char*> model_delta_pose("--pose-delta", "Model delta pose", 0);
}

int main (int argc, char** argv)
{
  arg_base::set_help_option("-h");
  arg_parse(argc, argv);
  ntk_debug_level = 1;
  cv::setBreakOnError(true);

  GPUSiftServer server;
  if (server.isSupported())
    server.run();

  QApplication::setGraphicsSystem("raster");
  QApplication app (argc, argv);

  // Opening cameras. Its important to remember that CvCapture has to be done
  // before PMD otherwise will crash
  // 1.- Logitech (openCV)
  // 2.- PMD (pmdlibrary)

  const char* fake_dir = opt::image();
  bool is_directory = opt::directory() != 0;
  if (opt::directory())
    fake_dir = opt::directory();

  ntk::RGBDProcessor rgbd_processor;
  rgbd_processor.setMaxNormalAngle(40);
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeMapping, false);

  RGBDGrabber* grabber = 0;

  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeKinectDepthBaseline, true);
  rgbd_processor.setFilterFlag(RGBDProcessor::NoAmplitudeIntensityUndistort, true);

  if (opt::image() || opt::directory())
  {
    std::string path = opt::image() ? opt::image() : opt::directory();
    FileGrabber* file_grabber = new FileGrabber(path, opt::directory() != 0);
    grabber = file_grabber;
  }
  else
  {
    KinectGrabber* k_grabber = new KinectGrabber();
    k_grabber->initialize();
    k_grabber->setIRMode(false);
    grabber = k_grabber;
  }

  if (opt::sync())
    grabber->setSynchronous(true);

  RGBDFrameRecorder frame_recorder (opt::dir_prefix());
  frame_recorder.setFrameIndex(opt::first_index());
  frame_recorder.setSaveOnlyRaw(false);

  ObjectDetector detector;

  MeshGenerator* mesh_generator = 0;
  ntk::RGBDCalibration calib_data;
  if (opt::calibration_file())
  {
    calib_data.loadFromFile(opt::calibration_file());
    mesh_generator = new MeshGenerator();
    grabber->setCalibrationData(calib_data);
  }

  GuiController gui_controller (*grabber, rgbd_processor);
  grabber->addEventListener(&gui_controller);
  gui_controller.setFrameRecorder(frame_recorder);
  gui_controller.setObjectDetector(detector);

  if (opt::sync())
    gui_controller.setPaused(true);

#if 1
  SurfelsRGBDModeler modeler;
  modeler.setMinViewsPerSurfel(1);
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeNormals, 1);
  rgbd_processor.setMaxNormalAngle(90);
#elif 0
  ICPSurfelsRGBDModeler modeler;
  modeler.setMinViewsPerSurfel(1);
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeNormals, 1);
  rgbd_processor.setMaxNormalAngle(60);
#else
  RGBDModeler modeler;
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeNormals, 1);
#endif

  ModelAcquisitionController* acq_controller = 0;
  if (opt::calibration_file())
  {
    acq_controller = new ModelAcquisitionController (gui_controller, modeler);
    gui_controller.setModelAcquisitionController(*acq_controller);
  }

  RelativePoseEstimator* pose_estimator = 0;
  if (opt::calibration_file())
  {
    if (opt::pose_estimator() == std::string("file"))
    {
      pose_estimator = new RelativePoseEstimatorFromFile();
    }
    else if (opt::pose_estimator() == std::string("delta"))
    {
      ntk_ensure(opt::model_initial_pose() && opt::model_delta_pose(),
                 "Must specify initial and delta pose if delta pose estimator chosen.");
      Pose3D initial_pose;
      initial_pose.parseAvsFile(opt::model_initial_pose());
      Pose3D delta_pose;
      delta_pose.parseAvsFile(opt::model_delta_pose());
      pose_estimator = new RelativePoseEstimatorFromDelta(initial_pose, delta_pose);
    }
    else if (opt::pose_estimator() == std::string("image"))
    {
      pose_estimator = new RelativePoseEstimatorFromImage();
    }

    acq_controller->setPoseEstimator(pose_estimator);
  }

  if (mesh_generator)
    gui_controller.setMeshGenerator(*mesh_generator);

  grabber->start();

  app.exec();
  delete mesh_generator;
  delete acq_controller;

  if (server.isSupported())
    server.stop();
}
