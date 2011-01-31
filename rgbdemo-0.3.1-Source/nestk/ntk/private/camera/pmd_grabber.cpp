#include "pmd_grabber.h"

#include <ntk/ntk.h>

using namespace cv;

namespace ntk
{

PmdGrabber :: PmdGrabber() : m_integration_time(1000)
{
}

PmdGrabber :: ~PmdGrabber()
{
  pmdClose (m_hnd);
}

void PmdGrabber :: setIntegrationTime(unsigned usecs)
{
  m_integration_time = usecs;

  QWriteLocker locker(&m_lock);
  checkError(pmdSetIntegrationTime(m_hnd, 0, usecs));
}

void PmdGrabber :: initialize()
{
  checkError(pmdOpen (&m_hnd, "camcube", "", "camcubeproc", ""));
  ntk_dbg(0) << "Camera opened.";
  //setIntegrationTime(m_integration_time);
  //ntk_dbg(0) << "Integration time set.";
}

void PmdGrabber :: checkError (int code)
{
  if (code != PMD_OK)
  {
    char err[256];
    pmdGetLastError (m_hnd, err, 256);
    pmdClose (m_hnd);
    ntk_throw_exception("Error: " + err);
  }
}

void PmdGrabber :: run()
{
  cv::Mat1f distance(204,204);
  cv::Mat1f amplitude(204,204);
  cv::Mat1f intensity(204,204);

  while (!m_should_exit)
  {
    waitForNewEvent();

    checkError(pmdUpdate(m_hnd));
    checkError(pmdGetIntensities(m_hnd, intensity[0], sizeof(float)*204*204));
    checkError(pmdGetDistances(m_hnd, distance[0], sizeof(float)*204*204));
    checkError(pmdGetAmplitudes(m_hnd, amplitude[0], sizeof(float)*204*204));

    {
      QWriteLocker locker(&m_lock);

      flip(distance, m_rgbd_image.rawDepthRef(), 1);
      flip(amplitude, m_rgbd_image.rawAmplitudeRef(), 1);
      flip(intensity, m_rgbd_image.rawIntensityRef(), 1);
    }

    advertiseNewFrame();
  }
}

} // ntk
