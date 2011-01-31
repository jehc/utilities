/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef   	NTK_CAMERA_PMD_GRABBER_H_
# define   	NTK_CAMERA_PMD_GRABBER_H_

#include <ntk/core.h>
#include <ntk/camera/rgbd_grabber.h>

#include "pmdsdk2.h"

namespace ntk
{

class PmdGrabber : public RGBDGrabber
{
public:
  PmdGrabber();
  void initialize();

  ~PmdGrabber();

public:
  unsigned integrationTime() { return m_integration_time; }
  void setIntegrationTime(unsigned usecs);
  void checkError (int code);

protected:
  virtual void run();

private:
  PMDHandle m_hnd;
  unsigned m_integration_time;
};

}

#endif // ! NTK_CAMERA_PMD_GRABBER_H_
