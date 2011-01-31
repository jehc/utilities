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
 * Author: Jorge Garcia Bueno <jgarcia@ing.uc3m.es>, (C) 2010
 */

# include "plane_estimator.h"
# include <ntk/ntk.h>
# include <ntk/geometry/pose_3d.h>

using namespace cv;

namespace ntk
{

  double PlaneSolver::EnergyFunction(double *trial,bool &bAtSolution)
  {
    double distTotal = 0.0;
    double distRel = 0.0;

    // Outliers removal - does not change results so much...
/*
    std::vector<double> v;
    std::vector<double> w;

    for (int i = 0; i < g.size(); i++)
    {
       // Norm 1
         distRel = (trial[0] * g[i].x + trial[1] * g[i].y + trial[2] * g[i].z + trial[3])/
                   (fabs(trial[0]) + fabs(trial[1]) +  fabs(trial[2]));

         v.push_back(distRel);

    }
    sort(v.begin(),v.end());

    double distOutliers = v.at(v.size() * 0.8);
*/
    for (int i = 0; i < g.size(); i++)
    {
       // Norm 2
       // distRel = (trial[0] * g[i].x + trial[1] * g[i].y + trial[2] * g[i].z + trial[3])/
       //            sqrt(trial[0] * trial[0] +  trial[1] * trial[1] +  trial[2] * trial[2]);

       // Norm 1
         distRel = (trial[0] * g[i].x + trial[1] * g[i].y + trial[2] * g[i].z + trial[3])/
                   (fabs(trial[0]) + fabs(trial[1]) +  fabs(trial[2]));
         //std::cout << distRel << std::endl;

         //if (distRel < distOutliers)
           distTotal+=((double)fabs(distRel));
    }
    //printf("dist total %f\n", distTotal);
    return(distTotal);
  }






  PlaneEstimator :: PlaneEstimator()
    : m_solver(dim, population),
      m_ref_plane(0.0,1.0,1.0)
  {
    ntk::normalize(m_ref_plane);
    double min[dim];
    double max[dim];
    int i;

    for (i=0;i<dim;i++)
    {
      max[i] = 1000.0;
      min[i] = -1000.0;
    }
    m_solver.Setup(min,max,DifferentialEvolutionSolver::stBest1Exp,0.8,0.75);

    //m_solver.Setup(min,max,DifferentialEvolutionSolver::stRandToBest1Bin,0.9,1.0);
  }

  void PlaneEstimator :: estimate(RGBDImage& image, bool remove_plane_pixels)
  {
    if (!image.normal().data)
    {
      ntk_dbg(0) << "WARNING: you must active the normal filter to get plane estimation!";
      return;
    }

    double min[dim];
    double max[dim];
    int i;

    for (i=0;i<dim;i++)
    {
      max[i] = 1000.0;
      min[i] = -1000.0;
    }
    m_solver.Setup(min,max,DifferentialEvolutionSolver::stBest1Exp,0.8,0.75);

    // Passing from 3D to the optimizator

    const cv::Mat3f& normal_image = image.normal();
    const cv::Mat1f& distance_image = image.depth();
    cv::Mat1b& mask_image = image.depthMaskRef();
    cv::Mat1b plane_points (distance_image.size());
    const cv::Mat3b rgb = image.mappedRgb();
    const cv::Mat1f& amplitude = image.amplitude();

    plane_points = 0;

   // Early estimation of plane points projecting the normal values

    for (int r = 1; r < plane_points.rows-1; ++r)
    for (int c = 1; c < plane_points.cols-1; ++c)
    {
      if (mask_image.data && mask_image(r,c))
      {
        cv::Vec3f normal = normal_image(r,c);
        double prod = normal.dot(m_ref_plane);
        if (prod > 0.95)
          plane_points(r,c) = 255;
        else
          plane_points(r,c) = 0;
      }
    }

    imwrite("debug_planexx.png", plane_points);

    dilate(plane_points,plane_points,cv::Mat());
    erode(plane_points,plane_points,cv::Mat());

    std:vector<Point3f>& g = m_solver.planePointsRef();

    g.clear();
    
    for (int r = 1; r < plane_points.rows-1; ++r)
    for (int c = 1; c < plane_points.cols-1; ++c)
    {
      if (plane_points(r,c))
      {
        // posible miembro de la mesa!
        Point3f p3d = image.calibration()->depth_pose->unprojectFromImage(Point2f(c,r), distance_image(r,c));
        g.push_back(p3d);
      }
    }
    

    
    std::cout << "Tamano vector: " << g.size() << std::endl;

    printf("Calculating...\n\n");
    m_solver.Solve(max_generations);

    double *solution = m_solver.Solution();
    solution[3] = solution[3]*1.08;

    // Plane normalizer
    float suma = solution[0] + solution[1] + solution[2] + solution[3] ;
    for (int i = 0; i < 4; i++)
      solution[i] = solution[i]/ suma;

    ntk::Plane plano (solution[0], solution[1], solution[2], solution[3]);


    m_plane.set(solution[0], solution[1], solution[2], solution[3]);
    double planedenom  = sqrt
                        (solution[0]*solution[0] +
                         solution[1]*solution[1] +
                         solution[2]*solution[2]);
    // FIXME: temp
    // ntk::Plane p (toVxl(Vec3f(0,1,0)), toVxl(Point3f(0,-0.1,0)));


    // Vectors with the values rgb of the neighbours of the estimated plane
    std::vector<uchar> rmedian;
    std::vector<uchar> gmedian;
    std::vector<uchar> bmedian;

    std::vector<double> nmedianx;
    std::vector<double> nmediany;
    std::vector<double> nmedianz;

    // median values for each vector

    cv::Vec3b colormedian;
    cv::Vec3f normalmedian;


    if (remove_plane_pixels)
    {

      for (int r = 0; r < distance_image.rows; ++r)
      for (int c = 0; c < distance_image.cols; ++c)
      {
        if (mask_image.data && mask_image(r,c))
        {

          Point3f p3d = image.calibration()->depth_pose->unprojectFromImage(Point2f(c,r), distance_image(r,c));

          double dist = (

                   solution[0] * p3d.x +
                   solution[1] * p3d.y +
                   solution[2] * p3d.z +
                   solution[3]) /
                   planedenom;

          //std::cout << fabs(dist) << std::endl;
          if (fabs(dist) < 0.025 ) // We are close to the table...
          {

            Point3f p3rgb = image.calibration()->rgb_pose->projectToImage(p3d);
            double am = amplitude(r,c);

            cv::Mat3b& rgb = image.rgbRef();
            cv::Vec3b rgbvalue;
            cv::Vec3f normal = normal_image(r,c);

            rgbvalue = rgb((int)p3rgb.x,(int)p3rgb.y);

            rmedian.push_back(rgbvalue[0]);
            gmedian.push_back(rgbvalue[1]);
            bmedian.push_back(rgbvalue[2]);

            nmedianx.push_back(am);
            //nmediany.push_back(normal[1]);
            //nmedianz.push_back(normal[2]);

            //mask_image(r,c) = 0;

          }


      }
    }// end for image

     sort(rmedian.begin(), rmedian.end());
     sort(gmedian.begin(), gmedian.end());
     sort(bmedian.begin(), bmedian.end());
     sort(nmedianx.begin(), nmedianx.end());
     //sort(nmediany.begin(), nmediany.end());
     //sort(nmedianz.begin(), nmedianz.end());


     //printf("RGB %d %d %d\n", rmedian.at(rmedian.size()/2), gmedian.at(gmedian.size()/2),  bmedian.at(bmedian.size()/2));

     colormedian = cv::Vec3b(rmedian.at(rmedian.size()/2));//,
                             //gmedian.at(gmedian.size()/2),
                             //bmedian.at(bmedian.size()/2));


     normalmedian = cv::Vec3f(nmedianx.at(nmedianx.size()/2),
                              nmediany.at(nmediany.size()/2),
                              nmedianz.at(nmedianz.size()/2));

     normalize(normalmedian);


     // Once median values are obtained, let's sweep out the table!

     for (int r = 0; r < distance_image.rows; ++r)
     for (int c = 0; c < distance_image.cols; ++c)
     {
       if (mask_image.data && mask_image(r,c))
       {

         Point3f p3d = image.calibration()->depth_pose->unprojectFromImage(Point2f(c,r), distance_image(r,c));

         double dist = ( solution[0] * p3d.x + solution[1] * p3d.y + solution[2] * p3d.z + solution[3]) / planedenom;

         if (fabs(dist) < 0.025 ) // We are close to the table...
         {

           // Color for those pixels
           Point3f p3rgb = image.calibration()->rgb_pose->projectToImage(p3d);
           double am = amplitude(r,c);

           cv::Mat3b& rgb = image.rgbRef();
           cv::Vec3b rgbvalue;
           cv::Vec3f normal = normal_image(r,c);

           rgbvalue = rgb((int)p3rgb.x,(int)p3rgb.y);

           // filter in color, normal and distance!

           double distanceColor =  (
               (rgbvalue[0] - colormedian[0])*(rgbvalue[0] - colormedian[0])+
               (rgbvalue[1] - colormedian[1])*(rgbvalue[1] - colormedian[1])+
               (rgbvalue[2] - colormedian[2])*(rgbvalue[2] - colormedian[2]));

           double distanceNormal =  normalmedian.dot(normal);


           //std::cout << distanceNormal << std::endl;
           //if (distanceNormal > 0.8)
           //if (distanceColor < 20000.0)
             mask_image(r,c) = 0;

         }

       }


     }



    }// end if filtering




    //std::cout << "A=" <<  solution[0] << "\nB=" << solution[1] << "\nC=" << solution[2] << "\nD=" << solution[3] << "\n";
  }

} // ntk
