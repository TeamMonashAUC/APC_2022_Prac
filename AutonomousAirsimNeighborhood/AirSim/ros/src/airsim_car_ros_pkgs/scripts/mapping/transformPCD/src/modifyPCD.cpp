#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define PI 3.14159265

float* multiply(float mat[][3],
              float vec[])
{
  float* res = new float[3]{0};

  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++){
        res[i]+= (mat[i][j]*vec[j]);

    }
  }
  return res;
}


int main ()
{
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
 
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/khai/pcd/maps/neighborhood.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from pcd file "
            << std::endl;
  
  float angle = -5.4 * PI / 180.0;
  float rotmat[3][3] = { { std::cos(angle), -std::sin(angle), 0 },
                       { std::sin(angle), std::cos(angle), 0 },
                       { 0, 0, 1 } };

  // Rearrange the x,y and z values of the pointcloud;
  for (auto& point: *cloud){

    float pt[3] = {-point.x, -point.y, point.z};
    float* new_pt = multiply(rotmat,pt);

    point.x = new_pt[0];
    point.y = new_pt[1];
    point.z = new_pt[2]; 

    delete[] new_pt;
  }
  
  pcl::io::savePCDFileASCII("neighbourhood_adjusted.pcd", *cloud);

  return (0);
}