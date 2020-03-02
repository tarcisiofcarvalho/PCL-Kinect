#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <iostream>

// For compression
#include <pcl/point_cloud.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

// For point reduction as voxel grid
#include <pcl/filters/voxel_grid.h>

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     typedef pcl::PointCloud<pcl::PointXYZRGBA> CloudType;

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {

        if (!viewer.wasStopped()){
          viewer.showCloud (cloud);
        }else{
          // ****** Remove NAN data *******
          //typedef pcl::PointCloud<pcl::PointXYZRGBA> CloudType;
          CloudType::Ptr outputCloud (new CloudType);
          CloudType::Ptr cloudOut (new CloudType);
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(*cloud,*outputCloud, indices);

          // ******* Compress point cloud ********
          // stringstream to store compressed point cloud
          std::stringstream compressedData;

          // compress point cloud
          PointCloudEncoder->encodePointCloud (outputCloud, compressedData);

          // decompress point cloud
          PointCloudDecoder->decodePointCloud (compressedData, cloudOut);

          // Voxel Grid - points reduction
          pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
          sor.setInputCloud (cloudOut);
          sor.setLeafSize (0.01f, 0.01f, 0.01f);
          CloudType::Ptr cloud_filtered (new CloudType);
          sor.filter (*cloud_filtered);

          // ****** Save point cloud data file *******
          pcl::io::savePCDFile("test_pcd_rgba_compressed.pcd", *cloud_filtered);
       }
     }

     void run ()
     {

        bool showStatistics = false;

        // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

        // instantiate point cloud compression for encoding and decoding
        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> ();

       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       const boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         [this] (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) { cloud_cb_ (cloud); };

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();

        // delete point cloud compression instances
        delete (PointCloudEncoder);
        delete (PointCloudDecoder);
     }

     pcl::visualization::CloudViewer viewer;

     pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
     pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;
     
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }