 #include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <iostream>

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped()){
         viewer.showCloud (cloud);
       }else{
         pcl::io::savePCDFile ("test_pcd.pcd", *cloud);
       }
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       const boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
         [this] (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) { cloud_cb_ (cloud); };

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }