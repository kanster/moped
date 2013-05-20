/* 
  MOPED (Multiple Object Pose Estimation and Detection) is a fast and
  scalable object recognition and pose estimation system. If you use this
  code, please reference our work in the following publications:
  
  [1] Collet, A., Berenson, D., Srinivasa, S. S., & Ferguson, D. "Object
  recognition and full pose registration from a single image for robotic
  manipulation." In ICRA 2009.  
  [2] Martinez, M., Collet, A., & Srinivasa, S. S. "MOPED: A Scalable and low
  Latency Object Recognition and Pose Estimation System." In ICRA 2010.
  
  Copyright: Carnegie Mellon University & Intel Corporation
  
  Authors:
   Alvaro Collet (alvaro.collet@gmail.com)
   Manuel Martinez (salutte@gmail.com)
   Siddhartha Srinivasa (siddhartha.srinivasa@intel.com)
  
  The MOPED software is developed at Intel Labs Pittsburgh. For more info,
  visit http://personalrobotics.intel-research.net/pittsburgh
  
  All rights reserved under the BSD license.
  
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. The name of the author may not be used to endorse or promote products
     derived from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <pr_msgs/ObjectPose.h>
#include <pr_msgs/ObjectPoseList.h>
#include <pr_msgs/Enable.h>
#include <cv_bridge/cv_bridge.h>


#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
#include <string>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <moped.hpp>
#include <boost/algorithm/string.hpp>

#include <omp.h>

#include <dirent.h>

#define foreach( i, c ) for( typeof((c).begin()) i##_hid=(c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && i##_hid!=(c).end(); ++i##_hid) for( typeof( *(c).begin() ) &i=*i##_hid, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2) 
#define eforeach( i, it, c ) for( typeof((c).begin()) it=(c).begin(), i##_hid = (c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && it!=(c).end(); (it==i##_hid)?++it,++i##_hid:i##_hid=it) for( typeof(*(c).begin()) &i=*it, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2) 

using namespace std;

using namespace MopedNS;
using namespace message_filters;

typedef union { char buffer[sizeof(Float)]; Float f; } Floatpun;

string fix_param_name(string s)
{
  using namespace boost::algorithm;
  return replace_all_copy(replace_all_copy(replace_all_copy(s,":","_"),"-","_"),".","_");
}


void processTest2( const sensor_msgs::PointCloud2::ConstPtr& pMsg){
    cerr << "Maybe?" << endl;
}

typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> KinectSyncPolicy;

class MopedROS {

	ros::NodeHandle n;
	image_transport::ImageTransport it;
	image_transport::Subscriber moped_sub;
	ros::Publisher moped_pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointSub;
    message_filters::Subscriber<sensor_msgs::Image> imageSub;
    Synchronizer<KinectSyncPolicy> kinectSync;
	
	Moped moped;
	Pt<4>   RGBIntrinsicLinearCalibration,
            DepthIntrinsicLinearCalibration;
	Pt<4>   RGBIntrinsicNonlinearCalibration,
            DepthIntrinsicNonlinearCalibration;

    // Use service to enable/disable MOPED
    int Enabled;
    ros::ServiceServer moped_enable;

public:

    // ----------------------- Constructor --------------------------- //
	MopedROS() : it(n), pointSub(n, "/DepthMap", 1), imageSub(n, "/Image", 1),
        kinectSync(KinectSyncPolicy(100), pointSub, imageSub)    
    {
        Enabled = 1;
        ros::NodeHandle pnh("~");
        string modelsPath;
        pnh.param("models_path", modelsPath, string("/pr/data/moped/regular_models"));

        DIR *dp;
        struct dirent *dirp;

        if((dp  = opendir(modelsPath.c_str())) ==  NULL) 
            throw string("Error opening \"") + modelsPath + "\"";

        vector<string> fileNames;
        while((dirp = readdir(dp)) != NULL) {

            string fileName =  modelsPath + "/" + string(dirp->d_name);
            if( fileName.rfind(".moped.xml") == string::npos ) continue;
            fileNames.push_back( fileName );
        }

        /*
        cerr << "Loading models!" << endl;
        */
        // Load models in parallel
        #pragma omp parallel for
        for(int i=0; i<(int)fileNames.size(); i++) {
            
            sXML XMLModel; 
            XMLModel.fromFile(fileNames[i]);
            
            #pragma omp critical(addModel)
            moped.addModel(XMLModel);
        }
        closedir(dp);
        /*
        cerr << "Done loading models!" << endl;
        */

        // string inputRGBImageTopicName;
        // string inputDepthmapTopicName;
        string outputObjectListTopicName;
        string EnableSrvName;
        // pnh.param("RGB_image_topic_name", inputRGBImageTopicName, std::string("/camera/rgb/image_color"));
        // pnh.param("Depthmap_topic_name", inputDepthmapTopicName, std::string("/camera/rgb/points"));
        pnh.param("output_object_list_topic_name", outputObjectListTopicName, std::string("/object_poses"));
        pnh.param("enable_service_name", EnableSrvName, std::string("/Enable"));

        /*
        cerr << "Subscribing" << endl;
        */

        moped_pub = n.advertise<pr_msgs::ObjectPoseList>(outputObjectListTopicName,100);
        /* subscribe to the the point cloud and image topic */

        /*
        cerr << "Trying to subscribe to " << inputDepthmapTopicName << " and " << inputRGBImageTopicName << endl;
        */

        kinectSync.registerCallback(boost::bind(&MopedROS::process, this, _1, _2));
        
/*        sync.registerCallback(boost::bind(&MopedROS::process, this, _1, _2)); */


    /*    moped_sub = it.subscribe(inputImageTopicName, 1, &MopedROS::process, this); */
        moped_enable = n.advertiseService(EnableSrvName, &MopedROS::EnableMoped, this);

        /*
        cerr << "Setting default intrinsics" << endl;
        */



        double d1, d2, d3, d4;
        n.param("KK_fx", d1, 1050.); 
        n.param("KK_fy", d2, 1050.);
        n.param("KK_cx", d3, 639.5);
        n.param("KK_cy", d4, 479.5);
        RGBIntrinsicLinearCalibration.init(d1, d2, d3, d4);

        // n.param("DEPTH_KK_fx", d1, 1050.00034); 
        // n.param("DEPTH_KK_fy", d2, 1050.00059);
        // n.param("DEPTH_KK_cx", d3, 639.015793);
        // n.param("DEPTH_KK_cy", d4, 479.015972);
        DepthIntrinsicLinearCalibration.init(d1, d2, d3, d4);

        n.param("RGB_kc_k1", d1, 1e-12);
        n.param("RGB_kc_k2", d2, 1e-12);
        n.param("RGB_kc_p1", d3, 1e-12);
        n.param("RGB_kc_p2", d4, 1e-12);
        RGBIntrinsicNonlinearCalibration.init(d1, d2, d3, d4);

        n.param("DEPTH_kc_k1", d1, 1e-12);
        n.param("DEPTH_kc_k2", d2, 1e-12);
        n.param("DEPTH_kc_p1", d3, 1e-12);
        n.param("DEPTH_kc_p2", d4, 1e-12);
        DepthIntrinsicNonlinearCalibration.init(d1, d2, d3, d4);

        map<string,string> config = moped.getConfig();		
        foreach( value, config ) 
          {
            n.param( fix_param_name(value.first), value.second, value.second);
          }
        moped.setConfig(config);

        ros::Rate loop_rate(60);
    }


    // ----------------------- Enable/Disable MOPED --------------------------- //
    bool EnableMoped(pr_msgs::Enable::Request& Req, pr_msgs::Enable::Response& Resp){
        Enabled = Req.Enable;
        Resp.ok = true;
        return true;
    }


    // ----------------------- Process --------------------------- //

	void process(   const sensor_msgs::PointCloud2::ConstPtr& pMsg, 
                    const sensor_msgs::Image::ConstPtr& iMsg){

    if (Enabled){

      vector<SP_Image> images;
       
      /* get the gray/rgb image */
      // IplImage *gs = bridge.imgMsgToCv( iMsg, "mono8" );
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(iMsg, "mono8"); 
      IplImage gs = (IplImage) cv_ptr->image;

      SP_Image gsMoped( new Image );
     
      gsMoped->imageType = IMAGE_TYPE_GRAY_IMAGE;
      gsMoped->name = "ROS_Image";
      /* copy over the intrinsics and extrinsics */
      gsMoped->intrinsicLinearCalibration = RGBIntrinsicLinearCalibration; 
      gsMoped->intrinsicNonlinearCalibration = RGBIntrinsicNonlinearCalibration;
      gsMoped->cameraPose.translation.init(0.,0.,0.);
      gsMoped->cameraPose.rotation.init(0.,0.,0.,1.);
      gsMoped->width = gs.width;
      gsMoped->height = gs.height;
      gsMoped->data.resize( gs.width * gs.height );
      for (int y = 0; y < gs.height; y++) 
        memcpy( &gsMoped->data[y*gs.width], &gs.imageData[y*gs.widthStep], gs.width );

        /* Done with the grayscale; now handle depth (slightly more complex) */
      
        /* the depthmap as a finished product */
        SP_Image depthmap( new Image );
        depthmap->name = "Depthmap";
        depthmap->imageType = IMAGE_TYPE_DEPTH_MAP;
        depthmap->intrinsicLinearCalibration = DepthIntrinsicLinearCalibration;
        depthmap->intrinsicNonlinearCalibration = DepthIntrinsicNonlinearCalibration;
        depthmap->cameraPose.translation.init(0.0,0.0,0.0);
        depthmap->cameraPose.rotation.init(0.0, 0.0, 0.0, 1.0);
        depthmap->width = gs.width;
        depthmap->height = gs.height;
        depthmap->data.resize( gs.width * gs.height * sizeof(Float) * 4 );

        /* the raw depthmap off the kinect */
        cv::Mat rawDepthmap(pMsg->height, pMsg->width, CV_32FC1);
        /* read in the raw depthmap */
        for(int y = 0; y < (int) rawDepthmap.rows; y++){
            for(int x = 0; x < (int) rawDepthmap.cols; x++){
                int i = y*rawDepthmap.cols + x;
                float *dataLocation = (float *) (&(pMsg->data[0]) + i*pMsg->point_step);
                if(isnan(dataLocation[2])){
                    rawDepthmap.at<float>(y, x) = -1e30;
                } else {
                    rawDepthmap.at<float>(y, x) = dataLocation[2];
                }
            }
        }  
       
        cv::Mat upscaledDepthmap; 
        /* Are the raw depthmap and our target depthmap the same size? */
        if (depthmap->width == pMsg->width && depthmap->height == pMsg->height)
        /* the upscaled depthmap */
          upscaledDepthmap = rawDepthmap;
        else{
          /* resize the raw depthmap */
          cv::resize(rawDepthmap, upscaledDepthmap, cv::Size(pMsg->width*2, pMsg->height*2));
        }

        Pt<4> K = DepthIntrinsicLinearCalibration;
        /* pass the results, processed somewhat to the depthmap */
        for(int v = 0; v < (int) gs.height; v++){
            for(int u = 0; u < (int) gs.width; u++){
                int i = v*gs.width+u;
                int offset = i*sizeof(Float)*4;
                Floatpun *buffer = (Floatpun *) ((&depthmap->data[0])+offset);
                if( (v >= (int) upscaledDepthmap.rows) || (u >= (int) upscaledDepthmap.cols)){
                    (buffer+0)->f = (buffer+1)->f = (buffer+2)->f = (buffer+3)->f = -1; 
                    continue;
                }
                float data = upscaledDepthmap.at<float>(v, u);
                if(data < 0){
                    (buffer+0)->f = (buffer+1)->f = (buffer+2)->f = (buffer+3)->f = -1; 
                } else {
                    /* Project into the world */
                    /* THESE HAVE TO BE MULTIPLIED BY THE DEPTH! OTHERWISE THIS TUGS THE VECTOR
                     * OUT OR IN, CHANGING WHAT PIXEL IT GOES THROUGH. */

                    Float x = data*(u - K[2]) / K[0], y = data*(v - K[3]) / K[1];
                    (buffer+0)->f = x;
                    (buffer+1)->f = y;
                    Float z = (buffer+2)->f = data;
                    /* update the distance */
                    (buffer+3)->f = sqrt(x*x+y*y+z*z); 
                }
            }
        }
      images.push_back( gsMoped );
      images.push_back( depthmap );

      list<SP_Object> objects;
      int retval = moped.processImages( images, objects );

      if (retval < 0)
        ROS_FATAL("Could not detect any features. SIFT-GPU might not have access to the GPU: please verify your X Server configuration.");
 
      pr_msgs::ObjectPoseList outList;
      outList.header.seq = iMsg->header.seq;
      outList.header.frame_id = "objdet_cam";
      outList.header.stamp = iMsg->header.stamp;
      outList.originalTimeStamp = iMsg->header.stamp;

      foreach( object, objects) {
        
        pr_msgs::ObjectPose out; // Our output msg
        
        out.name=object->model->name;
        
        out.pose.position.x = object->pose.translation[0];
        out.pose.position.y = object->pose.translation[1];
        out.pose.position.z = object->pose.translation[2];
        
        object->pose.rotation.norm();
        
        // Not really necessary, but this way we always display the same half of the quaternion hypersphere
        float flip = object->pose.rotation[0] + object->pose.rotation[1] + object->pose.rotation[2] + object->pose.rotation[3];
        if( flip < 0 ) {
          object->pose.rotation[0] *= -1;
          object->pose.rotation[1] *= -1;
          object->pose.rotation[2] *= -1;
          object->pose.rotation[3] *= -1;
        }

        out.pose.orientation.x = object->pose.rotation[0];
        out.pose.orientation.y = object->pose.rotation[1];
        out.pose.orientation.z = object->pose.rotation[2];
        out.pose.orientation.w = object->pose.rotation[3];
        
        out.mean_quality = object->score;
        out.used_points = 10;
        
        outList.object_list.push_back(out);
      }
      moped_pub.publish(outList);

      // Display some info
      clog << " Found " << objects.size() << " objects" << endl;
      foreach( object, objects )
        clog << " Found " << object->model->name << " at " << object->pose << " with score " << object->score << endl;

    }else{
      clog << "MOPED is disabled; skipping image" << endl;
    }
 
	}
};


int main(int argc, char** argv) {
	
	try {
		omp_set_num_threads(8);

		ros::init(argc, argv, "moped");
	
		MopedROS mopedROS;
	
		ros::spin();
	
		ROS_INFO("Done!");
		
	} catch( string s ) {
		cerr << "ERROR " << s << endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
