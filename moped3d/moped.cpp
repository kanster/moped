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
#include <cv_bridge/CvBridge.h>

#include <moped.hpp>
#include <boost/algorithm/string.hpp>

#include <omp.h>

#include <dirent.h>

#define foreach( i, c ) for( typeof((c).begin()) i##_hid=(c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && i##_hid!=(c).end(); ++i##_hid) for( typeof( *(c).begin() ) &i=*i##_hid, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2) 
#define eforeach( i, it, c ) for( typeof((c).begin()) it=(c).begin(), i##_hid = (c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && it!=(c).end(); (it==i##_hid)?++it,++i##_hid:i##_hid=it) for( typeof(*(c).begin()) &i=*it, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2) 

using namespace std;

using namespace MopedNS;


string fix_param_name(string s)
{
  using namespace boost::algorithm;
  return replace_all_copy(replace_all_copy(replace_all_copy(s,":","_"),"-","_"),".","_");
}

class MopedROS {

	ros::NodeHandle n;
	image_transport::ImageTransport it;
	image_transport::Subscriber moped_sub;
	ros::Publisher moped_pub;
	
	Moped moped;
	Pt<4> intrinsicLinearCalibration;
	Pt<4> intrinsicNonlinearCalibration;
  
  // Use service to enable/disable MOPED
  int Enabled;
  ros::ServiceServer moped_enable;

public:

  // ----------------------- Constructor --------------------------- //
	MopedROS() : it(n) {
    Enabled = 1;
		ros::NodeHandle pnh("~");
		string modelsPath;
		pnh.param("models_path", modelsPath, string("/pr/data/moped/regular_models") );

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

    // Load models in parallel
		#pragma omp parallel for
		for(int i=0; i<(int)fileNames.size(); i++) {
			
			sXML XMLModel; 
			XMLModel.fromFile(fileNames[i]);
			
			#pragma omp critical(addModel)
			moped.addModel(XMLModel);
		}
		closedir(dp);

    string inputImageTopicName;
    string outputObjectListTopicName;
    string EnableSrvName;
    pnh.param("input_image_topic_name", inputImageTopicName, std::string("/Image"));
    pnh.param("output_object_list_topic_name", outputObjectListTopicName, std::string("/object_poses"));
    pnh.param("enable_service_name", EnableSrvName, std::string("/Enable"));


    moped_pub = n.advertise<pr_msgs::ObjectPoseList>(outputObjectListTopicName,100);
    moped_sub = it.subscribe(inputImageTopicName, 1, &MopedROS::process, this);
    moped_enable = n.advertiseService(EnableSrvName, &MopedROS::EnableMoped, this);

		double d1, d2, d3, d4;
		n.param("KK_fx", d1, 100.); 
		n.param("KK_fy", d2, 100.);
		n.param("KK_cx", d3, 320.);
		n.param("KK_cy", d4, 240.);
		intrinsicLinearCalibration.init(d1, d2, d3, d4);

		n.param("kc_k1", d1, 1e-12);
		n.param("kc_k2", d2, 1e-12);
		n.param("kc_p1", d3, 1e-12);
		n.param("kc_p2", d4, 1e-12);
		intrinsicNonlinearCalibration.init(d1, d2, d3, d4);

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
	void process( const sensor_msgs::ImageConstPtr& in ) {
		
    if (Enabled){
      sensor_msgs::CvBridge bridge;
      
      IplImage *gs = bridge.imgMsgToCv( in );
    
      vector<SP_Image> images;
      
      SP_Image mopedImage( new Image );
      
      mopedImage->name = "ROS_Image";
      
      mopedImage->intrinsicLinearCalibration = intrinsicLinearCalibration; 
      mopedImage->intrinsicNonlinearCalibration = intrinsicNonlinearCalibration;
      
      mopedImage->cameraPose.translation.init(0.,0.,0.);
      mopedImage->cameraPose.rotation.init(0.,0.,0.,1.);
      
      mopedImage->width = gs->width;
      mopedImage->height = gs->height;
      
      mopedImage->data.resize( gs->width * gs->height );
      
      for (int y = 0; y < gs->height; y++) 
        memcpy( &mopedImage->data[y*gs->width], &gs->imageData[y*gs->widthStep], gs->width );


      images.push_back( mopedImage );

      list<SP_Object> objects;
      int retval = moped.processImages( images, objects );

      if (retval < 0)
        ROS_FATAL("Could not detect any features. SIFT-GPU might not have access to the GPU: please verify your X Server configuration.");

      
      pr_msgs::ObjectPoseList outList;
      outList.header.seq = in->header.seq;
      outList.header.frame_id = "objdet_cam";
      outList.header.stamp = in->header.stamp;
      outList.originalTimeStamp = in->header.stamp;

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
		omp_set_num_threads(4);

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
