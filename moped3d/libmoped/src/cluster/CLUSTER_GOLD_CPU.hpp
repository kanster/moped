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
#pragma once

namespace MopedNS {

	class CLUSTER_GOLD_CPU : public MopedAlg {
    
		
	public:

		CLUSTER_GOLD_CPU(){
		}

		void getConfig( map<string,string> &config ) const {
		

		}
		
		void setConfig( map<string,string> &config ) {
			

		}

        bool pointInPolygon(Float x, Float y, vector<pair<Float, Float> > polygon){
            /* From  http://paulbourke.net/geometry/insidepoly/ */
            int i, j, c = 0, n = polygon.size();
            for (i = 0, j = n-1; i < n; j = i++) {
                if ((((polygon[i].second <= y) && (y < polygon[j].second)) ||
                    ((polygon[j].second <= y) && (y < polygon[i].second))) &&
                    (x < (polygon[j].first - polygon[i].first) * (y - polygon[i].second) / 
                     (polygon[j].second - polygon[i].second) + polygon[i].first))
                    c = !c;
                }
            return c;
        }

		void process( FrameData &frameData ) {

			frameData.clusters.resize( models->size() );

            /* find the depthmap and distance map */
            SP_Image depthmap, grayImage;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                if(frameData.images[i]->imageType == IMAGE_TYPE_DEPTH_MAP){
                    depthmap = frameData.images[i];
                    break;
                } else if(frameData.images[i]->imageType == IMAGE_TYPE_GRAY_IMAGE){
                    grayImage = frameData.images[i];
                    break;
                }
            }
    
            ifstream f;
            f.open((grayImage->name+".polyLabels.txt").c_str());

            char buffer[4096];

            vector<vector<pair<float,float> > > polygons;
            vector<string> modelNames;
            while(!f.eof()){
                vector<pair<float,float> > polygon;
                string modelName;
                f.getline(buffer, 4096);
                stringstream ss(stringstream::in | stringstream::out);
                ss << buffer;
                ss >> modelName;
                Float x, y;
                while(!ss.eof()){
                    ss >> x >> y;
                    polygon.push_back(make_pair(x,y));
                }
                modelNames.push_back(modelName);
                polygons.push_back(polygon);
            } 

			#pragma omp parallel for
			for( int model=0; model<(int)frameData.matches.size(); model++) {

                /* get the current matches */
                vector<FrameData::Match> matches = frameData.matches[model];

                int numPolys = polygons.size();
                vector<int> clusterNumber; clusterNumber.reserve(numPolys);
                string modelName = (*models)[model]->name;
                int currentClusterNumber = 0;
                for(int i = 0; i < numPolys; i++){
                    if(modelNames[i] == modelName){
                        clusterNumber[i] = currentClusterNumber;
                        currentClusterNumber += 1;
                    }  else {
                        clusterNumber[i] = -1;
                    }
                }

                /* 
                vector<FrameData::Cluster >clusters = hierarchicalCluster(K);
                */
                vector<FrameData::Cluster> clusters;
                for(int i = 0; i < currentClusterNumber; i++){
                    FrameData::Cluster cluster;
                    clusters.push_back(cluster); 
                }
                
                for(int i = 0; i < (int) matches.size(); i++){
                    FrameData::Match match = matches[i];
                    for(int polyNum = 0; polyNum < numPolys; polyNum++){
                        if(clusterNumber[polyNum] == -1){
                            continue;
                        }
                        if(pointInPolygon(match.coord2D[0], match.coord2D[1], polygons[polyNum])){
                            clusters[clusterNumber[polyNum]].push_back(i);
                            break;
                        } 
                    } 
                }

                frameData.clusters[model] = clusters;
			}
			
			if( _stepName == "CLUSTER" ) frameData.oldClusters = frameData.clusters;
		}
	};
};
