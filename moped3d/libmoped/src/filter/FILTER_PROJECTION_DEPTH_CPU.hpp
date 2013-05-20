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

	class FILTER_PROJECTION_DEPTH_CPU :public MopedAlg {
	
		int MinPoints;
		Float FeatureDistance;
        Float PlausibleSqDistance;
		Float MinScore;
        Float DepthFraction;
        int TestSampleSize;
        Float MinKeypointFraction;

        /* the test-points for the depth incorect score */
        map<int, vector<Model::IP> > TestPoints;
        bool TestPointsSelected;

	public:

		FILTER_PROJECTION_DEPTH_CPU(int MinPoints, Float FeatureDistance, Float PlausibleSqDistance, 
                                    Float MinScore, Float DepthFraction, int TestSampleSize, 
                                    Float MinKeypointFraction)
		: MinPoints(MinPoints), FeatureDistance(FeatureDistance), PlausibleSqDistance(PlausibleSqDistance), 
          MinScore(MinScore), DepthFraction(DepthFraction), TestSampleSize(TestSampleSize),
          MinKeypointFraction(MinKeypointFraction){
            TestPointsSelected = false;
		}


    	// This function populates "samples" with nSamples references to object correspondences
		// The samples are randomly choosen, and aren't repeated
		template <typename T> void randSample( vector<T> &samples, const vector<T> &population, unsigned int nSamples) {
		
			// Create a vector of samples prefixed with a random int. The int has preference over the pointer when sorting the vector.
			deque< pair< Float, int > > randomSamples;
            for(int i = 0; i < (int) population.size(); i++){
				randomSamples.push_back( make_pair( (Float)rand(), i ) );
            }

			sort( randomSamples.begin(), randomSamples.end() );

            for(int i = 0; (i < (int) nSamples) && (i < (int) population.size()); i++){
                samples.push_back(population[randomSamples.front().second]); 
                randomSamples.pop_front();
            }
		}

        void selectTestPoints(){
            for(int modelNum = 0; modelNum < (int) models->size(); modelNum++){
                cerr << "model Num: " << modelNum << endl;
        		vector < Model::IP > keypoints;
                vector < Model::IP > chosenKeypoints;
                vector < Pt<3> > proj_pts;

                map< string, vector<MopedNS::Model::IP> >::const_iterator itDesc;
                SP_Model model = (*models)[modelNum];
                // Get keypoints for all descriptor types
                for (itDesc = model->IPs.begin(); itDesc != model->IPs.end(); itDesc++){
                    keypoints.insert(keypoints.end(), itDesc->second.begin(), itDesc->second.end());
                }
                if((int) keypoints.size() > TestSampleSize){
                    cerr << "Sampling" << endl;
                    randSample(chosenKeypoints, keypoints, TestSampleSize);
                    TestPoints[modelNum] = chosenKeypoints;
                } else {
                    TestPoints[modelNum] = keypoints;
                }
            }
            TestPointsSelected = true;
        }

		void getConfig( map<string,string> &config ) const {

			GET_CONFIG( MinPoints );
			GET_CONFIG( FeatureDistance );
			GET_CONFIG( MinScore );
            GET_CONFIG( PlausibleSqDistance );
            GET_CONFIG( DepthFraction );
            GET_CONFIG( TestSampleSize );

		}
		
		void setConfig( map<string,string> &config ) {
			
			SET_CONFIG( MinPoints );
			SET_CONFIG( FeatureDistance );
			SET_CONFIG( MinScore );
            SET_CONFIG( PlausibleSqDistance );
            SET_CONFIG( DepthFraction );
            SET_CONFIG( TestSampleSize );

		}
		
		void process( FrameData &frameData ) {

            if(!TestPointsSelected){
                selectTestPoints(); 
            }
		
			vector< SP_Image > &images = frameData.images;
			vector< vector< FrameData::Match > > &matches = frameData.matches;

            // Sanity check (problems when GPU is not behaving)
            if (matches.size() < models->size()) {
                return;
            }

            /* Find the depthmap and its distance fill image */
            SP_Image depthmap;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                SP_Image image = frameData.images[i];
                if(image->imageType == IMAGE_TYPE_DEPTH_MAP){
                    depthmap = image;
                }
            }
            Pt<4> K = depthmap->intrinsicLinearCalibration;

            SP_Image distanceMap;
            /* search for a depthmap fill distance image */
            for(int i = 0; i < (int) frameData.images.size(); i++){
                if(frameData.images[i]->imageType == IMAGE_TYPE_PROB_MAP){
                    if(frameData.images[i]->name == depthmap->name+".distance"){
                        distanceMap = frameData.images[i];
                        break; 
                    }
                }
            }
			
			map< pair<Pt<2>, Image *>, pair< Float, Object *> > bestPoints;
			
			//map< Object *, int > matchesPerObject;
			
			map< Object *, FrameData::Cluster > newClusters;

			
			for(int m=0; m<(int)models->size(); m++) {
				foreach( object, *frameData.objects) {
					if( object->model->name == (*models)[m]->name ) {
                        cerr << "Examining Object: " << endl;
                        cerr << object->model->name << " ";
                        cerr << object->pose << endl;
					
                        /* Compute the quality score */
						FrameData::Cluster &newCluster = newClusters[ object.get() ];
						Float score = 0;
                        int clusterSize = 0;
						for(int match=0; match<(int)matches[m].size(); match++ ) {					
						
							Pt<2> p = project( object->pose, matches[m][match].coord3D, *images[matches[m][match].imageIdx] );
							p -= matches[m][match].coord2D;
							float projectionError = p[0]*p[0]+p[1]*p[1];
							if( projectionError < FeatureDistance ) {
								newCluster.push_back( match );
								score+=1./(projectionError + 1.);
                            }
                            if( projectionError < PlausibleSqDistance) {
                                clusterSize++; 
                            }
						}

                        /* Compute the incorrect penalty */
                        Float IS = 0.0;
                        vector< Model::IP > keypoints = TestPoints[m];
                        int usedKeypointCount = 0;
                        TransformMatrix PoseTM;
                        PoseTM.init( object->pose);
                        for(int kI = 0; kI != (int) keypoints.size(); kI++){
                            Model::IP k = keypoints[kI]; 
                            /* Compute the 3D location and its projection onto the
                             * image */
                            Pt<3> p3D;
                            Pt<2> p2D;

                            PoseTM.transform( p3D, k.coord3D );
                            depthmap->TM.inverseTransform( p3D, p3D );

                            p2D[0] = p3D[0]/p3D[2] * K[0] + K[2];
                            p2D[1] = p3D[1]/p3D[2] * K[1] + K[3];

                            int ix = (int) p2D[0], iy = (int) p2D[1];

                            /* if the point's off of the image */
                            if((ix < 0) || (ix >= depthmap->width) || (iy < 0) || (iy >= depthmap->height)){
                                continue;
                            }

                            Float distance = distanceMap->getProb((int) p2D[0], (int) p2D[1]);
                            if(distance > 0){
                                continue; 
                            }
                            usedKeypointCount++;
                            
                            Float kinectDepth = depthmap->getDepth((int) p2D[0], (int) p2D[1]),
                                  putativeDepth = p3D[2];

                            /* if the kinect spots something in front of the point, it might be
                             * an occlusion */
                            if(kinectDepth < putativeDepth){
                                continue;
                            }
                            
                            Float cauchyScale = DepthFraction*kinectDepth;

                            Float term = (putativeDepth - kinectDepth) / cauchyScale;
                            term *= term;

                            /* on the other hand, if the point's too far away... */
                            IS += 1.0 - (1.0 / (1.0 + term));
                        }
                        cerr << "IS: " << IS << endl;
                        cerr << "used keypointCount" << usedKeypointCount << ";" << keypoints.size() << " (MinFraction = " << MinKeypointFraction << ") ";
                        cerr << "MinNum: " << (MinKeypointFraction*keypoints.size()) << endl;
                        /* If we don't have enough valid keypoints to use an incorrect score */
                        if(usedKeypointCount <= (int) (MinKeypointFraction*keypoints.size())){
                            IS = 0;
                        } else {
                            cerr << "IS: " << IS << " / " << usedKeypointCount << " ; " << "Cluster size: " << clusterSize << endl;
                            /* We now have the incorrect score over usedKeypointCount; make it
                             * over the number of plausible matches */
                            IS *= ((Float) clusterSize)/usedKeypointCount;
                        }

                            
						object->score = score - IS;
                        cerr << "Score:  " << score << " - " << IS << " => " << object->score << endl;
					        
						// Go through the list of matches, and transfer each potential match to the object with max score
						foreach( match, newCluster ) {
							
							pair< Float, Object *> &point = bestPoints[make_pair(matches[m][match].coord2D, images[matches[m][match].imageIdx].get())];
							if( point.first < score ) {
								
								//matchesPerObject[point.second]=max(0, matchesPerObject[point.second]-1);
								
								point.first = score;
								point.second = object.get();
								
								//matchesPerObject[point.second]++;
							}
						}
					}
				}
			}

			// Now put only the best points in each cluster
			newClusters.clear();
			for(int m=0; m<(int)models->size(); m++) {
				for (int match = 0; match < (int) matches[m].size(); match++) {
					// Find out the object that this point belongs to, and put it in the object's cluster
					pair< Float, Object *> &point = bestPoints[make_pair(matches[m][match].coord2D, images[matches[m][match].imageIdx].get())];
					if ( point.second != NULL && point.second->model->name == (*models)[m]->name )
						newClusters[ point.second ].push_back( match );
				}
			} 
			
			frameData.clusters.clear();
			frameData.clusters.resize( models->size() );

			for(int m=0; m<(int)models->size(); m++) {
				eforeach( object, object_it, *frameData.objects) {
					if( object->model->name == (*models)[m]->name ) {
						// Delete object instance if not reliable
						if( (int)newClusters[object.get()].size() < MinPoints || object->score < MinScore) {
						//if( matchesPerObject[object.get()] < MinPoints || object->score < MinScore) {
                        
                            cerr << "Deleting object(" << object->model->name << " ;  " << object->pose << ") ";
                            cerr << (int)newClusters[object.get()].size() << " ? " << MinPoints << " ; ";
                            cerr << object->score << " ? " << MinScore << endl;
							object_it = frameData.objects->erase(object_it);
					        } else {
                                cerr << "Keeping object(" << object->model->name << " ;  " << object->pose << ") ";
                                cerr << (int)newClusters[object.get()].size() << " ? " << MinPoints << " ; ";
                                cerr << object->score << " ? " << MinScore << endl;


							frameData.clusters[m].push_back( newClusters[object.get()] );
						}
					}
				}
			}
            cerr << "=====\n=====\n=====\n";
					
		}
	};
};
