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

	class CLUSTER_WEIGHTED_MEAN_SHIFT_CPU :public MopedAlg {
		
		Float Radius;
		Float Merge;
		int MinPts;
		int MaxIterations;
        Float halfWeightDistance;
        /*
         * Needs to use 3D points; the 2D points don't have weights 
        bool use3D;
        */

		template< int N >
		struct Canopy {
			Pt<N> center;
            Canopy<N> *merge;
			Canopy<N>( Pt<N> &p){
				center = p;
			} 
		};
		
		template< int N >
        void WeighedMeanShift(vector< FrameData::Cluster > &clusters, 
                vector<Pt<N> > points, 
                vector<Float> weights, 
                Float Radius, Float Merge, int MinPts, int MaxIterations){
            
            if(points.size() == 0){
                return;
            }
			
			Float SqRadius = Radius * Radius;
			Float SqMerge  = Merge  * Merge;

            vector< bool > active;
            active.reserve( points.size() );

			vector< Canopy<N> > can;
			can.reserve( points.size() );

            Float minWeight = 1.0;
            for(int i = 0; i < (int) points.size(); i++){
                if(weights[i] < minWeight){
                    minWeight = weights[i];
                }
            }
			
			for(int i=0; i<(int)points.size(); i++) {
                can.push_back( Canopy<N>(points[i]) );
                active[i] = true;
			}
			
			bool done = false;
            int nIter;
			for( nIter = 0; !done && nIter < MaxIterations; nIter++ ) { // shift canopies to their centroids

				done = true;
			    
                for(int canId = 0; canId < (int) points.size(); canId++){
                    if(!active[canId]){
                        continue;
                    }
                    /* figure out where to shift the canopy to */
                    Float weightSum = 0.0;
                    Pt<N> c;
                    for(int i = 0; i < N; i++){
                        c[i] = 0.0; 
                    }
                    for(int i = 0; i < (int) points.size(); i++){
                        /* if it's within the kernel, include it in the sum */
                        if(can[canId].center.sqEuclDist(points[i]) < SqRadius){
                            c += points[i]*weights[i];
                            weightSum += weights[i];
                        }
                    }
                    /* update the center */
                    can[canId].center = c / weightSum;
				}

                /* merge canopies */
                for(int canId = 0; canId < (int) points.size(); canId++){
                    if(!active[canId]){
                        continue;
                    }
                    for(int otherCanId = canId+1; otherCanId < (int) points.size(); otherCanId++){
						Float dist = can[canId].center.sqEuclDist( can[otherCanId].center );
						if (dist < SqMerge) {
                            active[otherCanId] = false;
                            done = false;
						}
					}
				}
				
			}
            for(int canId = 0; canId < (int) points.size(); canId++){
                if(!active[canId]){
                    continue;
                }
                FrameData::Cluster cluster;
                for(int i = 0; i < (int) points.size(); i++){
                    if(can[canId].center.sqEuclDist( points[i] ) < SqRadius){
                        cluster.push_front(i);
                    }
                }
                if((int) cluster.size() > MinPts){
                    clusters.push_back(cluster); 
                }
            }
		}
		
	public:

		CLUSTER_WEIGHTED_MEAN_SHIFT_CPU(	Float Radius, Float Merge, unsigned int MinPts, unsigned int MaxIterations, Float halfWeightDistance) 
		: Radius(Radius), Merge(Merge), MinPts(MinPts), MaxIterations(MaxIterations), halfWeightDistance(halfWeightDistance)  {
		}

		void getConfig( map<string,string> &config ) const {
		
			GET_CONFIG( Radius );
			GET_CONFIG( Merge );
			GET_CONFIG( MinPts );
			GET_CONFIG( MaxIterations );		
            GET_CONFIG( halfWeightDistance );
		}
		
		void setConfig( map<string,string> &config ) {
			
			SET_CONFIG( Radius );
			SET_CONFIG( Merge );
			SET_CONFIG( MinPts );
			SET_CONFIG( MaxIterations );
            SET_CONFIG( halfWeightDistance );
		}

        Float cauchyWeight(Float d, Float gamma){
            return 1.0 / (1.0 + ((d*d) / (gamma*gamma)));
        }
		
		void process( FrameData &frameData ) {

			frameData.clusters.resize( models->size() );
		
/*			#pragma omp parallel for */
			for( int model=0; model<(int)frameData.matches.size(); model++) {
                /* need one vector to pass into mean shift per image */
                vector< vector< Pt<3> > > pointsPerImage( frameData.images.size());
                vector< vector< Float > > weights(frameData.images.size());
                for(int match = 0; match < (int) frameData.matches[model].size(); match++){
                    int imageIdx = frameData.matches[model][match].imageIdx;
                    pointsPerImage[imageIdx].push_back(frameData.matches[model][match].depthData.coord3D);
                    weights[imageIdx].push_back(cauchyWeight(frameData.matches[model][match].depthData.fillDistance, halfWeightDistance));
                }
                for( int i=0; i<(int)frameData.images.size(); i++ ){
                    WeighedMeanShift(frameData.clusters[model], pointsPerImage[i], weights[i], Radius, Merge, MinPts, MaxIterations);
                }
			}
			
			if( _stepName == "CLUSTER" ) frameData.oldClusters = frameData.clusters;
		}
	};
};
