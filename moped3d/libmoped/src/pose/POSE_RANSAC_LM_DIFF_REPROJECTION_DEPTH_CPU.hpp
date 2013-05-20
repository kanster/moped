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
#include <lm.h>

#ifdef USE_DOUBLE_PRECISION
    #define LEVMAR_DIF dlevmar_dif
#else
    #define LEVMAR_DIF slevmar_dif
#endif

namespace MopedNS {

	class POSE_RANSAC_LM_DIFF_REPROJECTION_DEPTH_CPU :public MopedAlg {
	
		int MaxRANSACTests; 		// e.g. 500
		int MaxLMTests;     		// e.g. 500
		int MaxObjectsPerCluster; 	// e.g. 4
		int NPtsAlign; 			// e.g. 5
		int MinNPtsObject; 		// e.g. 8
		Float ErrorThreshold; 		// e.g. 5

        static const Float FillInCauchyScale = 25.0;  //The scale parameter for the cauchy weighting function
        static const Float MaximumError = 0.5;        //The maximum error

    
    		struct LmData {

			Image *image;
			
			Pt<2> coord2D;
			Pt<3> coord3D;
            Pt<3> world3D;
            Float cauchyWeight;
		};
		
		// This function populates "samples" with nSamples references to object correspondences
		// The samples are randomly choosen, and aren't repeated
		bool randSample( vector<LmData *> &samples, const vector<LmData *> &cluster, unsigned int nSamples) {
		
			// Do not add a correspondence of the same image at the same coordinate
			map< pair<Image *, Pt<2> >, int > used;
		
			// Create a vector of samples prefixed with a random int. The int has preference over the pointer when sorting the vector.
			deque< pair< Float, LmData * > > randomSamples;
			foreach( match, cluster )
				randomSamples.push_back( make_pair( (Float)rand(), match ) );
			sort( randomSamples.begin(), randomSamples.end() );
			
			while( used.size() < nSamples && !randomSamples.empty() ) {

				pair<Image *, Pt<2> > imageAndPoint( randomSamples.front().second->image, randomSamples.front().second->coord2D );
				
				if( !used[ imageAndPoint ]++ )
					samples.push_back( randomSamples.front().second );
				
				randomSamples.pop_front();
			}
			
			return used.size() == nSamples;
		}
		
		static void lmFuncQuat(Float *lmPose, Float *errors, int nPose, int nErrors, void *data) {
		
			vector<LmData *> &lmData = *(vector<LmData *> *)data;
			
			Pose pose;
			
			pose.rotation.init( lmPose );
			pose.rotation.norm();
			pose.translation.init( lmPose + 4 );
			
			TransformMatrix PoseTM;
			PoseTM.init( pose );			
			
			for( int i=0; i<nErrors/3; i++ ) {

				Pt<3> p3D;
				PoseTM.transform( p3D, lmData[i]->coord3D );
				lmData[i]->image->TM.inverseTransform( p3D, p3D );

                Pt<4> K = lmData[i]->image->intrinsicLinearCalibration;

				Pt<2> p;
				p[0] = p3D[0]/p3D[2] * K[0] + K[2];
				p[1] = p3D[1]/p3D[2] * K[1] + K[3];
	
                /* Do the 2D errors */
				if( p3D[2] < 0 ) {
					errors[3*i  ] = -p3D[2] + 10;
					errors[3*i+1] = -p3D[2] + 10;
                    errors[3*i+2] = -p3D[2] + 10; 
				} else {
                    /*
                     * How to get errors in meters:
                     *
                     * So, p and lmData[i]->coord2D are points on the image 
                     * plane; we project them out into the world with K^{-1}
                     *
                     *     [ fx  0   cx ]             [ 1/fx  0     -cx/fx ]
                     * K = [ 0   fy  cy ] => K^{-1} = [ 0     1/fy  -cy/fy ]
                     *     [ 0   0   1  ]             [ 0     0     1      ]
                     *
                     * Then [x y z]^T = K^{-1} [u v 1]^T
                     *
                     * x = u / fx + 1 * -cx/fx = (u - cx) / fx
                     * y = (similarly)         = (v - cy) / fy
                     * z = 1
                     *
                     * Given points p and q on the image plane, both are projected
                     * onto the same plane in the world. Then the distance is only
                     * in x and y. The difference in x in the world is:
                     *
                     *
                     * (p[0] - cx) / fx - (q[0] - cx) / fx = 
                     * (p[0] - cx - q[0] + cx) / fx = 
                     * (p[0] - q[0]) / fx
                     * and similarly for y.
                     *
                     * So we just take the distance in the image plane, and divide it by 
                     * the focal length.
                     */

                    Float dx = (p[0] - lmData[i]->coord2D[0]);
                    Float dy = (p[1] - lmData[i]->coord2D[1]);
					errors[3*i]   = dx*dx;
					errors[3*i+1] = dy*dy;
				}


                /* If vec is the vector extending from the camera center
                 * through (u,v) in the image plane, then the projection of the
                 * world point P onto that line (and thus the depth error is 
                 * given by vec vec^T P (under the assumption that the camera
                 * is placed at 0
                 */
                Float vec1 = p3D[0], vec2 = p3D[1], vec3 = p3D[2];
                
                /* Ok - so now compute (vec^T P); this is essentially the scaling factor
                 * to stretch vec to the projection */
                Float vecTP = vec1*lmData[i]->world3D[0] + vec2*lmData[i]->world3D[1] + vec3*lmData[i]->world3D[2];
               
                /* compute the world point projected onto the line */
                Pt<3> projWorld;
                projWorld.init(vec1*vecTP, vec2*vecTP, vec3*vecTP);
               
                /* compute the difference between it and the 3D point */
                Float depthError = projWorld.euclDist(p3D);

                Float adjustedDepthError = depthError;
                
                errors[3*i+2] = adjustedDepthError*adjustedDepthError;

                errors[3*i+2] *= 50;

                /* Adjust the errors with the weights */
                Float wi = lmData[i]->cauchyWeight;

                /*
                #pragma omp critical 
                {
                    cerr << "!!!" << errors[3*i] << " " << errors[3*i+1] << " " << errors[3*i+2] << endl;
                }
                */

                Float weight3D = (1 - Alpha)*wi; 

                errors[3*i+0] *= (1 - weight3D);
                errors[3*i+1] *= (1 - weight3D); 
                errors[3*i+2] *= weight3D;

			}
		}

        /*
         * \brief Get the weight of a data point given its fill-in distance */
        Float getCauchyWeight(Float distance){
            Float factor = distance / FillInCauchyScale;
            return 1.0 / (1 + factor*factor);
        }

        static Float sensorModelAdjust(Float depth, Float error){
            /* 
             * error / (\sigma_{1} / \sigma{depth}} => error @ 1m
             * = error / (0.0035*1*1 / (0.0035*depth*depth)) = error / (0.0035 / (0.0035*depth*depth))
             * = error / (depth*depth) 
             */
            return error / (depth*depth);
        }

        static Float errorBound(Float error){
            if(error > MaximumError){
                return MaximumError;
            }
            return error; 
        }

		
		Float optimizeCamera( Pose &pose, const vector<LmData *> &samples, const int maxLMTests ) {

			// set up vector for LM
			Float camPoseLM[7] = {
				pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3],
				pose.translation[0], pose.translation[1], pose.translation[2] };
			
			// LM expects errors as a single vector
			vector<Float> errors( samples.size()*3, 0 );
		
			Float info[LM_INFO_SZ];
				
			// call levmar
			int retValue = LEVMAR_DIF(lmFuncQuat, camPoseLM, &errors[0], 7, samples.size()*3, maxLMTests, 
							NULL, info, NULL, NULL, (void *)&samples);

			if( retValue < 0 ) return retValue;
			
			pose.rotation.init( camPoseLM );
			pose.translation.init( camPoseLM + 4 );
			
			pose.rotation.norm();
			// output is in camPoseLM
			return info[1];
		}
				
		void testAllPoints( vector<LmData *> &consistentCorresp, const Pose &pose, const vector<LmData *> &testPoints, const Float ErrorThreshold ) {
			
			consistentCorresp.clear();
						
			foreach( corresp, testPoints ) {

				Pt<2> p = project( pose, corresp->coord3D, *corresp->image );
				p-=corresp->coord2D;
				
				Float projectionError = p[0]*p[0]+p[1]*p[1];

				if( projectionError < ErrorThreshold )
					consistentCorresp.push_back(corresp);
			}
		}

		void initPose( Pose &pose, const vector<LmData *> &samples ) {
			
			pose.rotation.init( (rand()&255)/256., (rand()&255)/256., (rand()&255)/256., (rand()&255)/256. );
            Pt<3> sum;
            sum.init(0.0, 0.0, 0.0);
            int n = (int) samples.size();
            for(int i = 0; i < n; i++){
                sum += samples[i]->world3D; 
            }
			pose.translation.init( sum[0] / n, sum[1] / n, sum[2] / n);
		}
		
		bool RANSAC( Pose &pose, const vector<LmData *> &cluster ) {

			vector<LmData *> samples;
			for ( int nIters = 0; nIters<MaxRANSACTests; nIters++) {

				samples.clear();
				if( !randSample( samples, cluster, NPtsAlign ) ) return false;
				
				initPose( pose, samples );
					
                Pose initialPose(pose.rotation, pose.translation);
				int LMIterations = optimizeCamera( pose, samples, MaxLMTests );
                Pose optimizedPose(pose.rotation, pose.translation);
				if( LMIterations == -1 ) continue;

				
				vector<LmData *> consistent;
				testAllPoints( consistent, pose, cluster, ErrorThreshold );

				if ( (int)consistent.size() > MinNPtsObject ) {
					
					optimizeCamera( pose, consistent, MaxLMTests );
					return true;
				}		
			}
			return false;
		}

		void preprocessAllMatches( vector< vector< LmData > > &optData, const vector< vector< FrameData::Match > > &matches, const vector< SP_Image > &images ) {
			
			optData.resize( matches.size() );
			for(int model=0; model<(int)matches.size(); model++ )
				optData[model].resize( matches[model].size() );	
			
			
			vector< pair<int,int> > tasks; tasks.reserve(1000);
			for(int model=0; model<(int)matches.size(); model++ )
				for(int match=0; match<(int)matches[model].size(); match++ )
					tasks.push_back( make_pair(model, match) );
				
			#pragma omp parallel for
			for(int task=0; task<(int)tasks.size(); task++) {
				
				
				int model=tasks[task].first;
				int match=tasks[task].second;
				const SP_Image &image = images[ matches[model][match].imageIdx ];
				
				optData[model][match].image = image.get();
				optData[model][match].coord2D = matches[model][match].coord2D;
				optData[model][match].coord3D = matches[model][match].coord3D;
                optData[model][match].world3D = matches[model][match].depthData.coord3D;
                optData[model][match].cauchyWeight = getCauchyWeight(matches[model][match].depthData.fillDistance);
			}
		}

	public:
        //The 2D trade-off parameter; how much weight to give the
        //2D information if our 3D information is completely accurate
        static Float Alpha;               


		POSE_RANSAC_LM_DIFF_REPROJECTION_DEPTH_CPU( int MaxRANSACTests, int MaxLMTests, int MaxObjectsPerCluster, int NPtsAlign, int MinNPtsObject, Float ErrorThreshold, Float _Alpha )
		: MaxRANSACTests(MaxRANSACTests), MaxLMTests(MaxLMTests), MaxObjectsPerCluster(MaxObjectsPerCluster), NPtsAlign(NPtsAlign), 
		  MinNPtsObject(MinNPtsObject), ErrorThreshold(ErrorThreshold) {
              Alpha = _Alpha;
		}

		void getConfig( map<string,string> &config ) const {
			
			GET_CONFIG( MaxRANSACTests );
			GET_CONFIG( MaxLMTests );
			GET_CONFIG( NPtsAlign );
			GET_CONFIG( MinNPtsObject );
			GET_CONFIG( ErrorThreshold );
		}
		
		void setConfig( map<string,string> &config ) {

			SET_CONFIG( MaxRANSACTests );
			SET_CONFIG( MaxLMTests );
			SET_CONFIG( NPtsAlign );
			SET_CONFIG( MinNPtsObject );
			SET_CONFIG( ErrorThreshold );
		}


		void process( FrameData &frameData ) {


			
			int NObjectsCluster = MaxObjectsPerCluster;

			vector< SP_Image > &images = frameData.images;
			vector< vector< FrameData::Match > > &matches = frameData.matches;
			vector< vector< FrameData::Cluster > >&clusters = frameData.clusters;

			vector< vector< LmData > > lmData;
			preprocessAllMatches( lmData, matches, images );
			
			vector< pair<int,int> > tasks; tasks.reserve(1000);
			for(int model=0; model<(int)clusters.size(); model++ ){
				for(int cluster=0; cluster<(int)clusters[model].size(); cluster++ )
					for(int obj=0; obj<NObjectsCluster; obj++)
						tasks.push_back( make_pair(model, cluster) );
			}
			
			#pragma omp parallel for 
			for(int task=0; task<(int)tasks.size(); task++) {
			
				int model=tasks[task].first;
				int cluster=tasks[task].second;
	
				vector<LmData *> cl;
				foreach( point, clusters[model][cluster] )
					cl.push_back( & lmData[model][ point ] );
				
				Pose pose;
				bool found = RANSAC( pose, cl );
				if( found > 0 ) 
				#pragma omp critical(POSE)
				{

					SP_Object obj(new Object);
					frameData.objects->push_back(obj);
					
					obj->pose = pose;
					obj->model = (*models)[model];
				}
			}
			
			if( _stepName == "POSE" ) frameData.oldObjects = *frameData.objects;
		}
	};

    Float POSE_RANSAC_LM_DIFF_REPROJECTION_DEPTH_CPU::Alpha;
    
};
                                                

