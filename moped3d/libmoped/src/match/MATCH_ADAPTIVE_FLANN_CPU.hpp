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

	class MATCH_ADAPTIVE_FLANN_CPU:public MopedAlg {

		static inline void norm( vector<float> &d ) {
			float norm=0; for (int x=0; x<(int)d.size(); x++) norm += d[x]*d[x]; norm = 1/sqrtf(norm);
			for (int x=0; x<(int)d.size(); x++) d[x] *=norm;
		}
		
		int DescriptorSize;
		string DescriptorType;
        int NumTrees;

        /* The minimum ratio (at 0m away) */
        Float MinRatioMin;
        Float MinRatioMax;

        /* The maximum ratio; used between 200 and 100 pixels */
        Float MaxRatioMin;
        Float MaxRatioMax;

        Float sigmoidTranslate;
        Float sigmoidScale;

        Float DimensionPeak;
        Float DimensionFade;

        /* After this distance, auto-reject */
        Float MaximumDepth;

        Float DefaultDepth;
        Float CauchyScale;

		bool skipCalculation;


		vector< pair<int, Pt<3> *> > modelPointData;

        /* used in the calculation of the adaptive ratio;
         *
         *        { ratioLow -> ratioHigh   : 0 d < maxRatioDepth
         * f(d) = { ratioHigh               : maxRatioDepth <= d <= minRatioDepth
         *        { rapidly down to 0       : minRatioDepth < d
         *
         * */
        vector<float> maxRatioDepths;
        vector<float> minRatioDepths;
        vector<float> ratioLows;
        vector<float> ratioHighs;
	
		shared_ptr<cv::flann::Index> index_id[MAX_THREADS];
		cv::Mat mat;
	
		void Update(FrameData &frameData) {

            sigmoidTranslate = 1750;
            sigmoidScale = 250;

            MaximumDepth = 4.0;
            DefaultDepth = 1.0;
            CauchyScale = 0.1;
			
			skipCalculation = true;
			
			if( models==NULL ) return;
			
			modelPointData.clear();
			vector<float> dataset;


			for( int nModel = 0; nModel < (int)models->size(); nModel++ ) {
				
				vector<Model::IP> &IPs = (*models)[nModel]->IPs[DescriptorType];
				
				for( int nFeat = 0; nFeat < (int)IPs.size(); nFeat++ ) {
					
					norm( IPs[nFeat].descriptor );
					for( int i = 0; i < (int)IPs[nFeat].descriptor.size(); i++ )
						dataset.push_back( IPs[nFeat].descriptor[i] );
				
					modelPointData.push_back( make_pair( nModel, &IPs[nFeat].coord3D ) );
				}
			}
			mat = cv::Mat( dataset.size()/DescriptorSize, DescriptorSize, CV_32F);
			
			if( modelPointData.size() > 1 ) { 
				skipCalculation = false;

				for(int x=0; x<(int)dataset.size(); x++) mat.at<float>(x/DescriptorSize,x%DescriptorSize)=dataset[x];
				
				#pragma omp parallel for
				for(int x=0; x<MAX_THREADS; x++) 
                    index_id[x]=shared_ptr<cv::flann::Index>( new cv::flann::Index( mat, cv::flann::KDTreeIndexParams(NumTrees) ) );
                    // dfouhey: using default values for everything except precision
                    // index_id[x]=shared_ptr<cv::flann::Index>( new cv::flann::Index( mat, cv::flann::AutotunedIndexParams(Precision, 1.0 / 100, 1.0 / 100, 0.01)));
			}

            /* get an image for the intrinsics */
            SP_Image grayImage;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                SP_Image image = frameData.images[i];
                if(image->imageType == IMAGE_TYPE_GRAY_IMAGE){
                    grayImage = image; 
                } 
            }

            Float MinRatioRange = MinRatioMax - MinRatioMin,
                  MaxRatioRange = MaxRatioMax - MaxRatioMin;

            /* now do the computations for the adaptive matching */
            for(int modelNum = 0; modelNum < (int) models->size(); modelNum++){
                SP_Model model = (*models)[modelNum];
                Float depth200Pix = solveProjectionDepth(model, grayImage->intrinsicLinearCalibration, DimensionPeak, 100, 0.01);
                Float depth100Pix = solveProjectionDepth(model, grayImage->intrinsicLinearCalibration, DimensionFade, 100, 0.01);
                Float featureCount = (int) model->IPs[DescriptorType].size();
                maxRatioDepths.push_back(depth200Pix);
                minRatioDepths.push_back(depth100Pix);
                /* we use the density to compute the high / low values; min/maxRatioDepths tells us where
                 * the function changes; this gives us the values */
                Float densityAdjust = canonicalSigmoid( (sigmoidTranslate - featureCount) / sigmoidScale);
                /* push back the ratio function control points */
                Float MinRatio = MinRatioMin+densityAdjust*MinRatioRange,
                      MaxRatio = MaxRatioMin+densityAdjust*MaxRatioRange; 
                /*
                cerr << model->name << ": " << featureCount << endl; 
                cerr << model->name << ": " << MinRatio << " => " << MaxRatio << endl;  
                */
                ratioLows.push_back(MinRatio);
                ratioHighs.push_back(MaxRatio);
            }


			configUpdated = false;
		}

        /*
         * \brief The usual sigmoid function.
         *
         * The function f(x) 1 / (1 + \exp(-x)) 
         *   f: \mathbb{R} -> (0,1)
         */
        Float canonicalSigmoid(Float x){
            return 1.0 / (1.0 + exp(-1.0 * x));
        }

        Float getRatio(Float depth, int modelNum){
            /* We can automatically reject a large number of points this way */
            if(depth > MaximumDepth){
                return 0.0;
            }
            /* get the relevant control points */
            Float maxRatioDepth = maxRatioDepths[modelNum],
                  minRatioDepth = minRatioDepths[modelNum],
                  ratioLow = ratioLows[modelNum],
                  ratioHigh = ratioHighs[modelNum];
            if(depth < maxRatioDepth){
                Float progress = depth / maxRatioDepth;
                /* linearly interpolate */
                return ratioLow + progress*(ratioHigh - ratioLow);
            } else if(depth < minRatioDepth){
                return ratioHigh; 
            } else if(depth < minRatioDepth*2){
                /* linearly interpolate down to 0 */
                Float progress = (minRatioDepth*2 - depth) / minRatioDepth;
                return progress*ratioHigh;
            } 
            return 0.0;
        }


        Float getFeatureDensity(SP_Model model){
            /* approximate the surface area with the surface area of the bounding box */
            Float minX = model->boundingBox[0][0], maxX = model->boundingBox[1][0], 
                  minY = model->boundingBox[0][1], maxY = model->boundingBox[1][1],
                  minZ = model->boundingBox[0][2], maxZ = model->boundingBox[1][2];
            /* get the range in each dimension */
            Float xRange = maxX - minX, yRange = maxY - minY, zRange = maxZ - minZ;
            /* twice each of the types of surfaces on the bounding box */
            Float surfaceArea = 2*(xRange*yRange+xRange*zRange+yRange*zRange);
            /* the feature count */
            Float featureCount = (int) model->IPs[DescriptorType].size();
            /* 
            cerr << model->name << ": Feature Count=" << featureCount << "; Feature Density=" << featureCount / surfaceArea << endl; 
            */
            return featureCount / surfaceArea;
        }

        /* 
         * \brief Get the area of the projection of a rectangle in the world onto the image plane.
         *
         * Returns the area of the given rectangle projected onto the image plane, under the
         * assumption that the projection is a parallelogram.
         */
        Float getProjectedArea(Pt<4> k, Pt<3> points[4]){
            Pt<2> projectedPoint;
            Pt<2> minVal, maxVal;
            for(int i = 0; i < 4; i++){
                Pt<3> p = points[i];
                /* project the point; 
                 * w = p[2], u = uw / w */
                Float u = (k[0]*p[0] + k[2]*p[2]) / p[2];
                Float v = (k[1]*p[1] + k[3]*p[2]) / p[2];
                projectedPoint.init(u,v);
                if(i == 0){
                    minVal = projectedPoint; maxVal = projectedPoint; 
                } else {
                    minVal = min(minVal, projectedPoint);
                    maxVal = max(maxVal, projectedPoint);
                }
            }
            return (maxVal[0] - minVal[0])*(maxVal[1] - minVal[1]);
        }

        /*
         * \brief Get a notion of the length of an object at a particular depth.
         *
         * Returns an approximation of the length of the size of an object. Specifically,
         * the average square root of the projection of the surfaces of the bounding box. 
         *
         */
        Float getAverageProjectedLength(Pt<3> bbox[2], Pt<4> camIntrinsics, Float depth){
            Float minX = bbox[0][0], maxX = bbox[1][0], 
                  minY = bbox[0][1], maxY = bbox[1][1],
                  minZ = bbox[0][2], maxZ = bbox[1][2];

            /* center the coordinates */
            Float xRange = (maxX - minX), yRange = (maxY - minY), zRange = (maxZ - minZ);
            minX = -xRange/2; maxX = xRange/2; 
            minY = -yRange/2; maxY = yRange/2;
            minZ = -zRange/2; maxZ = zRange/2;

            /* construct the 3D surfaces and then compute the square root of the projected
             * area */

            /*
             *           -----           - = object
             *             |
             *             |
             *             *             * = camera location
             *
             * Note that z should be the depth; we've centered the object on 
             * (0,0) in x/y, and so the depth of the object (i.e., distance from
             * the camera) is depth.
             *
             */

            Pt<3> surface[4];

            Float zConstSurf = (maxX - minX)*(maxY - minY),
                  yConstSurf = (maxX - minX)*(maxZ - minZ),
                  xConstSurf = (maxY - minY)*(maxZ - minZ);

            if((zConstSurf >= xConstSurf) && (zConstSurf >= yConstSurf)){
                /* surface 1: hold Z constant */
                surface[0].init(minX, minY, depth); surface[1].init(minX, maxY, depth);
                surface[2].init(maxX, maxY, depth); surface[3].init(maxX, minY, depth);
                return sqrt(getProjectedArea(camIntrinsics, surface));
            } else if((yConstSurf >= xConstSurf) && (yConstSurf >= zConstSurf)){
                /* surface 2: hold Y constant */
                surface[0].init(minX, minZ, depth); surface[1].init(minX, maxZ, depth);
                surface[2].init(maxX, maxZ, depth); surface[3].init(maxX, minZ, depth);
                return sqrt(getProjectedArea(camIntrinsics, surface));
            } 
            /* surface 3: hold X constant */
            surface[0].init(minY, minZ, depth); surface[1].init(minY, maxZ, depth);
            surface[2].init(maxY, maxZ, depth); surface[3].init(maxY, minZ, depth);
            return sqrt(getProjectedArea(camIntrinsics, surface));
        }
            

        /* \brief Find the depth at which the object is approximately targetLength pixels high or wide.
         *
         * More or less, solve getAverageProjectedLength(depth) == targetLength. Fortunately, the function
         * is very well behaved, and we can solve it with a binary search extremely efficiently. 
         *
         */
        Float solveProjectionDepth(SP_Model model, Pt<4> camIntrinsics, Float targetLength, int iters, Float tolerance){
            Float left = 0.0, right = 2.0;
            int iter = 0;
            /* get a right-bound if necessary */
            while(iter < iters){
                iter++;
                Float length = getAverageProjectedLength(model->boundingBox, camIntrinsics, right);
                if(length > targetLength){
                    right *= 2;
                } else {
                    break;
                }
            }

            /* Get the maximum error */
            Float maxError = targetLength*tolerance;

            /* now divide up the search window */
            while(iter < iters){
                iter++;
                Float middle = (left+right) / 2;
                Float length = getAverageProjectedLength(model->boundingBox, camIntrinsics, middle);
                /* If we've reached our target within the given tolerance, return it */
                if( fabs(length - targetLength) < maxError ){
                    return middle; 
                } 
                /* otherwise, change our search window, depending on whether we've overshot or undershot */
                if(length > targetLength){
                    left = middle;
                } else {
                    right = middle;
                }
            }
            return (left+right) / 2; 

        }

        Float getAdjustedRatio(int x, int y, SP_Image depthmap, SP_Image distanceMap, int modelNumber){
            /* compute the cauchy weighting */
            Float weightTerm = distanceMap->getProb(x, y) / CauchyScale;
            Float weight = 1.0 / (1.0 + weightTerm*weightTerm);
            /* Get the ratio at the putative and default depths */
            Float putativeRatio = getRatio(depthmap->getDepth(x, y), modelNumber);
            Float defaultRatio = getRatio(DefaultDepth, modelNumber);
            /*
            #pragma omp critical
            {
                cerr << "Adjusted ratio (w = " << weight << ";d = " << depthmap->getDepth(x,y) << " ): " << weight*putativeRatio + (1.0 - weight)*defaultRatio;
                cerr << "(Default = " << defaultRatio << ")" << endl;
            }
            */
            return weight*putativeRatio + (1.0 - weight)*defaultRatio; 
        }

				
	public:


		MATCH_ADAPTIVE_FLANN_CPU( int DescriptorSize, string DescriptorType, int NumTrees, 
                        Float MinRatioMin, Float MinRatioMax, 
                        Float MaxRatioMin, Float MaxRatioMax,
                        Float DimensionPeak, Float DimensionFade) :   DescriptorSize(DescriptorSize), DescriptorType(DescriptorType), NumTrees(NumTrees), 
            MinRatioMin(MinRatioMin), MinRatioMax(MinRatioMax), MaxRatioMin(MaxRatioMin), MaxRatioMax(MaxRatioMax),
            DimensionPeak(DimensionPeak), DimensionFade(DimensionFade){


			skipCalculation=false;
		}

		void getConfig( map<string,string> &config ) const {
		
            GET_CONFIG(MinRatioMin);
            GET_CONFIG(MinRatioMax);
            GET_CONFIG(MaxRatioMin);
            GET_CONFIG(MaxRatioMax);
			GET_CONFIG(DimensionPeak);
			GET_CONFIG(DimensionFade);
			GET_CONFIG(NumTrees);
			GET_CONFIG(DescriptorType);
			GET_CONFIG(DescriptorSize);

		};
		
		void setConfig( map<string,string> &config ) {
	        SET_CONFIG(MinRatioMin);
            SET_CONFIG(MinRatioMax);
            SET_CONFIG(MaxRatioMin);
            SET_CONFIG(MaxRatioMax);
			SET_CONFIG(DimensionPeak);
			SET_CONFIG(DimensionFade);
			SET_CONFIG(NumTrees);
			SET_CONFIG(DescriptorType);
			SET_CONFIG(DescriptorSize);

		};


		void process( FrameData &frameData ) {

			if( configUpdated ) Update(frameData);
			
			if( skipCalculation ) return;
				
			vector< FrameData::DetectedFeature > &corresp = frameData.detectedFeatures[DescriptorType];
			if( corresp.empty() ) return;
			
			vector< vector< FrameData::Match > > &matches = frameData.matches;
			
			vector< vector< vector< FrameData::Match > > > threadedMatches( MAX_THREADS, vector< vector< FrameData::Match > >( models->size() ) );


            /* Find a depthmap */
            SP_Image depthmap;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                SP_Image image = frameData.images[i];
                if(image->imageType == IMAGE_TYPE_DEPTH_MAP){
                    depthmap = image;
                }
            }
            /* and its distance map */
            SP_Image distanceMap;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                if(frameData.images[i]->imageType == IMAGE_TYPE_PROB_MAP){
                    if(frameData.images[i]->name == depthmap->name+".distance"){
                        distanceMap = frameData.images[i];
                        break; 
                    }
                }
            }


			#pragma omp parallel for
			for( int i=0; i<(int)corresp.size(); i++)  {
				
				int threadNum = omp_get_thread_num();
				vector< vector< FrameData::Match > > &matches = threadedMatches[ threadNum ];
			
				vector<float> desc(DescriptorSize);
				vector<int> nx(2);
				vector<float> dx(2);

                /* get the location of the feature in the image plane */
                Pt<2> loc2D = corresp[i].coord2D;
                int x = (int) loc2D[0], y = (int) loc2D[1];
                x = min( max(x,0), depthmap->width);
                y = min( max(y,0), depthmap->height);


				norm( corresp[i].descriptor );
				for(int x=0; x<DescriptorSize; x++ ) desc[x]=corresp[i].descriptor[x];

                Float depth = depthmap->getDepth(x, y);
                /* Don't even bother searching */
                if(depth > MaximumDepth){
                    continue;
                }
				
				index_id[threadNum]->knnSearch(desc, nx, dx, 2, cv::flann::SearchParams(32) );

                int nModel = modelPointData[nx[0]].first;

                Float Ratio = getAdjustedRatio(x, y, depthmap, distanceMap, nModel);

				if(  dx[0]/dx[1] < Ratio ) {				

					
					matches[nModel].resize( matches[nModel].size() +1 );
					
					FrameData::Match &match = matches[nModel].back();

                    COPY_FEATURE_TO_MATCH(corresp[i], match);
					match.coord3D = *modelPointData[nx[0]].second;
				}
			}
            
			matches.resize( models->size() );
			for(int nModel = 0; nModel < (int)models->size(); nModel++) {

				int sz = 0;
				for(int t=0; t<MAX_THREADS; t++)
					sz += threadedMatches[t][nModel].size();

				matches[nModel].resize(sz);

				sz = 0;
				for(int t=0; t<MAX_THREADS; t++) {
					memcpy( &matches[nModel][sz], &threadedMatches[t][nModel][0], sizeof(FrameData::Match) * threadedMatches[t][nModel].size() );
					sz += threadedMatches[t][nModel].size();
				}
			}
		}
	};
};
