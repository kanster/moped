/*
 * David Fouhey
 * Depthfilter
 */

#pragma once

#include <math.h>

#define DEBUG 0


namespace MopedNS {

	class DEPTHFILTER_CPU: public MopedAlg {
            int PatchSize;
            Float Density;
            int ToFilter;

	    public:

        DEPTHFILTER_CPU(int PatchSize, Float Density, int ToFilter): PatchSize(PatchSize), Density(Density), ToFilter(ToFilter) { 
        }
		
		void getConfig( map<string,string> &config ) const {
            GET_CONFIG( PatchSize );
            GET_CONFIG( Density );
            GET_CONFIG( ToFilter );
		}
			
		void setConfig( map<string,string> &config ) {
            SET_CONFIG( PatchSize ); 
            SET_CONFIG( Density ); 
            SET_CONFIG( ToFilter ); 
		}

        /* \brief Project a point into the world.
         */
        inline void projectPoint(Float u, Float v, Float depth, Pt<4> K, Pt<3> &ret){
            Float x = (u - K[2]) / K[0]*depth,
                  y = (v - K[3]) / K[1]*depth,
                  z = depth;
            ret.init(x, y, z);
        }

        /* \brief Get the area of a rectangle projected from the image plane at 
         * the given depth.
         *
         * Compute the area of the projection of quadrilateral (x0,y0) <-> (x1,y1)
         * from the image plane into the world at the given depth. Requires the 
         * depthmap for camera parameters. 
         */
        Float getArea(Pt<4> K, int x0, int y0, int x1, int y1, Float depth){
            Pt<3> quadCorners[4];
            /* project the 3 corners of the quadrilateral */
            projectPoint(x0, y0, depth, K, quadCorners[0]);
            projectPoint(x0, y1, depth, K, quadCorners[1]);
            projectPoint(x1, y0, depth, K, quadCorners[2]);
            Float w = quadCorners[0].euclDist(quadCorners[2]),
                  h = quadCorners[0].euclDist(quadCorners[1]);
            return w*h; 
        }

        Float clamp(Float v, Float minVal, Float maxVal){
            if(v < minVal){
                return minVal;
            } else if(v > maxVal){
                return maxVal;
            }
            return v; 
        }

        /*
         * \brief Dilate an array; this needs to allocate a copy 
         */
        void dilate(Float *array, int width, int height){
            int N = width*height;
            Float *copy = new Float[N];
            for(int i = 0; i < N; i++){
                copy[i] = array[i];
            }
            /* iterate through the pixels */
            for(int y = 0; y < height; y++){
                for(int x = 0; x < width; x++){
                    /* iterate through the neighboring pixels */
                    for(int dy = -1; dy <= 1; dy++){
                        int yp = y+dy;
                        if((yp < 0) || (yp >= height)){
                            continue;
                        }
                        for(int dx = -1; dx <= 1; dx++){
                            int xp = x+dx;
                            if((xp < 0) || (xp >= width)){
                                continue; 
                            }

                            /* if we're here, we're in a neighboring pixel that 
                             * exists; see if a dilation would increase the 
                             * current pixel */
                            if(copy[yp*width+xp] > array[y*width+x]){
                                array[y*width+x] = copy[yp*width+xp];
                            }
                        } 
                    }
                }
            }
            delete [] copy;
        }

		void process( FrameData &frameData ) {
            if(!ToFilter){
                return; 
            }

            /* If we don't have at least two images, then return and don't bother
             * looking for the depthmap */
            if(frameData.images.size() < 2){
                return; 
            } 
    
            SP_Image depthmap;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                SP_Image image = frameData.images[i];
                if(image->imageType == IMAGE_TYPE_DEPTH_MAP){
                    depthmap = image;
                }
            }

            /* Minimum density in cm^2 -> in m^2 */
            Float filter = Density*100*100;


            /* Divide the image into patches */

            int w = depthmap->width, h = depthmap->height;
            /* if the width or height can't be cleanly divided, we'll just add 
             * an extra patch to the image */
            int pw = w / PatchSize + (w % PatchSize == 0 ? 0 : 1),
                ph = h / PatchSize + (h % PatchSize == 0 ? 0 : 1);

            /* For counting the minimum depth in the patch */
            Float *minDepthMap = new Float[pw*ph];
            /* the number of pixels in a patch; this is not constant if the 
             * patch size doesn't divide the image width 
             */
            Float *sizeMap = new Float[pw*ph];

            for(int py = 0; py < ph; py++){
                for(int px = 0; px < pw; px++){
                    minDepthMap[py*pw+px] = 1e10;
                }
            }

            Float count = 0;
            for(int y = 0; y < h; y++){
                for(int x = 0; x < w; x++){
                    if(depthmap->getDepth(x,y) < 1e-4){
                        count += 1;
                    }
                    int px = x / PatchSize, py = y / PatchSize;
                    /* update the minimum depth */
                    minDepthMap[py*pw+px] = min(minDepthMap[py*pw+px], depthmap->getDepth(x,y));
                }
            }
        
            /* compute the size of the patch projected into the world at the 
             * relevant depth */
            for(int py = 0; py < ph; py++){
                for(int px = 0; px < pw; px++){
                    int x0 = px*PatchSize, x1 = min((px+1)*PatchSize, depthmap->width),
                        y0 = py*PatchSize, y1 = min((py+1)*PatchSize, depthmap->width);
                    /* Get the area by projecting the patch into the world at the 
                     * minimum depth */
                    sizeMap[py*pw+px] = getArea(depthmap->intrinsicLinearCalibration, 
                                                x0, y0, x1, y1, 
                                                minDepthMap[py*pw+px]);
                }
            }

            if(ToFilter == 1){
                /* For counting the number of features in each patch; this is
                 * also the density map */
                Float *countMap = new Float[pw*ph];
                for(int i = 0; i < pw*ph; i++){
                    countMap[i] = 0.0;
                }
                
                vector< FrameData::DetectedFeature > &corresp = frameData.detectedFeatures["SIFT"];
                
                for( int i=0; i<(int)corresp.size(); i++)  {
                    Pt<2> location = corresp[i].coord2D;
                    int px = ((int) location[0]) / PatchSize, py = ((int) location[1]) / PatchSize;
                    countMap[py*pw+px] += 1.0 / sizeMap[py*pw+px];
                }

                dilate(countMap, pw, ph);

                vector< FrameData::DetectedFeature > newCorresp;

                for( int i=0; i<(int)corresp.size(); i++)  {
                    Pt<2> location = corresp[i].coord2D;
                    int px = ((int) location[0]) / PatchSize, py = ((int) location[1]) / PatchSize;
                    if(countMap[py*pw+px] > filter){
                        newCorresp.push_back(corresp[i]);
                    }
                    /*
                    else {
                        
                        cerr << "FREJECT:" << location[0] << "," << location[1] << endl;
                    }
                    */
                }
                frameData.detectedFeatures["SIFT"] = newCorresp;
                delete [] countMap;
            } else if(ToFilter == 2){

                #pragma omp parallel for
                for(int modelNum = 0; modelNum < (int) frameData.matches.size(); modelNum++){
                    Float *countMap = new Float[pw*ph];
                    for(int i = 0; i < pw*ph; i++){
                        countMap[i] = 0.0;
                    }
                    vector<FrameData::Match> &matches = frameData.matches[modelNum]; 
                    for(int i = 0; i < (int) matches.size(); i++){
                        Pt<2> location = matches[i].coord2D;
                        int px = ((int) location[0]) / PatchSize, py = ((int) location[1]) / PatchSize;
                        countMap[py*pw+px] += 1.0 / sizeMap[py*pw+px];
                    }

                    dilate(countMap, pw, ph);

                    vector<FrameData::Match> newMatches;
                    for(int i = 0; i < (int) matches.size(); i++){
                        Pt<2> location = matches[i].coord2D;
                        int px = ((int) location[0]) / PatchSize, py = ((int) location[1]) / PatchSize;
                        if(countMap[py*pw+px] > filter){
                            newMatches.push_back(matches[i]);
                        }
                        /*
                        else {
                            #pragma omp critical 
                            {
                                cerr << "MREJECT:" << modelNum << ":" << location[0] << "," << location[1] << endl;
                            }
                        }
                        */
                    }
                    frameData.matches[modelNum] = newMatches;
                    delete [] countMap;
                }
            }
            delete [] sizeMap;
            delete [] minDepthMap;
        }
    };
}

