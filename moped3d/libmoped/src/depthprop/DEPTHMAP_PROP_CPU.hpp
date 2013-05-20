/*
 * David Fouhey
 * Depthmap propagation component
 */

#pragma once

#include <math.h>


/*
 * Depthmap format:
 * 
 * The depthmap is represented by an Image with width*height pixels. Each pixel
 * is formed of 4 Floats (i.e. the type set by libmoped):
 *
 * x y z norm 
 *
 * if norm is positive, it is ||[x y z]||; 
 * if norm is negative, then the depth is unresolved at that pixel.
 *
 * This format is chosen since it makes no sense to discard the xyz given by 
 * the kinect, and so x, y, and z need to be present. At the same time, we need
 * to be able to signal where the kinect could not resolve the data. There are 
 * no restrictions on the sign of x,y,z, and NaNs do not always function 
 * properly with an optimized compiler. We then store the norm, which must be
 * positive, so we can signal unresolved pixels, and so we don't have to 
 * calculate it repeatedly. 
 */

namespace MopedNS {

    /* Doing type-punning without a union seems to be frowned upon by gcc */
    union DepthmapFloatBuffer{
        char data[sizeof(Float)];
        Float f;
    };

	class DEPTHMAP_PROP_CPU: public MopedAlg {
	    public:

        DEPTHMAP_PROP_CPU(){ }
		
		void getConfig( map<string,string> &config ) const {

		}
			
		void setConfig( map<string,string> &config ) {
			
		}

		void process( FrameData &frameData ) {

            /* If we don't have at least two images, then return and don't bother
             * looking for the depthmap */
            if(frameData.images.size() < 2){
                return; 
            } 
    
            SP_Image grayImage, depthmap, distanceMap;
            int grayIdx = -1, depthIdx  = -1;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                SP_Image image = frameData.images[i];
                if(image->imageType == IMAGE_TYPE_GRAY_IMAGE){
                    grayImage = image;
                    grayIdx = i;
                }
                else if(image->imageType == IMAGE_TYPE_DEPTH_MAP){
                    depthmap = image;
                    depthIdx = i;
                }
            }

            /* if we don't have both an intensity and depth image, return */
            if(!((grayIdx != -1) && (depthIdx != -1))){
                return;
            }

            /* if the images aren't the same dimensions, return */
            if( (grayImage->width != depthmap->width) || 
                (grayImage->height != depthmap->height) ){
                return;
            }

            /* search for a depthmap fill distance image */
            int fillDistanceIdx = -1;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                SP_Image image = frameData.images[i];
                if(image->imageType == IMAGE_TYPE_PROB_MAP){
                    if(image->name == depthmap->name+".distance"){
                        fillDistanceIdx = i;
                        distanceMap = image;
                        break; 
                    }
                }
            }

            /* resolve all the matches */
			#pragma omp parallel for
            for(int modelNum = 0; modelNum < (int) frameData.matches.size(); modelNum++){
                for(int i = 0; i < (int) frameData.matches[modelNum].size(); i++){
                    Pt<2> location = frameData.matches[modelNum][i].coord2D;
                    int ix = (int) location[0], iy = (int) location[1];
                    struct depthInformation &depthInfo = frameData.matches[modelNum][i].depthData;
                    copyDepthInformation(depthmap, ix, iy, depthInfo);
                    /* fill in the distance if we have it, and set it to -1 if not */
                    if(fillDistanceIdx != -1){
                        depthInfo.fillDistance = distanceMap->getProb(ix, iy);
                    } else {
                        depthInfo.fillDistance = -1;
                    }
                } 
            }
        }

        /* 
         * \brief Get the depth at a given location in the depthmap 
         */
        void copyDepthInformation(SP_Image depthmap, int x, int y, struct depthInformation &depthInfo){
            union DepthmapFloatBuffer *buffer;
            /* Perhaps this should be bilinear interpolation, but that seems like 
             * relying on data that's not there given the uncertainty in our depthmaps
             */
            int offset = (y*depthmap->width+x)*sizeof(Float)*4; 
            /* get the location of the depth information for the picture */
            buffer = (union DepthmapFloatBuffer *) (((char *) &depthmap->data[0]) + offset);
            for(int i = 0; i < 3; i++){
                depthInfo.coord3D[i] = (buffer+i)->f;
            }
            depthInfo.depth = (buffer+2)->f;
            depthInfo.depthValid = ((buffer+3)->f >= 0) ? 1 : 0; 
        }

    };
}

