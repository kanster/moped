#pragma once
#include <queue>

namespace MopedNS {

	class DEPTH_FILL_EXACT_CPU :public MopedAlg {

        int scaleFactor;
        bool doBilinearInterpolation;

	public:
        /* Doing type-punning without a union seems to be frowned upon by gcc */
        union DepthmapFloatBuffer{
            char data[sizeof(Float)];
            Float f;
        };

		DEPTH_FILL_EXACT_CPU(int scaleFactor, bool doBilinearInterpolation) : scaleFactor(scaleFactor), doBilinearInterpolation(doBilinearInterpolation) { }

		void getConfig( map<string,string> &config ) const {
			GET_CONFIG( scaleFactor );
		}
		
		void setConfig( map<string,string> &config ) {
			SET_CONFIG( scaleFactor );
		}

        /*
         * \brief Return true iff all the 8-neighbors of (x,y) are valid according to
         * dataValid.
         */
        inline bool all8NeighborsValid(int x, int y, bool *valid, SP_Image depthmap){
            for(int dx = -1; dx <= 1; dx++){
                for(int dy = -1; dy <= 1; dy++){
                    if((dx == 0) && (dy == 0)){
                        continue; 
                    }
                    if(!dataValid(x+dx, y+dy, valid, depthmap)){
                        return false;
                    }
                }
            }
            return true;
        }

        /*
         * \brief Return false iff (x,y) is inside the given depthmap and (x,y) is invalid.
         *
         * If (x,y) is within the image defined by depthmap, check inside valid, which is 
         * an indicator of which pixels in depthmap refer to valid depths, and return false
         * iff (x,y) refers to an invalid pixel.
         */
        inline bool dataValid(int x, int y, bool *valid, SP_Image depthmap){
            /* returns false only when the x,y refers to data inside the depthmap
             * that is invalid */
            return  (y < 0) || (y >= depthmap->height) || \
                    (x < 0) || (x >= depthmap->width) || \
                    valid[y*depthmap->width+x];
        }

        /*
         * \brief Do nearest neighbor interpolation of two images.
         *
         * Downscale is a constant multiple size of upscale, valid indicates which pixels of upscale are valid,
         * and getVal and setVal get and set pixel (x,y) respectively 
         */
        void NNInterp(SP_Image downscale, SP_Image upscale, bool *valid, Float (*getVal)(SP_Image, int, int), void (*setVal)(SP_Image, int, int, Float)){
            /* Do nearest neighbor interpolation */
            int w = upscale->width, h = upscale->height;
            int scaleFactor = upscale->width / downscale->width;
            int ly = 0, lx;
            for(int uy = 0; uy < h; uy++){
                lx = 0;
                for(int ux = 0; ux < w; ux++){
                    if((ux != 0) && (ux % scaleFactor == 0)){
                        lx++;
                    }
                    if(valid[uy*w+ux]){
                        continue;
                    }
                    setVal(upscale, ux, uy, getVal(downscale, lx, ly));
                }
                if((uy != 0) && (uy % scaleFactor == 0)){
                    ly++;
                }
            }
        }

        /*
         * \brief Do bilinear interpolation of two images
         *
         * Downscale is a constant multiple size of upscale, valid indicates which pixels of upscale are valid,
         * and getVal and setVal get and set pixel (x,y) respectively 
         */
        void bilinearInterp(SP_Image downscale, SP_Image upscale, bool *valid, Float (*getVal)(SP_Image, int, int), void (*setVal)(SP_Image, int, int, Float)){
            /* Do bilinear interpolation */
            int w = upscale->width, h = upscale->height;
            int scaleFactor = upscale->width / downscale->width;
            /* rather than compute the location of the upper-left corners, etc. at each
             * invocation in the inner loop,  we keep track of the left corners, and their
             * ``influence'' on the value that we're currently at */
            double delta = 1.0 / scaleFactor;
            double upInfluence = 0, leftInfluence = 0;
            /* lower scale locations; set to -1 for easy updating of the location inside
             * the loop*/
            int ly = -1, lx;
            for(int uy = 0; uy < h; uy++){
                upInfluence -= delta;
                /* when we're directly above a lower scale known value, that value alone
                 * determines the interpolated value in that direction */
                if(uy % scaleFactor == 0){
                    ly++;
                    upInfluence = 1;
                }
                lx = -1;
                for(int ux = 0; ux < w; ux++){
                    leftInfluence -= delta;
                    if(ux % scaleFactor == 0){
                        lx++;
                        leftInfluence = 1;
                    }
                    /* don't interpolate if we knew the value to start with */
                    if(valid[uy*w+ux]){
                        continue;
                    }
                    /* The four corners in the downsampled image */
                    int x0 = lx, y0 = ly, x1 = lx+1, y1 = ly+1;

                    /* compute the influences weight(Xcorner)(Ycorner) */
                    Float w00 = leftInfluence*upInfluence,
                          w01 = leftInfluence*(1-upInfluence),
                          w10 = (1-leftInfluence)*upInfluence,
                          w11 = (1-leftInfluence)*(1-upInfluence);

                    /* If the scale factor doesn't divide evenly, change we're we're accessing 
                     * in the downscaled map, and shift the weights accordingly */
                    if(x1 == downscale->width){
                        /* shift all the weight to the left (i.e valid) pixels, and set x1 = x0 */
                        w00 += w01; w01 = 0;
                        w01 += w11; w11 = 0;
                        x1 = x0;
                    }
                    if(y1 == downscale->height){
                        /* shift all the weight to the up pixels, and set y1 = y0 */
                        w00 += w10; w10 = 0;
                        w10 += w11; w11 = 0;
                        y1 = y0;
                    }
                    /*
                    cerr << "Interp img " << upscale->width << "x" << upscale->height << " <= " << downscale->width << "x" << downscale->height << ";";
                    cerr << "Accessing @ (" << x0 << "," << y0 << ") , (" << x1 << "," << y1 << ") w = [" << w00 << " " << w01 << " " << w10 << w11 << "]";
                    cerr << "Writing to (" << ux << ", " << uy << ")" << endl;
                    */
                    /* interpolate */
                    Float interpValue = w00*getVal(downscale, x0, y0) +  \
                                        w01*getVal(downscale, x0, y1) + \
                                        w10*getVal(downscale, x1, y0) + \
                                        w11*getVal(downscale, x1, y1);
                    /* update the depthmap */
                    setVal(upscale, ux, uy, interpValue);
                }
            }
        }

        /*
         * \brief Fill in a depthmap with the nearest valid neighbor.
         *
         * Returns a distance matrix, where all values are multiplied by distanceDilate
         */
        SP_Image fillIn(SP_Image depthmap, Float distanceDilate){
            int pixelCount = depthmap->width*depthmap->height;
            bool *valid = new bool[pixelCount];

            SP_Image bestDistance(new Image);
            bestDistance->imageType = IMAGE_TYPE_PROB_MAP;
            bestDistance->width = depthmap->width; bestDistance->height = depthmap->height;
            bestDistance->data.resize(bestDistance->width*bestDistance->height*sizeof(Float));

            union DepthmapFloatBuffer *buffer;
            int validCount = 0;
            /* set whether the points are valid */
            for(int i = 0; i < pixelCount; i++){
                buffer = (union DepthmapFloatBuffer *) (((char *) &depthmap->data[0]) + i*4*sizeof(Float)); 
                valid[i] = (buffer+2)->f >= 0 ? true : false;
                if(valid[i]){
                    validCount++;
                }
            }

            /* A queue of pairs giving:
             *      (startX, startY) and (currentX, currentY)
             */
            queue< pair< pair<int, int> , pair<int, int> > >  growQueue;
            for(int y = 0; y < depthmap->height; y++){
                for(int x = 0; x < depthmap->width; x++){
                    if(dataValid(x, y, valid, depthmap) && (!all8NeighborsValid(x, y, valid, depthmap))){
                        growQueue.push(make_pair(make_pair(x, y),make_pair(x, y)));
                    }
                    setProb(bestDistance, x,  y, valid[y*depthmap->width+x] ? 0 : 1e30);
                }
            }
            int iters = 0;
            while(!growQueue.empty()){
                iters++;
                /* unpack the element and check the adjacent elements */
                pair< pair<int, int> , pair<int, int> > elt = growQueue.front();
                growQueue.pop(); 
                int x0 = elt.first.first, y0 = elt.first.second,
                    x1 = elt.second.first, y1 = elt.second.second;
               
                /* check if any neighbors can be updated;
                 * We only need to check these neighbors since basically we're computing the
                 * 2D L2 norm voronoi diagrams, and each region is connected and convex (i.e., 
                 * if we say (x0,y0) is the nearest known value to (x,y), it is also for a 
                 * convex region in between them).
                 */
                for(int dy = -1; dy <= 1; dy++){
                    for(int dx = -1; dx <= 1; dx++){
                        int xp = x1+dx, yp = y1+dy;
                        if(dataValid(xp, yp, valid, depthmap)){
                            continue; 
                        }
                        /* if we can beat the current best distance, beat it and continue */
                        Float currentDistance = sqrt((Float) ((xp - x0)*(xp - x0) + (yp - y0)*(yp - y0)))*distanceDilate;
                        Float currentBestDistance = getProb(bestDistance, xp, yp);
                        if(currentDistance < currentBestDistance){
                            setProb(bestDistance, xp, yp, currentDistance);
                            setDepth(depthmap, xp, yp, getDepth(depthmap, x0, y0));
                            growQueue.push( make_pair( make_pair(x0, y0), make_pair(xp, yp)));
                        }
                    }
                }
            }
 

            delete [] valid;
            return bestDistance;
        }

        /*
         * \brief Takes the filled in depthmap, and update the norms if they were filled in.
         *
         */
        void normalizeDepthmap(SP_Image depthmap, bool *valid){
            Pt<4> K = depthmap->intrinsicLinearCalibration;
            for(int v = 0; v < depthmap->height; v++){
                for(int u = 0; u < depthmap->width; u++){
                    if(valid[v*depthmap->width+u]){
                        continue;
                    }
                    union DepthmapFloatBuffer *buffer;
                    int offset = (v*depthmap->width+u)*sizeof(Float)*4;
                    buffer = (union DepthmapFloatBuffer *) (((char *) &depthmap->data[0])+offset);

                    /* Project into the world */
                    Float x = (u - K[2]) / K[0], y = (v - K[3]) / K[1];
                    /* update x and y.
                     *
                     * THESE HAVE TO BE MULTIPLIED BY THE DEPTH! OTHERWISE THIS TUGS THE VECTOR
                     * OUT OR IN, CHANGING WHAT PIXEL IT GOES THROUGH. */
                    (buffer+0)->f = x*(buffer+2)->f; 
                    (buffer+1)->f = y*(buffer+2)->f;
                    x = (buffer+0)->f; y = (buffer+1)->f;
                    /* load z */
                    Float z = (buffer+2)->f;
                    /* update the distance */
                    (buffer+3)->f = sqrt(x*x+y*y+z*z);
                }
            }
        }

        /*
         *\brief Fill in depthmap holes with their nearest-valid neighbor, using a scaled approximation. 
         *
         * Downscales the depthmap, fills in the holes with the nearest valid neighbors, and upscales the 
         * filled-in depthmap.
         */
        SP_Image fillInScaled(SP_Image depthmap, int scaleFactor){
            int w = depthmap->width, h = depthmap->height;
            /* generate a validity map */
            bool *valid = new bool[w*h];
            int validCount = 0;
            for(int y = 0; y < h; y++){
                for(int x = 0; x < w; x++){
                    Float depth = getDepth(depthmap, x, y);
                    valid[y*w+x] = (depth >= 0);
                    if(valid[y*w+x]){
                        validCount++;
                    }
                }
            }

            if(scaleFactor == -1){
                Float invalidRatio = ((Float) w*h - validCount)  / (w*h);
                if(invalidRatio < 0.1){
                    scaleFactor = 1;
                } else if(invalidRatio < 0.2){
                    scaleFactor = 2;
                } else if(invalidRatio < 0.4){
                    scaleFactor = 4;
                } else if(invalidRatio < 0.6){
                    scaleFactor = 8;
                } else {
                    scaleFactor = 16;
                }
            }



            /* Initialize the distance map */
            SP_Image upscaledDistance(new Image);
            upscaledDistance->imageType = IMAGE_TYPE_PROB_MAP;
            upscaledDistance->width = w; upscaledDistance->height = h;
            upscaledDistance->data.resize(w*h*sizeof(Float));
            upscaledDistance->name = depthmap->name+".distance";
            for(int i = 0; i < w*h; i++){
                setProb(upscaledDistance, i % w, i / w, 0.0);
            }

            /* downscale width / height */
            SP_Image downscaledDepth(new Image);
            int dw = w / scaleFactor, dh = h / scaleFactor;
            downscaledDepth->width = dw; downscaledDepth->height = dh;
            downscaledDepth->data.resize(dw*dh*sizeof(Float)*4);
            /* Nearest neighbor downsample: properly interpolating would
             * be a mess and probably not worth it; specifically, what happens when 
             * one of the adjacent pixels is invalid */
            for(int y = 0; y < dh; y++){
                for(int x = 0; x < dw; x++){
                    int uy = y*scaleFactor, ux = x*scaleFactor;
                    setDepth(downscaledDepth, x, y, getDepth(depthmap, ux, uy));

                }
            }
            SP_Image fillDist = fillIn(downscaledDepth, scaleFactor);

            /* upscale the depthmap using bilinear interpolation if we filled
             * in those values */
            if(scaleFactor != 1){
                if(doBilinearInterpolation){
                    bilinearInterp(downscaledDepth, depthmap, valid, MopedNS::DEPTH_FILL_EXACT_CPU::getDepth, MopedNS::DEPTH_FILL_EXACT_CPU::setDepth);
                    bilinearInterp(fillDist, upscaledDistance, valid, MopedNS::DEPTH_FILL_EXACT_CPU::getProb, MopedNS::DEPTH_FILL_EXACT_CPU::setProb);
                } else {
                    NNInterp(downscaledDepth, depthmap, valid, MopedNS::DEPTH_FILL_EXACT_CPU::getDepth, MopedNS::DEPTH_FILL_EXACT_CPU::setDepth);
                    NNInterp(fillDist, upscaledDistance, valid, MopedNS::DEPTH_FILL_EXACT_CPU::getProb, MopedNS::DEPTH_FILL_EXACT_CPU::setProb);
                }
            } else {
                depthmap = downscaledDepth;
                upscaledDistance = fillDist;
            }
            normalizeDepthmap(depthmap, valid); 


            delete [] valid;
            return upscaledDistance;
        }

        Float bound(Float val, Float minVal, Float maxVal){
            if(val < minVal){
                return minVal;
            } else if(val > maxVal){
                return maxVal;
            } 
            return val; 
        }

        IplImage *renderDepthmapJet(SP_Image image){
            int w = image->width, h = image->height;
            IplImage *render = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 3);
            Float minVal = FLT_MAX, maxVal = 0;
            for(int y = 0; y < h; y++){
                for(int x = 0; x < w; x++){
                    Float val = image->getDepth(x,y);
                    if(val < minVal){
                        minVal = val;
                    } 
                    if(val > maxVal){
                        maxVal = val;
                    }
                }
            }
            for(int y = 0; y < h; y++){
                for(int x = 0; x < w; x++){
                    Float nVal4 = 4*(image->getDepth(x,y) - minVal) / (maxVal - minVal);
                    unsigned char *pix = &CV_IMAGE_ELEM(render, unsigned char, y, x*3);
                    pix[0] = (unsigned char) bound(min(nVal4-1.5, 4.5-nVal4)*255,0,255);
                    pix[1] = (unsigned char) bound(min(nVal4-0.5, 3.5-nVal4)*255,0,255);
                    pix[2] = (unsigned char) bound(min(nVal4+0.5, 2.5-nVal4)*255,0,255);
                }
            }
            return render;
        }

        IplImage *renderDistanceMap(SP_Image distance, Float cauchyWeight){
            int w = distance->width, h = distance->height;
            IplImage *render = cvCreateImage(cvSize(distance->width, distance->height), IPL_DEPTH_8U, 1);
            for(int y = 0; y < h; y++){
                for(int x = 0; x < w; x++){
                    Float val = distance->getProb(x, y);
                    Float weight = 1.0 / (1.0 + ((val*val)/(cauchyWeight*cauchyWeight)));
                    unsigned char *pix = &CV_IMAGE_ELEM(render, unsigned char, y, x);
                    *pix = (unsigned char) bound(255*weight,0,255);
                }
            }
            return render; 
        }
		
		void process( FrameData &frameData ) {
            for(int i = 0; i < (int) frameData.images.size(); i++){
                SP_Image image = frameData.images[i];
                if(image->imageType == IMAGE_TYPE_DEPTH_MAP){ 
                    SP_Image distanceMap = fillInScaled(image, scaleFactor);
                    frameData.images.push_back(distanceMap);
                    /*
                    cvSaveImage("filledDepthmap.png", renderDepthmapJet(image));
                    cvSaveImage("distanceWeight.png", renderDistanceMap(distanceMap,75.0));
                    */
                }
            } 
        }
      
        /*
         * \brief Get a value at the given location in a probability map.
         */
        static inline Float getProb(SP_Image prob, int x, int y){
            int offset = (y*prob->width+x)*sizeof(Float); 
            /* get the location of the depth information for the picture */
            return ((union DepthmapFloatBuffer *) (((char *) &prob->data[0]) + offset))->f;
        }

        /*
         * \brief Set the value at the given location in the probability map.
         */
        static inline void setProb(SP_Image prob, int x, int y, Float val){
            int offset = (y*prob->width+x)*sizeof(Float); 
            /* get the location of the depth information for the picture */
            ((union DepthmapFloatBuffer *) (((char *) &prob->data[0]) + offset))->f = val;
        }

        /*
         * \brief Get the depth at a given location in a depthmap.
         */
        static inline Float getDepth(SP_Image depthmap, int x, int y){
            union DepthmapFloatBuffer *buffer;
            int offset = (y*depthmap->width+x)*sizeof(Float)*4; 
            /* get the location of the depth information for the picture */
            buffer = (union DepthmapFloatBuffer *) (((char *) &depthmap->data[0]) + offset);
            return (buffer+2)->f;
        }
       
        /*
         * \brief Set the depth at a given location in a depthmap
         */
        static inline void setDepth(SP_Image depthmap, int x, int y, Float depth){
            union DepthmapFloatBuffer *buffer;
            int offset = (y*depthmap->width+x)*sizeof(Float)*4;
            buffer = (union DepthmapFloatBuffer *) (((char *) &depthmap->data[0])+offset);
            (buffer+2)->f = depth; 
        }
    };
}


