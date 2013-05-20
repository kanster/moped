#pragma once
#include <queue>

namespace MopedNS {

	class DEPTH_NO_FILL_CPU :public MopedAlg {

        Float FillDepth;
        Float FillDistance;

	public:
        /* Doing type-punning without a union seems to be frowned upon by gcc */
        union DepthmapFloatBuffer{
            char data[sizeof(Float)];
            Float f;
        };

		DEPTH_NO_FILL_CPU(Float FillDepth, Float FillDistance) : FillDepth(FillDepth), FillDistance(FillDistance) { }

		void getConfig( map<string,string> &config ) const {
			GET_CONFIG( FillDepth );
			GET_CONFIG( FillDistance );
		}
		
		void setConfig( map<string,string> &config ) {
			SET_CONFIG( FillDepth );
			SET_CONFIG( FillDistance );
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
                    /* load z */
                    Float z = (buffer+2)->f;
                    /* update the distance */
                    (buffer+3)->f = sqrt(x*x+y*y+z*z);
                }
            }
        }

        SP_Image fillDummyData(SP_Image depthmap){
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
            SP_Image distanceMap(new Image);
            distanceMap->imageType = IMAGE_TYPE_PROB_MAP;
            distanceMap->width = w; distanceMap->height = h;
            distanceMap->data.resize(w*h*sizeof(Float));
            distanceMap->name = depthmap->name+".distance";
            for(int y = 0; y < h; y++){
                for(int x = 0; x < w; x++){
                    int i = y*w+x;
                    if(valid[i]){
                        distanceMap->setProb(x,y,0);
                    } else {
                        distanceMap->setProb(x,y,FillDistance);
                        depthmap->setDepth(x,y,FillDepth);
                    }
                }
            }
            normalizeDepthmap(depthmap, valid); 
            delete [] valid;
            return distanceMap;
        }

		
		void process( FrameData &frameData ) {
            for(int i = 0; i < (int) frameData.images.size(); i++){
                SP_Image image = frameData.images[i];
                if(image->imageType == IMAGE_TYPE_DEPTH_MAP){ 
                    SP_Image distanceMap = fillDummyData(image);
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


