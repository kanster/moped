/*
 * David Fouhey
 * Depthfilter
 */

#pragma once

#include <math.h>

#define DEBUG 0


namespace MopedNS {

	class DEPTHFILTER_IGNORE_CPU: public MopedAlg {

	    public:

        DEPTHFILTER_IGNORE_CPU(){ }
		
		void getConfig( map<string,string> &config ) const {
		}
			
		void setConfig( map<string,string> &config ) {
   		}


		void process( FrameData &frameData ) {
            return; 
        }
    };
}

