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

	class CLUSTER_LINKAGE_CPU : public MopedAlg {
    
        Float Cutoff;
        int MinPts;
        int Use3DFilter;
        Float WeightGamma;
        Float Alpha;
        int LinkageType;
        Float Sigma2D;
        Float Sigma3D;
		
	public:

		CLUSTER_LINKAGE_CPU(Float Cutoff, 
                            int MinPts, 
                            int Use3DFilter, 
                            Float WeightGamma, 
                            Float Alpha, 
                            int LinkageType, 
                            Float Sigma2D, 
                            Float Sigma3D ) : 
            Cutoff(Cutoff), MinPts(MinPts), Use3DFilter(Use3DFilter), WeightGamma(WeightGamma), Alpha(Alpha), LinkageType(LinkageType),
            Sigma2D(Sigma2D), Sigma3D(Sigma3D) {
		}

		void getConfig( map<string,string> &config ) const {
		
			GET_CONFIG( Cutoff );
            GET_CONFIG( MinPts );
            GET_CONFIG( Use3DFilter );
            GET_CONFIG( WeightGamma );
            GET_CONFIG( Alpha );
            GET_CONFIG( LinkageType );

		}
		
		void setConfig( map<string,string> &config ) {
			
			SET_CONFIG( Cutoff );
            SET_CONFIG( MinPts );
            SET_CONFIG( Use3DFilter );
            SET_CONFIG( WeightGamma );
            SET_CONFIG( Alpha );
            SET_CONFIG( LinkageType );

		}

        void getAverageNNDistances(vector<FrameData::Match> &matches, Float &nn2D, Float &nn3D){
            /* get the average nearest neighbor distances in 2D and 3D */
            int N = (int) matches.size();
            /* get the number of pairs to evaluate on */
            nn2D = 0;
            nn3D = 0;
            for(int i = 0; i < N; i++){
                FrameData::Match mi = matches[i];
                Float nn2Di = DBL_MAX, nn3Di = DBL_MAX;
                for(int j = 0; j < N; j++){
                    if(i == j){
                        continue;
                    }
                    FrameData::Match mj = matches[j];
                    Float dist2D = mi.coord2D.euclDist(mj.coord2D);
                    Float dist3D = mi.coord3D.euclDist(mj.coord3D);
                    if(nn2Di > dist2D){
                        nn2Di = dist2D;
                    }
                    if(nn3Di > dist3D){
                        nn3Di = dist3D;
                    }
                }
                nn2D += nn2Di; nn3D += nn3Di;
            }
            nn2D /= N; nn3D /= N;
        }

        SP_Image getSimilarityMatrix(int N){
            SP_Image K = SP_Image(new Image(IMAGE_TYPE_PROB_MAP));
            K->width = K->height = N;
            K->data.resize(N*N*sizeof(Float));
            return K;
        }

        /* \brief Get a Gaussian kernel similarity matrix. */
        SP_Image getGaussK(vector <FrameData::Match> &matches, Float sigma, bool do3D){
            int N = (int) matches.size();
            SP_Image K = getSimilarityMatrix(N);
            /* used in the evaluation; only evaluate it once */
            Float twoSigmaSq = 2*sigma*sigma;
            for(int i = 0; i < N; i++){
                FrameData::Match mi = matches[i];
                for(int j = i; j < N; j++){
                    /* this branch should be easy to predict */
                    Float distance = do3D ? mi.depthData.coord3D.sqEuclDist(matches[j].depthData.coord3D) : 
                                            mi.coord2D.sqEuclDist(matches[j].coord2D);
                    Float val = exp(-1*distance/twoSigmaSq);
                    K->setProb(i, j, val); K->setProb(j, i, val);
                }
            }
            return K;
        }

        SP_Image get3DFilterK(vector <FrameData::Match> &matches){
            int N = (int) matches.size();
            SP_Image K = getSimilarityMatrix(N);
            /* set to whatever error rate you want to have score 0.61 */
            Float sigma = 0.1;
            Float twoSigmaSq = 2*sigma*sigma;
            /* get the 3D real-world / model distance disparity */
            for(int i = 0; i < N; i++){
                FrameData::Match mi = matches[i];
                K->setProb(i,i, 1.0);
                for(int j = i+1; j < N; j++){
                    FrameData::Match mj = matches[j];
                    Float distanceModel = mi.coord3D.euclDist(mj.coord3D);
                    Float distanceRealWorld = mi.depthData.coord3D.euclDist(mj.depthData.coord3D);
                    /* get the error of the distance recorded in the 3D world */
                    Float distanceError = fabs(distanceModel - distanceRealWorld) / distanceModel;
                    /* guassian kernel */
                    Float val = exp((-1*distanceError*distanceError)/twoSigmaSq);
                    K->setProb(i,j, val); K->setProb(j, i, val); 
                }
            }
            return K; 
        }


        void bresenhamIterate(vector<pair<int, int> > &coords, pair<int,int> p, pair<int,int> q, int numberSamples){

            /* Implementation taken from Wikipedia */
            int x0 = p.first, y0 = p.second, x1 = q.first, y1 = q.second, t;
            bool steep = abs(y1 - y0) > abs(x1 - x0);
            if(steep){
                /* swap(x0,y0), swap(x1,y1) */
                t = x0; x0 = y0; y0 = t;
                t = x1; x1 = y1; y1 = t; 
            }
            if(x0 > x1){
                /* swap(x0, x1), swap(y0,y1) */
                t = x0; x0 = x1; x1 = t;
                t = y0; y0 = y1; y1 = t;
            }
            Float deltaX = (Float) x1 - x0, deltaY = fabs((Float) y1 - y0);

            
            int yStep = (y0 < y1) ? 1 : -1;

            int perStep = (int) (x1 - x0) / numberSamples;
            if(perStep < 1){
                perStep = 1;
            }

            /* the total accumulated error, and how much we accumulate per step in X */
            Float error = 0.0, deltaError = ((Float) deltaY) / deltaX;
            Float intPart;
            /* what way are we going in terms of y? */
            int y = y0;
            for(int x = x0; x <= x1;){
                /* push back every perStep samples */
                if(steep){
                    coords.push_back(make_pair(y, x)); 
                } else {
                    coords.push_back(make_pair(x, y));
                }
               
                /* advance the x and error by the increment */
                x += perStep;
                error += deltaError*perStep*yStep;
                error = modf(error, &intPart); 
                y += intPart;
            }
        }

        pair<int,int> saturatePair(int x, int y, SP_Image depthImage){
            x = (x < 0) ? 0 : x;
            x = (x >= depthImage->width) ? depthImage->width - 1 : x;
            y = (y < 0) ? 0 : y;
            y = (y >= depthImage->height) ? depthImage->height - 1 : y;
            return make_pair(x,y); 
        }

        SP_Image getDiscontinuityMatrix(vector <FrameData::Match> &matches, SP_Image depthImage){

            int N = matches.size();
            SP_Image K = getSimilarityMatrix(N);

            /* used for enumerating the points on a path from one point to the next */
            vector<pair<int, int> > depthCoords; 
            
            /* -2 * sigma^2 if sigma = M_PI / 128 */
            Float discontinuityDiv = -2*(M_PI/128)*(M_PI/128);

            for(int i = 0; i < N; i++){
                FrameData::Match mi = matches[i];
                pair<int, int> li = saturatePair((int) mi.coord2D[0], (int) mi.coord2D[1], depthImage);
                for(int j = i; j < N; j++){
                    FrameData::Match mj = matches[j];
                    pair<int, int> lj = saturatePair((int) mj.coord2D[0], (int) mj.coord2D[1], depthImage);
                    depthCoords.clear();
                    /* get the points on the way from li to lj */
                    bresenhamIterate(depthCoords, li, lj, 20);
                   
                    /* get the depth at the start and the end */
                    Float depthStart = depthImage->getDepth(li.first, li.second);
                    Float depthEnd = depthImage->getDepth(lj.first, lj.second);

                    /* figure out the distance between the two points in the image plane */
                    int xDiff = (li.first  - lj.first), yDiff = (li.second - lj.second);
                    Float imagePlaneDist = sqrt( (float) (xDiff*xDiff+yDiff*yDiff));
                    
                    /* get the angle indicating the depth */
                    Float directAngle = atan2(depthEnd - depthStart, imagePlaneDist);

                    Float maxAngleDiff = -1;

                    for(int pix = 0; pix < (int) depthCoords.size() - 1; pix++){ 
                        pair<int, int> coord1 = depthCoords[pix], coord2 = depthCoords[pix+1];
                        Float depth1 = depthImage->getDepth(coord1.first, coord1.second);
                        Float depth2 = depthImage->getDepth(coord2.first, coord2.second);
                        Float dx = coord1.first - coord2.first, dy = coord1.second - coord2.second;
                        Float pixDistance = sqrt(dx*dx+dy*dy);
                        /* figure out the angle at this particular location */
                        Float pixAngle = atan2(depth2 - depth1, pixDistance);
                        Float angleDiff = fabs(directAngle - pixAngle);
                        /* update the maximum angle difference */
                        if(angleDiff > maxAngleDiff){
                            maxAngleDiff = angleDiff;
                        }
                    }
                    /* set the discontinuity to the exp(-x^2/2sigma^2) where x = maximum angle difference */
                
                    Float val = exp(maxAngleDiff*maxAngleDiff/discontinuityDiv);
                    K->setProb(i, j, val); K->setProb(j, i, val);
                }
            }

            return K; 
        }

        void getProduct(SP_Image X, SP_Image Y, SP_Image Z){
            for(int y = 0; y < X->height; y++){
                for(int x = 0; x < X->width; x++){
                    Z->setProb(x,y,X->getProb(x,y)*Y->getProb(x,y));
                }
            }
        }

        void getSum(SP_Image X, SP_Image Y, SP_Image Z){
            for(int y = 0; y < X->height; y++){
                for(int x = 0; x < X->width; x++){
                    Z->setProb(x,y,X->getProb(x,y)+Y->getProb(x,y));
                }
            }
        }

        /* Normalize a similarity matrix */
        void normalizeSimilarityMatrix( SP_Image K){
            int N = K->width;
            Float maxValue = -1;
            for(int i = 0; i < N; i++){
                for(int j = 0; j < N; j++){
                    Float value = K->getProb(i,j);
                    if(value > maxValue){
                        maxValue = value; 
                    }
                }
            }
            /* divide through by the maximum value */
            for(int i = 0; i < N; i++){
                for(int j = 0; j < N; j++){
                    K->setProb(i,j, K->getProb(i,j) / maxValue);
                }
            } 
        }


        SP_Image adaptiveWeightSum( vector<FrameData::Match> &matches, SP_Image distanceMap, 
                                    SP_Image K2D, SP_Image K3D, 
                                    Float alpha, Float gamma){

            int N = matches.size();
            Float gammaSq = gamma*gamma;
            vector<Float> weights;
            /* get the individual weights; the weights per element of the similarity matrix
             * are the product of these */
            for(int i = 0; i < N; i++){
                FrameData::Match m = matches[i];
                Float d = distanceMap->getProb((int) m.coord2D[0], (int) m.coord2D[1]);
                /* cauchy weighting of the distance */
                weights.push_back(1.0 / (1 + (d*d/gammaSq)));
            }

            /* calculate 1 - alpha */
            Float alphaBar = 1.0 - alpha;

            SP_Image K = SP_Image(new Image(IMAGE_TYPE_PROB_MAP));
            K->width = K->height = N;
            K->data.resize(N*N*sizeof(Float));
            Float averageWeight3D = 0.0;
            for(int i = 0; i < N; i++){
                for(int j = i; j < N; j++){
                    /* get the elements of interest */
                    Float K2De = K2D->getProb(i,j);
                    Float K3De = K3D->getProb(i,j);
                    /* get the joint weight of the two elements; this ranges from 0 to 1 */
                    Float jointWeight = weights[i]*weights[j];
                    /* the 2d information gets alpha automatically, plus whatever
                     * of what 3D gets (i.e., alphaBar) that is invalidated by
                     * the weight */
                    Float w2D = (alpha + alphaBar*(1.0 - jointWeight)),
                          w3D = alphaBar*jointWeight;
                    Float val = w2D*K2De+w3D*K3De;
                    K->setProb(i,j, val); K->setProb(j, i, val);
                    averageWeight3D += w3D;
                }
            }
            return K;
        }

        /* \brief Compute average linkage.
         * Given a similarity matrix K and lists of points cluster1 and cluster2,
         * return the average linkage of cluster1 and cluster2.
         * */
        Float averageLinkage(SP_Image K, list<int> cluster1, list<int> cluster2){
            Float s = 0.0;
            int N = cluster1.size(), M = cluster2.size();
            foreach(i, cluster1){
                foreach(j, cluster2){
                    s += K->getProb(i,j);
                }
            }
            return s / (N*M);
        }

        /* \brief Compute maximum linkage.
         * Given a similarity matrix K and lists of points cluster1 and cluster2,
         * return the maximum linkage of cluster1 and cluster2
         */
        Float maximumLinkage(SP_Image K, list<int> cluster1, list<int> cluster2){
            Float maxLink = -1;
            foreach(i, cluster1){
                foreach(j, cluster2){
                    if(K->getProb(i,j) > maxLink){
                        maxLink = K->getProb(i,j);
                    }
                }
            }
            return maxLink;
        }


        /* \brief Compute minimum linkage.
         * Given a similarity matrix K and lists of points cluster1 and cluster2,
         * return the minimum linkage of cluster1 and cluster2
         */
        Float minimumLinkage(SP_Image K, list<int> cluster1, list<int> cluster2){
            Float minLink = 1e20;
            foreach(i, cluster1){
                foreach(j, cluster2){
                    if(K->getProb(i,j) < minLink){
                        minLink = K->getProb(i,j);
                    }
                }
            }
            return minLink;
        }

        /* \brief Perform agglomerative clustering.
         * Given a similarity matrix K, perform agglomerative hierarchical clustering
         */
        vector<FrameData::Cluster> hierarchicalCluster(SP_Image K){

   
            int N = K->width;
            vector<FrameData::Cluster > clusters;
            vector<bool> valid;
            /* build the collection of clusters */
            for(int i = 0; i < N; i++){
                list<int> cluster;
                cluster.push_back(i);
                clusters.push_back(cluster);
                valid.push_back(true);
            }
           
            /* allocate a cluster similarity matrix and distance validity matrix */
            Float *distances = new Float[N*N];
            list<int> validIndices;

            /* compute the similarities only once */
            for(int i = 0; i < N; i++){
                validIndices.push_back(i);
                for(int j = i; j < N; j++){
                    distances[j*N+i] = distances[i*N+j] = K->getProb(i,j);
                }
            }

            /* start out not removing any values */
            int removeValue = -1;

            while(true){
                /* find the pair with maximum similarity distance */
                Float maxSimilarity = -1;
                pair<int,int> maxPair;

                list<int>::iterator index1_it, index2_it;
                int index1, index2;
                for(index1_it = validIndices.begin(); index1_it != validIndices.end(); index1_it++){
                    index1 = *index1_it;
                    if(index1 == removeValue){
                        index1_it = validIndices.erase(index1_it);
                        continue;
                    }
                    index2_it = index1_it;
                    index2_it++;
                    for(; index2_it != validIndices.end(); index2_it++){
                        index2 = *index2_it;
                        if(distances[index1*N+index2] > maxSimilarity){
                            maxSimilarity = distances[index1*N+index2];
                            maxPair = make_pair(index1,index2);
                        }
                    }
                }
                /*
                for(int i = 0; i < N; i++){
                    if(!valid[i]){
                        continue;
                    }
                    for(int j = i+1; j < N; j++){
                        if(!valid[j]){
                            continue;
                        }
                        if(distances[i*N+j] > maxSimilarity){
                            maxSimilarity = distances[i*N+j];
                            maxPair = make_pair(i, j);
                        }
                    }
                }
                */
                if(maxSimilarity < Cutoff){
                    break; 
                }

                /* For average linkage, we'lll need the size of the clusters */
                int SToUpdate = clusters[maxPair.first].size(),
                    SRemoveValue = clusters[maxPair.second].size();

                /* now that we've found the minimum pair, merge them */
                while(clusters[maxPair.second].size() != 0){
                    clusters[maxPair.first].push_back(clusters[maxPair.second].back());
                    clusters[maxPair.second].pop_back(); 
                }
                /* valid[maxPair.second] = false; */
                int toUpdate = maxPair.first;
                removeValue = maxPair.second;

                /* update the distance matrix */
                for(int i = 0; i < N; i++){
                    if(LinkageType == 0){
                        distances[toUpdate*N+i] = distances[i*N+toUpdate] = minimumLinkage(K, clusters[i], clusters[toUpdate]);
                    } else if(LinkageType == 1){
                        /* Average linkage has the following update rule
                         *
                         * D(X u Y, A) = (1/ |X u Y|) * [ |X| D(X,A) + |Y| D(Y,A) ]
                         *
                         * */
                        distances[toUpdate*N+i] = (1.0 / (SToUpdate+SRemoveValue))*(
                                                    SToUpdate*distances[toUpdate*N+i]+SRemoveValue*distances[removeValue*N+i]
                                                                                    );
                        distances[i*N+toUpdate] = distances[toUpdate*N+i];

                        /*
                        distances[toUpdate*N+i] = distances[i*N+toUpdate] = averageLinkage(K, clusters[i], clusters[toUpdate]);
                        */
                    } else {
                        distances[toUpdate*N+i] = distances[i*N+toUpdate] = maximumLinkage(K, clusters[i], clusters[toUpdate]);
                    }
                }
            }

            /* free the distances matrix */
            delete [] distances;

            /* return the valid clusters */
            vector<FrameData::Cluster > finalClusters;
            for(int i = 0; i < N; i++){
                if(!valid[i]){
                    continue;
                }
                if((int) clusters[i].size() > MinPts){
                    finalClusters.push_back(clusters[i]);
                }
            }
            return finalClusters; 
        }

        Float clamp(Float val, Float minVal, Float maxVal){
            if(val < minVal){
                return minVal;
            } else if(val > maxVal){
                return maxVal;
            } 
            return val; 
        }

        IplImage *renderMatrix(SP_Image K){
            int N = K->width;
            IplImage *render = cvCreateImage(cvSize(N, N), IPL_DEPTH_8U, 3);
            cvZero(render);
            Float maxValue = -1;
            for(int i = 0; i < N; i++){
                for(int j = 0; j < N; j++){
                    if(K->getProb(i,j) > maxValue){
                        maxValue = K->getProb(i,j);
                    } 
                }
            }
            for(int i = 0; i < N; i++){
                for(int j = 0; j < N; j++){
                    Float renderValue = K->getProb(i,j) / maxValue;
                    /* jet coloring */
                    Float vx4 = 4*renderValue;
                    unsigned char *pix = &(CV_IMAGE_ELEM(render, unsigned char, i, j*3));
                    pix[2] = (unsigned char) clamp((min(vx4-1.5, 4.5-vx4)*255), 0, 255);
                    pix[1] = (unsigned char) clamp((min(vx4-0.5, 3.5-vx4)*255), 0, 255);
                    pix[0] = (unsigned char) clamp((min(vx4+0.5, 2.5-vx4)*255), 0, 255);
                }
            }
            return render;
        }

		
		void process( FrameData &frameData ) {

			frameData.clusters.resize( models->size() );

            /* find the depthmap and distance map */
            SP_Image depthmap, distanceMap;
            for(int i = 0; i < (int) frameData.images.size(); i++){
                if(frameData.images[i]->imageType == IMAGE_TYPE_DEPTH_MAP){
                    depthmap = frameData.images[i];
                    break;
                }
            }

            /* search for a depthmap fill distance image */
            for(int i = 0; i < (int) frameData.images.size(); i++){
                if(frameData.images[i]->imageType == IMAGE_TYPE_PROB_MAP){
                    if(frameData.images[i]->name == depthmap->name+".distance"){
                        distanceMap = frameData.images[i];
                        break; 
                    }
                }
            }

            int start = depthmap->name.rfind("/")+1;
            string basename = depthmap->name.substr(start);
            /*
            cerr << "saving with basename: " << basename << endl;
	        */	
			#pragma omp parallel for
			for( int model=0; model<(int)frameData.matches.size(); model++) {


                /* get the current matches */
                vector<FrameData::Match> matches = frameData.matches[model];
                   

                SP_Image K2D, K3D, BK, K;

                /* get 2D and 3D kernel sigmas */
                Float k2DSigma, k3DSigma;

                /* if we don't know one of them, compute them */
                if(Sigma2D == -1 || Sigma3D == -1){
                    getAverageNNDistances(matches, k2DSigma, k3DSigma);
                    if(Sigma2D != -1){
                        k2DSigma = Sigma2D;
                    }
                    if(Sigma3D != -1){
                        k3DSigma = Sigma3D;
                    }
                } else {
                    k2DSigma = Sigma2D;
                    k3DSigma = Sigma3D;
                }

                /*
                cerr << "Sigma2D = " << k2DSigma << endl;
                cerr << "Sigma3D = " << k3DSigma << endl;
                */

                /*
                cerr << "AVG2D: " << k2DSigma << endl;
                cerr << "AVG3D: " << k3DSigma << endl;
                */

                /* calculate the GGaussian kernels */
                K2D = getGaussK(matches, k2DSigma, false);
                K3D = getGaussK(matches, k3DSigma, true);

                /*
                 * Uh - these are normalized by default? 
                normalizeSimilarityMatrix(K2D); normalizeSimilarityMatrix(K3D);
                */

                /*
                cvSaveImage((basename+"K2D.png").c_str(), renderMatrix(K2D));
                cvSaveImage((basename+"K3D.png").c_str(), renderMatrix(K3D));
                */

                /* calculate the discontinuity matrix */
                BK = getDiscontinuityMatrix(matches, depthmap); 

                /*
                cvSaveImage((basename+"BK.png").c_str(), renderMatrix(BK));
                */

                /* Set K3D = BK * K3D (per element) */
                getSum(K3D, BK, K3D);
                normalizeSimilarityMatrix(K3D);

                if(Use3DFilter){
                    SP_Image K3F = get3DFilterK(matches);
                    /*
                    cvSaveImage((basename+"K3F.png").c_str(), renderMatrix(K3F));
                    */
                    if(Use3DFilter == 1){
                        getSum(K3D, K3F, K3D);
                    } else { 
                        getProduct(K3D, K3F, K3D);
                    }
                    normalizeSimilarityMatrix(K3D);
                }

/*                cvSaveImage((basename+"K3DpBK.png").c_str(), renderMatrix(K3D)); */

                /* get the final similarity matrix */
                K = adaptiveWeightSum(matches, distanceMap, K2D, K3D, 0.5, 25);

                /*
                ofstream out;
                out.open((basename+(*models)[model]->name+"feat.txt").c_str());

                for(int i = 0; i < (int) matches.size(); i++){
                    FrameData::Match m = matches[i];
                    out << (int) m.coord2D[0] << " " << (int) m.coord2D[1] << endl; 
                }
                out.close();
                cvSaveImage((basename+(*models)[model]->name+"AK.png").c_str(), renderMatrix(K)); 
                */
                
                vector<FrameData::Cluster >clusters = hierarchicalCluster(K);
                frameData.clusters[model] = clusters;
			}
			
			if( _stepName == "CLUSTER" ) frameData.oldClusters = frameData.clusters;
		}
	};
};
