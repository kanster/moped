/*
 * David Fouhey
 * Moped Benchmarking code 
 */
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include "MopedBench.hpp"

#define MOPED_BENCH_USE_FILE true
#define MOPED_BENCH_FILE_PREFIX "outputMopedBench"

using namespace std;
using namespace MopedNS;

/*
 * \brief Return an output name that's probably unique.
 * Return a filename formed from FILE_PREFIX and the current process's PID. 
 */
string getOutputName(){
//    pid_t p = getpid();
    stringstream s;
//    s << MOPED_BENCH_FILE_PREFIX << p << ".txt";
    s << MOPED_BENCH_FILE_PREFIX << ".txt";
    return s.str();
}

/*
 * \brief Return the current benchmark class.
 * A function that returns the current benchmark class
 */
Benchmark *MopedBench::getBenchmark(){
    return &bench; 
}

/*
 * \brief Output nothing.
 * A function that outputs nothing 
 */
void outputNothing(Benchmark &bench, string stepName, FrameData &frame, void *&auxData){

}

/*
 * \brief Output the current models
 * A function that outputs all of the models 
 */
void outputModels(Benchmark &bench, string stepName, FrameData &frame, void *&auxData){
    foreach(obj, *(frame.objects)){
        bench << stepName << ":OBJ " << obj->model->name << " " << obj->pose << " " << obj->score << endl;
    }
}

/*
 * \brief Output the current matches.
 * A function that outputs all the information about each of the current 
 * matches.
 */
void outputMatches(Benchmark &bench, string stepName, FrameData &frame, void *&auxData){
    for(int i = 0; i < (int) frame.matches.size(); i++){
        for(int j = 0; j < (int) frame.matches[i].size(); j++){
            FrameData::Match m = frame.matches[i][j];
            bench << stepName << ":MATCH "; 
            bench << "ModelNum:" << i << ";";
            bench << "Idx:" << m.imageIdx << ";2DLoc:" << m.coord2D;
            bench << ";DepthValid:" << m.depthData.depthValid << ";XYZ: [";
            bench << m.depthData.coord3D[0];
            bench << " " << m.depthData.coord3D[1];
            bench << " " << m.depthData.coord3D[2] << "] ";
            bench << ";Depth:" << m.depthData.depth;
            bench << ";FillDistance:" << m.depthData.fillDistance << endl; 
        } 
    }
}

/*
 * \brief Output the current match count.
 * A function that outputs the number of matches per model 
 */
void outputMatchCount(Benchmark &bench, string stepName, FrameData &frame, void *&auxData){
    bench << stepName << " ";
    for(int i = 0; i < (int) frame.matches.size(); i++){
        if(i != 0){
            bench << "|";
        }
        bench << frame.matches[i].size();
    }
    bench << endl; 
}

/*
 * \brief Output all the descriptors.
 * A function that outputs all of the SIFT descriptors 
 */
void outputFeatureData(Benchmark &bench, string stepName, FrameData &frame, void *&auxData){
    vector<FrameData::DetectedFeature> features = frame.detectedFeatures["SIFT"];
    for(int i = 0; i < (int) features.size(); i++){
        bench << stepName << ":" << features[i].coord2D[0] << "," << features[i].coord2D[1];
        bench << ";";
        for(int j = 0; j < (int) features[i].descriptor.size(); j++){
            bench << features[i].descriptor[j] << " ";
        }
        bench << endl;
    }
}

/*
 * \brief Output the features that comprise the clusters.
 * Outputs, for each cluster, the collection of matches that are in the cluster.
 */
void outputClusterData(Benchmark &bench, string stepName, FrameData &frame, void *&auxData){
    /* there's a vector of clusters for each model */
    vector< vector< FrameData::Cluster> > clusters = frame.clusters;
    vector< vector< FrameData::Match > > matches = frame.matches;
    for(int m = 0; m < (int) clusters.size(); m++){
        /* a clusters is a list of indices of matches giving which matches belong to which cluster */
        vector<FrameData::Cluster> clustersForModel = clusters[m];
        /* go through the clusters */
        for(int ci = 0; ci < (int) clustersForModel.size(); ci++){
            for(FrameData::Cluster::iterator it = clustersForModel[ci].begin(); it != clustersForModel[ci].end(); it++){
                FrameData::Match match = matches[m][*it];
                /* output the model index and cluster index */
                bench << stepName << ":CLUSTER " << m << ";" << ci << ";";
                bench << "[ " << match.coord2D[0] << " " << match.coord2D[1] << " ]" << endl;
            } 
        } 
    }
}

/*
 * \brief Outputs the number of matches in each cluster.
 * Outputs a single line, separated by |, with the number of matches in each cluster
 */
void outputClusterCount(Benchmark &bench, string stepName, FrameData &frame, void *&auxData){
    vector< vector< FrameData::Cluster> > clusters = frame.clusters;
    bench << stepName << ":CLUSTERCOUNT:"; 
    for(int m = 0; m < (int) clusters.size(); m++){
        vector<FrameData::Cluster> clustersForModel = clusters[m];
        if(m != 0){
            bench << "|";
        }
        bench << clustersForModel.size();
    }
    bench << endl;
}

/*
 * \brief Output the convex hull of each detected object.
 * Outputs the convex hull of the projections of the 3D points in each object into 
 * the image.
 */
void outputObjectCH(Benchmark &bench, string stepName, FrameData &frame, void *&auxData){
    /* find a gray image */
    int imageIndex = -1;
    SP_Image image;
    for(int i = 0; i < (int) frame.images.size(); i++){
        if(frame.images[i]->imageType == IMAGE_TYPE_GRAY_IMAGE){
            image = frame.images[i];
            imageIndex = i; 
        }
    }
    foreach(obj, *(frame.objects)){
        bench << stepName << ":OBJHULL:" << obj->model->name;
        /* only output if we know something about the image */
        if(imageIndex != -1){
            std::list< Pt<2> > hull = obj->getObjectHull((Image &) (*image));
            foreach( pt, hull) {
                bench << ":" << pt[0] << ";" << pt[1];
            }
        }
        bench << endl;
    }
}

/*
 * \brief Called before a step runs.
 */
void MopedBench::beforeAlgorithm(string &stepName, FrameData &frame){
    if(actions.count(stepName)){
        actions[stepName].first(bench, "PRE:"+stepName, frame, auxData);
    }
    algStart = bench.tick();
}

/*
 * \brief Called after a step runs.
 */
void MopedBench::afterAlgorithm(string &stepName, FrameData &frame){
    double timeTaken = bench.tock(algStart);
    bench << "TIME:" << stepName << ":" << timeTaken << endl;
    if(actions.count(stepName)){
        actions[stepName].second(bench, "POST:"+stepName, frame, auxData);
    }

}

/*
 * \brief Called when moped is finished.
 * Output the detected objects 
 */
void MopedBench::allDone(FrameData &frame){
    foreach(obj, *(frame.objects)){
        bench << "OBJ: " << obj->model->name << " " << obj->pose << " " << obj->score << endl;
    }
}


/*
 * \brief Initialize the object.
 */
void MopedBench::init(){
    if(MOPED_BENCH_USE_FILE){
        bench.redirect(getOutputName());
    }
//    actions["MATCH_SIFT"] = make_pair(outputFeatureData, outputNothing);
    actions["CLUSTER"] = make_pair(outputMatches, outputClusterCount);
    actions["FILTER"] = make_pair(outputClusterCount, outputClusterCount);
    actions["POSE"] = make_pair(outputClusterCount, outputObjectCH);
//    actions["CLUSTER"] = make_pair(outputMatches, outputClusterCount);
//    actions["CLUSTER"] = make_pair(outputNothing, outputNothing);
//    actions["DEPTHPROP"] = make_pair(outputNothing, outputNothing);
    actions["FILTER2"] = make_pair(outputNothing, outputObjectCH);
}



