
/*
 * LICENCE: BSD
 */
#pragma once
#define MAX_THREADS 8
#include <util/UTIL_UNDISTORT.hpp>
#include <feat/FEAT_SIFT_GPU.hpp>
#include <feat/FEAT_SIFT_CPU.hpp>
#include <feat/FEAT_SURF_CPU.hpp>
#include <feat/FEAT_DISPLAY.hpp>
#include <match/MATCH_HASH_CPU.hpp>
#include <match/MATCH_ANN_CPU.hpp>
#include <match/MATCH_ADAPTIVE_FLANN_CPU.hpp>
#include <match/MATCH_FLANN_CPU.hpp>
#include <match/MATCH_DISPLAY.hpp>
#include <depthfilter/DEPTHFILTER_CPU.hpp>
#include <depthfilter/DEPTHFILTER_IGNORE_CPU.hpp>
#include <depthfill/DEPTH_FILL_EXACT_CPU.hpp>
#include <depthprop/DEPTHMAP_PROP_CPU.hpp>
#include <cluster/CLUSTER_MEAN_SHIFT_CPU.hpp>
#include <cluster/CLUSTER_WEIGHTED_MEAN_SHIFT_CPU.hpp>
#include <cluster/CLUSTER_CUSTOM_CPU.hpp>
#include <cluster/CLUSTER_LINKAGE_CPU.hpp>
#include <cluster/CLUSTER_DISPLAY.hpp>
#include <pose/POSE_RANSAC_LM_DIFF_REPROJECTION_CPU.hpp>
#include <pose/POSE_RANSAC_LM_DIFF_BACKPROJECTION_DEPTH_CPU.hpp>
#include <pose/POSE_RANSAC_LBFGS_REPROJECTION_CPU.hpp>
#include <pose/POSE_LBFGS_CPU.hpp>
#include <pose/POSE_DISPLAY.hpp>
#include <filter/FILTER_PROJECTION_CPU.hpp>
#include <STATUS_DISPLAY.hpp>
#include <GLOBAL_DISPLAY.hpp>
#define DEFAULT_DISPLAY_LEVEL 1
namespace MopedNS {
	
	void createPipeline( MopedPipeline &pipeline ) {
 pipeline.addAlg( "UNDISTORTED_IMAGE", new UTIL_UNDISTORT );
 pipeline.addAlg("DEPTHFILL", new DEPTH_FILL_EXACT_CPU(8, false));
 pipeline.addAlg( "SIFT", new FEAT_SIFT_GPU("-1", "0", ":0.0") );
 pipeline.addAlg( "DEPTHFILTER", new DEPTHFILTER_CPU(64, 0.050000, 1));
 pipeline.addAlg( "MATCH_SIFT", new MATCH_ADAPTIVE_FLANN_CPU( 128, "SIFT", 8, 0.600000, 0.750000, 0.650000, 0.800000, 150, 50));
 pipeline.addAlg( "DEPTHFILTER2", new DEPTHFILTER_CPU(64, 0.010000, 2));
 pipeline.addAlg("DEPTHPROP", new DEPTHMAP_PROP_CPU());
 pipeline.addAlg( "CLUSTER", new CLUSTER_LINKAGE_CPU( 0.100000, 7, 2, 1, 0.0, 1, -1, -1));
 pipeline.addAlg( "POSE", new POSE_RANSAC_LM_DIFF_BACKPROJECTION_DEPTH_CPU( 192, 100, 4, 5, 6, 8, 0.5));
 pipeline.addAlg( "FILTER", new FILTER_PROJECTION_CPU( 6, 4096., 2) );
 pipeline.addAlg( "POSE2", new POSE_RANSAC_LM_DIFF_BACKPROJECTION_DEPTH_CPU( 64, 250, 4, 6, 8, 5, 0.5));
 pipeline.addAlg( "FILTER2", new FILTER_PROJECTION_CPU( 8, 8192., 1e-4) );
	}
};
