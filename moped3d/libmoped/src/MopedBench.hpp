/*
 * David Fouhey
 * Moped Benchmarking header
 */
#pragma once
#include <utility>
#include "Benchmark.hpp"
#include "moped.hpp"
#include "util.hpp"

//#define DO_MOPED_BENCH

using namespace MopedNS;

/*
 * \brief Function typedef for output functions.
 * Output function: 
 *      Gives access to the current benchmark logger, the name of the step 
 *      that's about to be run, the current frame data, and access to a
 *      void * to store things between the algorithm call.
 */
typedef void (*BenchStep)(Benchmark &bench, string stepName, FrameData &frame, void *&auxData);

class MopedBench{
    private:
        /* \brief The benchmark log accessor. */
        Benchmark bench;
        /* \brief A clock that is used to time the algorithm */
        benchClock *algStart;
        /* \brief Map step names to pre/post output actions. */
        map<string, pair<BenchStep,BenchStep> > actions;
        /* \brief Auxiliary data for the pre/post output functions */
        void *auxData;
    public:
        /* Call to initialize the benchmarking code */
        void init();
        /* Call before the algorithm is run */
        void beforeAlgorithm(string &stepName, FrameData &);
        /* Call after the algorithm is run */
        void afterAlgorithm(string &stepName, FrameData &);
        /* Call after moped is done */
        void allDone(FrameData &);
        /* Get access to the benchmark class; useful for output */
        Benchmark *getBenchmark();
};


