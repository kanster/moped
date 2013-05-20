#include <iostream>
#include <fstream>
#include <time.h>

#pragma once

using namespace std;
typedef struct timespec benchClock;

/*! \brief A class for benchmarking
 * 
 * This defines a class for benchmarking.
 */
class Benchmark{
    public:
        ostream *output;
        Benchmark();
        Benchmark(string filename);
        ~Benchmark();
        void logRawTime();
        void logTime();
        benchClock *tick();
        double tock(benchClock *clock);
        void logTock(benchClock *clock);
        void redirect(string filename);
        /* 
         * Thanks to: 
         *  http://stackoverflow.com/questions/2612116/operator-cannot-output-stdendl-fix
         * for getting this to work. endl is a function, and something like 
         * this is necessary for it to work
         */
        ostream& operator<<(ostream&(*f)(ostream&)){ 
            return f(*output); 
        }
};
/*
 * Define a << operator so that we can do things like
 *    bench << "Hello World";
 */
template<typename T> Benchmark& operator << (Benchmark& o, T s){
    *(o.output) << s;
    return o;
}


