/*
 * David Fouhey
 * Benchmark class
 *
 */

#include "Benchmark.hpp"


/*! \brief Get the current time
 * Gets the current time in seconds.
 */
double getTime(){
    benchClock currentTime;
    clock_gettime(CLOCK_REALTIME, &currentTime);
    return (currentTime.tv_sec*1000000000LL + currentTime.tv_nsec)/1000000000.;
}
/*! \brief Return the difference between two times.
 * Returns the difference between two clocks in seconds.
 */
inline double getTimeDiff(benchClock *start, benchClock *end){
    return ((end->tv_sec - start->tv_sec)*1000000000LL + (end->tv_nsec - start->tv_nsec))/1000000000.;
}


/*!
 * \brief Create a benchmark instance logging to cout.
 * Construct a benchmark instance. By default, output the benchmark 
 * data to cout.
 */ 
Benchmark::Benchmark(){
    output = &cout;
}

/*!
 * \brief Redirect the logging to a file.
 * Close any previous log file, and redirect logging to a new file with the 
 * given name.
 */
void Benchmark::redirect(string filename){
    if(output != &cout){
        delete output;
    }
    output = new ofstream(filename.c_str());
}

/*!
 * \brief Benchmark destructor.
 * Destroy the benchmark, deleting the stream if it's not cout
 */ 
Benchmark::~Benchmark(){
    if(output != &cout){
        delete output;
    }
}

/*!
 * \brief Log the time as a number.
 * Log the time in the form of a number without any formatting.
 */
void Benchmark::logRawTime(){
    *output << getTime() << endl;
}

/*!
 * \brief Start a clock.
 * Allocate, start and then return a clock, call tock or logTock to stop the 
 * clock. Note: if neither tock nor logTock are called, this induces a memory
 * leaka
 */
benchClock *Benchmark::tick(){
    benchClock *clock = new benchClock;
    clock_gettime(CLOCK_REALTIME, clock);   
    return clock;
}

/*!
 * \brief Stop a clock
 * Stop a clock, free it, and return the time since its start.
 */
double Benchmark::tock(benchClock *clock){
    benchClock end;
    clock_gettime(CLOCK_REALTIME, &end);
    double diff = getTimeDiff(clock, &end);
    delete clock;
    return diff;
}

/* \brief Log the time difference between now and the start of the clock.
 * Stop the given clock, and log the time taken.
 */
void Benchmark::logTock(benchClock *clock){
    *output << tock(clock) << endl;
}

/*
 * Example program:
 
int main(){
    Benchmark bench("asdf.txt");
    bench << "Hello!" << endl;
    benchClock *c = bench.tick();
    int x = 0;
    for(int i = 0; i < 10000; i++){for(int j = 0; j < 10000; j++){x++;}}
    bench.logTock(c);

    return 0;
}
*/

