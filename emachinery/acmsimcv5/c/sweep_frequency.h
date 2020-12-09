#include "ACMSim.h"


struct SweepFreq{
    REAL time;
    REAL freq_step_size;
    REAL current_freq;
    REAL      current_freq_end_time;
    REAL last_current_freq_end_time;
};
