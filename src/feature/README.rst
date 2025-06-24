1. Overview
***********

This is the additional features that run with the help of DSP library and FPU support on the nRF5340 SoC's application core. The dsp_filter.c/h includes the usage of
a 5-th order Butterworth bandpass filter. The gait_analysis.c/h implements a gait detection and analysis algorithm improved based on this `paper <https://www.frontiersin.org/journals/neurology/articles/10.3389/fneur.2021.719442/full>`_

2. DSP Filter
*************

One can run the supplied python helper to get the high pass coefficients and low pass coefficients defined in the source code. The size of each array depends on the order 
of the filter. Cut-off frequencies will affect the coefficients inside the array. It is the developer's responsibility to choose the most suitable cutoff frequency based on 
application requirements and the sample frequency of the sensor data. Current implementation demonstrate a filter for 100Hz data stream. 

3. Gait Analysis
****************

Both gait quantity and gait quality is estimated in the source code. 
