#!/usr/bin/env python

import numpy
from math import factorial


class calibration:

    def __init__(self):

        self.turn_360_steps = 455

        #464,512 ,474


    def calibrate_robot(self,data):

        wave = self.measure_step_to_360(data)
        digital_wave = self.digitalise_wave(wave)
        self.turn_360_steps = self.compute_num_steps_to_360(digital_wave)


    def measure_step_to_360(self,data):

        nsonar = numpy.array(data)

        #give a odd number
        length1 = len(nsonar)/5
        if length1%2 == 0:
            length1 += 1


        #give a odd number
        length2 = len(nsonar)/10
        if length2%2 == 0:
            length2 += 1

        yhat1 = self.savitzky_golay(nsonar, length1, 2)
        yhat2 = self.savitzky_golay(yhat1, length1, 1)
        yhat3 = self.savitzky_golay(yhat2, length2, 1)
        yhat4 = self.savitzky_golay(yhat3, length2, 1)

        return yhat4

    # through way to make a analog signal digital
    def digitalise_wave(self,wave):

        digital = []
        for i in wave:
          if i > numpy.mean(wave) :
             digital.append(1)
          else:
             digital.append(0)

        return digital

    # compute number of step to 360 through the peaks
    def compute_num_steps_to_360(self,digital_signal):

        rising_edge_list = []
        for i in xrange(len(digital_signal)-1):
            previous = digital_signal[i]
            current = digital_signal[i+1]
            if current - previous == 1:
                rising_edge_list.append(i)

        peaks = zip(rising_edge_list, rising_edge_list[1:])

        periods = []
        for i in peaks:
            periods.append(i[1] - i[0])

        return sum(periods)/len(periods)



    def savitzky_golay(self, y, window_size, order, deriv=0, rate=1):

        try:
            window_size = numpy.abs(numpy.int(window_size))
            order = numpy.abs(numpy.int(order))
        except ValueError, msg:
            raise ValueError("window_size and order have to be of type int")
        if window_size % 2 != 1 or window_size < 1:
            raise TypeError("window_size size must be a positive odd number")
        if window_size < order + 2:
            raise TypeError("window_size is too small for the polynomials order")
        order_range = range(order+1)
        half_window = (window_size -1) // 2
        # precompute coefficients
        b = numpy.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
        m = numpy.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
        # pad the signal at the extremes with
        # values taken from the signal itself
        firstvals = y[0] - numpy.abs( y[1:half_window+1][::-1] - y[0] )
        lastvals = y[-1] + numpy.abs(y[-half_window-1:-1][::-1] - y[-1])
        y = numpy.concatenate((firstvals, y, lastvals))

        return numpy.convolve( m[::-1], y, mode='valid')