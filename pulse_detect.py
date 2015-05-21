import numpy, random, matplotlib
import sys
import scipy
import scipy.signal as signal

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import peakdetect
from rtlsdr import RtlSdr

block_size=512

def update_func(i, lines, sdr):

    data = sdr.read_samples(block_size)
    
    #fft next block
    fft_data = abs(numpy.fft.fft(data))
    fft_data = numpy.fft.fftshift(fft_data)
    #peaks = signal.find_peaks_cwt(fft_data, numpy.arange(1,10), noise_perc=0, min_snr=3)
    fft_data = fft_data/numpy.linalg.norm(fft_data)
    fft_data[0]=0 #dont care about dc
    
    peaks = peakdetect.peakdet(fft_data, .3)
    if peaks.size==0:
      peaks={}
    lines[0].set_ydata( abs(fft_data) )

    #set peak detect graph
    peak_data = numpy.full(block_size, -1)
    for x in peaks:
      peak_data[x]=fft_data[x] #set to top of fft peak
    lines[1].set_ydata( peak_data )
    

def main():

  #get data
  sdr = RtlSdr()
  sdr.sample_rate = 2.4e6  # Hz
  #sdr.center_freq = 485.9e6     # Hz
  sdr.center_freq = 26.5e6     # Hz
  #sdr.freq_correction = 60   # PPM
  sdr.gain = 0
  
  data = sdr.read_samples(block_size)
  fft_data = abs(numpy.fft.fft(data))
  fft_data = fft_data/numpy.linalg.norm(fft_data)
  fft_data = numpy.fft.fftshift(fft_data) #rearange the data so it makes sence 
  
  x_axis = numpy.fft.fftfreq(data.size,d=1/sdr.sample_rate)
  x_axis = numpy.fft.fftshift(x_axis)
  x_axis+=sdr.center_freq  

  #plot first frame
  fig, subs = plt.subplots(nrows=1, ncols=1)
  subs.set_title('FFT')
  lines = [None] * 4
  fft_data[0]=1
  lines[0], = subs.plot( x_axis, fft_data )
  lines[1], = subs.plot( x_axis,  numpy.zeros(block_size), "ro" )
 
  ani = animation.FuncAnimation(fig, update_func, fargs=(lines,sdr), interval=10)
  plt.show()
  
if __name__ == "__main__":
    main()
