import numpy, random, matplotlib
import sys
import scipy
import scipy.signal as signal

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import peakdetect
from rtlsdr import RtlSdr

block_size=256

def update_func(i, lines, sdr):

    data = sdr.read_samples(block_size)
    
    #fft next block
    fft_data = abs(numpy.fft.fft(data))
    fft_data = numpy.fft.fftshift(fft_data)
    #peaks = signal.find_peaks_cwt(fft_data, numpy.arange(1,10), noise_perc=0, min_snr=3)
    fft_data = fft_data/numpy.linalg.norm(fft_data)
    fft_data[0]=0 #dont care about dc
    
    peaks = peakdetect.peakdet(fft_data, .3)
    if peaks.size>0:
      print(peaks)
    else:
      peaks={}
    lines[0].set_ydata( abs(fft_data) )

    #set peak detect graph
    peak_data = numpy.full(block_size, -1)
    for x in peaks:
      peak_data[x]=fft_data[x] #set to top of fft peak
    lines[1].set_ydata( peak_data )
    
    
    lines[2].set_ydata( data.real / numpy.linalg.norm(data.real) )
    lines[3].set_ydata( data.imag / numpy.linalg.norm(data.imag) )

def main():

  #get data
  sdr = RtlSdr(1)
  sdr.sample_rate = 1e6  # Hz
  #sdr.center_freq = 485.9e6     # Hz
  sdr.center_freq = 120e6     # Hz
  #sdr.freq_correction = 60   # PPM
  sdr.gain = 'auto'
  
  data = sdr.read_samples(block_size)
  fft_data = abs(numpy.fft.fft(data))
  fft_data = fft_data/numpy.linalg.norm(fft_data)
  fft_data = numpy.fft.fftshift(fft_data) #rearange the data so it makes sence 
  
  x_axis = numpy.fft.fftfreq(data.size,d=1/sdr.sample_rate)
  x_axis = numpy.fft.fftshift(x_axis)
  x_axis+=sdr.center_freq  

  #plot first frame
  fig, subs = plt.subplots(nrows=2, ncols=1)
  subs[0].set_title('FFT')
  subs[1].set_title('Input real/imag')
  lines = [None] * 4
  fft_data[0]=1
  lines[0], = subs[0].plot( x_axis, fft_data )
  lines[1], = subs[0].plot( x_axis,  numpy.zeros(block_size), "ro" )
  lines[2], = subs[1].plot( x_axis, data.real/numpy.linalg.norm(data.real), "r" )
  lines[3], = subs[1].plot( x_axis, data.imag/numpy.linalg.norm(data.imag), "g" )
 
  ani = animation.FuncAnimation(fig, update_func, fargs=(lines,sdr), interval=10)
  plt.show()
  
if __name__ == "__main__":
    main()
