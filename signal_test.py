import numpy, random, matplotlib
import sys
import scipy
import scipy.signal as signal

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import peakdetect
from rtlsdr import RtlSdr

block_size=256

def main():

  #get data
  sdr = RtlSdr(1)
  sdr.sample_rate = 2.4e6  # Hz
  sdr.center_freq = 485.9e6     # Hz
  #sdr.center_freq = 26e6     # Hz
  #sdr.freq_correction = 60   # PPM
  sdr.gain = 100

  fft_bins = numpy.fft.fftfreq(block_size,d=1/sdr.sample_rate)
  fft_bins = numpy.fft.fftshift(fft_bins)
  fft_bins+=sdr.center_freq  
  
  while True:
    data = sdr.read_samples(block_size)
    
    #fft next block
    fft_data = abs(numpy.fft.fft(data))
    fft_data = numpy.fft.fftshift(fft_data)
    #peaks = signal.find_peaks_cwt(fft_data, numpy.arange(1,10), noise_perc=0, min_snr=3) #TODO look into fixing this
    fft_data = fft_data/numpy.linalg.norm(fft_data)
    peaks = peakdetect.peakdet(fft_data, .5)
    freqs = list()
    if peaks.size>0:
      for f in peaks:
        freqs.append(fft_bins[f]/1000000)
      print(freqs)

  
if __name__ == "__main__":
    main()
