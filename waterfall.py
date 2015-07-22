#    This file is part of pyrlsdr.
#    Copyright (C) 2013 by Roger <https://github.com/roger-/pyrtlsdr>
#
#    pyrlsdr is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    pyrlsdr is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with pyrlsdr.  If not, see <http://www.gnu.org/licenses/>.


from __future__ import division
import matplotlib.animation as animation
from matplotlib.mlab import psd
import pylab as pyl
import numpy as np
import sys
import math
from rtlsdr import RtlSdr
from rtlsdr import librtlsdr
import matplotlib as mpl




# A simple waterfall, spectrum plotter
#
# Controls:
#
# * Scroll mouse-wheel up or down, or press the left or right arrow keys, to
#   change the center frequency (hold shift for finer control).
# * Press "+" and "-" to control gain, and space to enable AGC.
# * Type a frequency (in MHz) and press enter to directly change the center frequency
SCAN_START = 400e6
SCAN_END = 1000e6

NFFT = 1024
SAMP_RATE=2.6e6
NUM_BUFFERED_SWEEPS = 100

NUM_SAMPLES_PER_SCAN = NFFT*32 # Time at each freq
NUM_SCANS_PER_SWEEP = math.ceil(abs(SCAN_END-SCAN_START)/SAMP_RATE)


class Waterfall(object):
    
    image_buffer = -100*np.ones( (NUM_BUFFERED_SWEEPS, NUM_SCANS_PER_SWEEP*NFFT) )

    def __init__(self, sdrs, fig=None):
        self.fig = fig if fig else pyl.figure()
        self.sdrs = sdrs
        
        print("Using:",len(self.sdrs),"sdr")
        self.init_plot()

    def init_plot(self):
        
        self.ax = self.fig.add_subplot(1,1,1)
        self.fig.frameon = False
        self.ax.axis('off')
        
        self.image = self.ax.imshow(self.image_buffer, aspect='auto',\
                                    interpolation='nearest', vmin=-50, vmax=10)
        self.ax.set_xlabel('Current frequency (MHz)')
        self.ax.get_yaxis().set_visible(False)

        pyl.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
        figManager = pyl.get_current_fig_manager()
        figManager.full_screen_toggle()

    def update_plot_labels(self):
        freq_range = (SCAN_START - SAMP_RATE/2)/1e6, (SCAN_START + SAMP_RATE*(NUM_SCANS_PER_SWEEP - 0.5))/1e6
        self.image.set_extent(freq_range + (0, 1))
        self.fig.canvas.draw_idle()

    def add_samples(self, samples, context):
        scan_num, idx = context
        self.sdrs[idx].cancel_read_async()
        
        psd_scan, f = psd(samples, NFFT=NFFT) # estimate PSD for one scan
        self.image_buffer[0, (scan_num+idx)*NFFT: ((scan_num+idx)*NFFT)+NFFT] = 10*np.log10(psd_scan)
        
    def update(self, *args):
        
        # prepare space in buffer
        # TODO: use indexing to avoid recreating buffer each time
        self.image_buffer = np.roll(self.image_buffer, 1, axis=0)
        self.image_buffer[0] = 0
        print("New Line")
        
        for scan_num in range(0, NUM_SCANS_PER_SWEEP,len(self.sdrs)):
            for idx, sdr in enumerate(self.sdrs):
                
                center_freq = SCAN_START+((scan_num+idx)*SAMP_RATE)
                
                if(center_freq>SCAN_END): continue
                #print("tuning:",idx,"to:",center_freq)

                sdr.center_freq=center_freq
                psd_scan1, f = psd( sdr.read_samples(NUM_SAMPLES_PER_SCAN) , NFFT=NFFT) # estimate PSD for one scan
                self.image_buffer[0, (scan_num+idx)*NFFT: ((scan_num+idx)*NFFT)+NFFT] += 10*np.log10(psd_scan1)
                self.image_buffer[0, (scan_num+idx)*NFFT: ((scan_num+idx)*NFFT)+NFFT] /= 2

                '''
                #print(((scan_num+idx)*NFFT)+(NFFT/2), ((scan_num+idx)*NFFT)+(NFFT*1.5))
                if ( ((scan_num+idx)*NFFT)+(NFFT*1.5) <= NUM_SCANS_PER_SWEEP*NFFT):
                    sdr.center_freq=center_freq+(SAMP_RATE/2)
                    psd_scan2, f = psd( sdr.read_samples(NUM_SAMPLES_PER_SCAN) , NFFT=NFFT) # estimate PSD for one scan
                    self.image_buffer[0, ((scan_num+idx)*NFFT)+(NFFT/2) : ((scan_num+idx)*NFFT)+(NFFT)+(NFFT/2)] += 10*np.log10(psd_scan2)
                    self.image_buffer[0, ((scan_num+idx)*NFFT)+(NFFT/2) : ((scan_num+idx)*NFFT)+(NFFT)+(NFFT/2)] /= 2
                '''  
        #Wait for all dongles to finish sweep
        # plot entire sweep
        self.image.set_array(self.image_buffer)
        
       # self.image.set_array(self.image_buffer)

        return self.image,

    def start(self):
        self.update_plot_labels()
        if sys.platform == 'darwin':
            # Disable blitting. The matplotlib.animation's restore_region()
            # method is only implemented for the Agg-based backends,
            # which the macosx backend is not.
            blit = False
        else:
            blit = True
        ani = animation.FuncAnimation(self.fig, self.update, interval=.01,
                blit=blit)

        pyl.show()
    
        return


def main():
    sdrs = list()

    
    mpl.rcParams['toolbar'] = 'None'
    
    num_sdrs =  librtlsdr.rtlsdr_get_device_count()
    # some defaults
    for idx in range( 0, num_sdrs):
        sdr = RtlSdr(device_index=idx)
        sdr.sample_rate = SAMP_RATE
        sdr.center_freq = SCAN_START
        sdr.gain = 100000
        sdrs.append(sdr)
        
    wf = Waterfall(sdrs)

    wf.start()

    # cleanup
    sdr.close()


if __name__ == '__main__':
    main()
