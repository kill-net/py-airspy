import libairspy ,ctypes
import traceback
from pylab import *
import numpy as np
airspy = libairspy.airspy()
import numpy as np
fc=1082 * 1000 * 1000
sr=10 * 1000 * 1000
if airspy.is_open == False:

    airspy.setup()
    airspy.open()
    try:
        airspy.set_freq(fc)
        airspy.set_sample_rate(sr)
        airspy.set_mixer_gain(0)
        airspy.set_lna_gain(10)
        airspy.set_vga_gain(5)
    except:
        traceback.print_exc()
    finally:
        pass

rx_count=0
pow=[]
samaple=[]
freqs=[]
def callback_fun(airspy_transfer):
    global  rx_count
    global samaple

    print airspy_transfer.contents.airspy_sample_type

    array_type = (ctypes.c_float*airspy_transfer.contents.sample_count)
    values = ctypes.cast(airspy_transfer.contents.samples, ctypes.POINTER(array_type)).contents
    samaple=libairspy.packed_bytes_to_iq(values)
    #print     '=====',len(airspy_transfer.contents.samples), airspy_transfer.contents.sample_count

    #iq data here
    rx_count+=1
    return 0

airspy.start_rx_mode(callback_fun)
while rx_count<=1:
    pass
airspy.stop_rx_mode()
pow, freqs = psd(samaple, NFFT=4000, window=mlab.window_hanning, Fs=sr / 1e6,
                 Fc=fc / 1e6)
savefig('1.jpg')
print rx_count
pow=10*np.log10(pow)
print np.max(pow)
for i in range(1100,2200):
    print  freqs[i],pow[i]

