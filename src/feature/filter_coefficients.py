import numpy as np
from scipy.signal import butter

def butter_sos(order, cutoff, btype, fs):
    return butter(order, cutoff, btype=btype, fs=fs, output='sos')

def sos_to_cmsis(sos):
    """Convert SciPy SOS to CMSIS biquad rows"""
    rows=[]
    for b0,b1,b2,a0,a1,a2 in sos:
        rows.append([b0/a0, b1/a0, b2/a0, -a1/a0, -a2/a0])
    return np.array(rows)

fs = 100.0
order = 5

sos_hp=butter_sos(order,0.5,'highpass',fs)
sos_lp=butter_sos(order,10.0,'lowpass',fs)

cmsis_hp=sos_to_cmsis(sos_hp)
cmsis_lp=sos_to_cmsis(sos_lp)

print(cmsis_hp)
print(cmsis_lp)
