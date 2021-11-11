# -*- coding: utf-8 -*-
"""
Created on Thu May 20 13:35:50 2021

@author: enriq
"""

import numpy as np
from matplotlib import pyplot as plt

data = np.load('00_data.npz')




# n = 2499-1

# plt.figure(figsize = (10,10))
# plt.plot(r.real,r.imag,linewidth=2,zorder=1)
# plt.scatter(r[n,:].real,r[n,:].imag,marker='o' ,color='r',zorder=4)
# plt.title('Phase Plane')
# plt.xlabel(r'$x [m]$')
# plt.ylabel(r'$y [m]$')
# plt.grid()