# -*- coding: utf-8 -*-
"""
Created on Fri Jul 30 11:04:38 2021

@author: enriq
"""

import nummp as np

data = np.load('DATA/PE2_data.npz')

N = data['robots']
tspan = data['tiempo']
tspan2 = data['tiempo2']
Xr = data['Xref']
Yr = data['Yref']
Xir = data['Xiref']
x = data['xest']
y = data['yest']
xi = data['xiest']
err_x = data['errx']
err_y = data['erry']
err_xi = data['errxi']
v = data['vellin']
w = data['velang']

