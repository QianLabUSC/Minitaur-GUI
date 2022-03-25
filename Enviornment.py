import matplotlib.pyplot as plt
import numpy as np
import math

def obstacle(ax2, num = 0, x_o=[], y_o=[], z_o=[], d = 3, p = 6):
    if len(x_o)==0 and len(y_o)==0 and len(z_o)==0:
        x_ob = np.arange(-30, 30, 10)
        y_ob = np.arange(-d/2, d/2, 0.1)
        x_o, y_o = np.meshgrid(x_ob, y_ob)
        z_o = d/2*np.sin(np.arccos(abs(y_o/(d/2))))
    ax2.plot_surface(x_o, num * p + d/2 + y_o, z_o, cmap = 'rainbow')
    return x_o, y_o, z_o