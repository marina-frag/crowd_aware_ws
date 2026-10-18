#!/usr/bin/env python3
"""Exercise compiled Matplotlib extensions, actual rendering, and optional X11 GUI."""
import argparse
import io


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--gui',action='store_true');a=p.parse_args()
    import numpy as np
    import matplotlib
    matplotlib.use('TkAgg' if a.gui else 'Agg',force=True)
    import matplotlib.pyplot as plt
    import tkinter
    tkinter.Tcl()  # Does not require DISPLAY; verifies the installed Tk bindings.
    print(f'NumPy {np.__version__}: {np.__file__}',flush=True)
    print(f'Matplotlib {matplotlib.__version__}: {matplotlib.__file__}',flush=True)
    figure,axis=plt.subplots()
    axis.plot(np.linspace(0,1,10),np.linspace(0,1,10)**2)
    axis.set_title('DWAL plotting compatibility check')
    figure.canvas.draw()  # Loads/executes the native rendering extensions.
    buffer=io.BytesIO();figure.savefig(buffer,format='png')
    if not buffer.getvalue().startswith(b'\x89PNG\r\n\x1a\n'):raise RuntimeError('PNG rendering failed')
    print('PASS: actual figure rendered to PNG; Tk bindings available.',flush=True)
    if a.gui:plt.show()
    plt.close(figure)

if __name__=='__main__':main()
