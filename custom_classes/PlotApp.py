
from functools import partial
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
from matplotlib.backends.backend_tkcairo import FigureCanvasTkCairo
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk

class PlotApp(tk.Tk):
    global position_estimate

    def __init__(self, crazyflie_logconf, logdata, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.update_idletasks()

        self.configure(background='lightgreen')
        self.geometry("1280x720")
        self.wm_title("Embedded in Tk")
        #self.title("Simple Tkinter App") # Set the title of the main window
        
        # Create a label widget with initial text "Hello, Tkinter!"
        #self.label = tk.Label(self, text="Hello, Tkinter!")
        #self.label.pack() # Add the label widget to the window
        self.t = np.array([])
        self.crazyflie_logconf = crazyflie_logconf

        size = 200
        xdata = np.linspace(0, 4 * np.pi, size * 2, endpoint=0)
        ydata = np.sin(xdata)

        #self.figure, ax = plt.subplots()
        #ln, = ax.plot(xdata[:size], ydata[:size], 'r-', lw=3)

        # GUI Initialization
        self.figure = self._fig_base()
        self.label = tk.Label(self, text="Crazyflie Logger").pack(side="top", fill="both", expand=False)

        self.ax1 = self.figure.add_subplot(3, 1, 1)
        self.ax1.set_title('stateEstimate.x', loc='left', fontstyle='oblique', fontsize='medium')
        self.ax1.set(xlim=[0, 1000], ylim=[-1.0, 1.0], xlabel='Time [ms]', ylabel='X [m]')
        self.signal1, = self.ax1.plot(np.array([]), np.array([]), linewidth=1.2, marker='')

        self.ax2 = self.figure.add_subplot(3, 1, 2)
        self.ax2.set_title('stateEstimate.y', loc='left', fontstyle='oblique', fontsize='medium')
        self.ax2.set(xlim=[0, 1000], ylim=[-1.0, 1.0], xlabel='Time [ms]', ylabel='Y [m]')
        self.signal2, = self.ax2.plot(np.array([]), np.array([]), linewidth=1.2, marker='', color='green')

        self.ax3 = self.figure.add_subplot(3, 1, 3)
        self.ax3.set_title('stateEstimate.z', loc='left', fontstyle='oblique', fontsize='medium')
        self.ax3.set(xlim=[0, 1000], ylim=[-1.0, 1.0], xlabel='Time [ms]', ylabel='Z [m]')
        self.signal3, = self.ax3.plot(np.array([]), np.array([]), linewidth=1.2, marker='', color='orange')

        self.ax3.margins(x=0)

        # Matplotlib animation

        def init():
            print("[PlotApp]: Plot initialization")
            data = np.array([])
            
            self.signal1.set_xdata(data)
            self.signal1.set_ydata(data)

            self.signal2.set_xdata(data)
            self.signal2.set_ydata(data)
            
            self.signal3.set_xdata(data)
            self.signal3.set_ydata(data)

            return self.signal1, self.signal2, self.signal3,

        def update(frame):

            t = self.signal1.get_xdata()
            x = self.signal1.get_ydata() # numpy.ndarray class
            y = self.signal2.get_ydata()
            z = self.signal3.get_ydata()

            t = np.append(t, frame*1)
            x = np.append(x, logdata[0]) # frame[1]['stateEstimate.x'])
            y = np.append(y, logdata[1]) # frame[1]['stateEstimate.y'])
            z = np.append(z, logdata[2]) # frame[1]['stateEstimate.y'])

            self.signal1.set_xdata(t)
            self.signal1.set_ydata(x)

            self.signal2.set_xdata(t)
            self.signal2.set_ydata(y)
            
            self.signal3.set_xdata(t)
            self.signal3.set_ydata(z)

            xmin, xmax = self.ax1.get_xlim()

            if frame > xmax:
                self.ax1.set_xlim(xmin, 2*xmax)
                self.ax2.set_xlim(xmin, 2*xmax)
                self.ax3.set_xlim(xmin, 2*xmax)
                
                self.ax1.figure.canvas.draw()
                self.ax2.figure.canvas.draw()
                self.ax3.figure.canvas.draw()

            #self.ax1.clear()
            #self.signal1 = self.ax1.plot(t,x)
            #self.canvas.draw()

            #self.ax1.relim() # recompute the ax.dataLim
            #self.ax2.relim()
            #self.ax3.relim()

            #self.ax1.autoscale()
            #self.ax2.autoscale()
            #self.ax3.autoscale()

            #ax.autoscale()
            #ax.autoscale_view() # update ax.viewLim using the new dataLim
            #xmin, xmax = ax.get_xlim()
            #ymin, ymax = ax.get_ylim()

            """
            if x[-1] >= xmax:
                ax.set_xlim(xmin, 1.3*xmax)
                ax.figure.canvas.draw()

            if x[-1] <= xmin:
                ax.set_xlim(-1.3*xmin, xmax)
                ax.figure.canvas.draw()

            if y[-1] >= ymax:
                ax.set_ylim(ymin, 1.3*ymax)
                ax.figure.canvas.draw()
            
            if y[-1] <= ymin:
                ax.set_ylim(-1.3*ymin, ymax)
                ax.figure.canvas.draw()
            """

            #print("[PlotApp]: ", logdata)
            return self.signal1, self.signal2, self.signal3,
        
        self.ani = animation.FuncAnimation(
            fig=self.figure, 
            func=partial(update), 
            frames=None, 
            init_func=init, 
            event_source=crazyflie_logconf,
            blit=True,
            save_count=100
        )
        plt.show(block=False)

        self.canvas = FigureCanvasTkCairo(figure=self.figure, master=self)
        #self.canvas.draw()
        #self.figure.canvas.draw()
        #self.after(func=self.update_plot, ms=10)
        
        """
        self.canvas.get_tk_widget().grid(row=1, column=1, sticky='nsew')
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
        """
        
        # Toolbar
        # pack_toolbar=False will make it easier to use a layout manager later on.
        #toolbar = NavigationToolbar2Tk(self.canvas, self, pack_toolbar=False)
        #toolbar.update()

        # Pack
        #toolbar.pack(side='bottom', fill='x')
        self.canvas.get_tk_widget().pack(side="top", fill='both', expand=True)

        #self._populate()
        

    def _fig_base(self):
        fig = Figure(figsize=(6, 4), facecolor='lightskyblue', layout='constrained')
        fig.suptitle('stateEstimate')
        #ax = fig.add_axes([0, 0, 1, 1])
        #langs = ['V1','V2']
        #data = [23, 17]
        #ax.bar(langs, data)
        return fig