#%% Imports

import tkinter as tk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.dates as mdates
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator

import threading
import time
from datetime import datetime

from .AMRBase import FleetManager

#%% Constant Data

RST_list = [
    'DDT',
    'PDT',
    'AT',
    'BT',
    'PT',
    'PedT',
    'DT',
    'FDT',
    'DedT',
    'FDedT',
    'IFT',
    'IET',
    ]

RST_dict = {
    'DDT': 'TripAggDropOffDrivingTime',
    'PDT': 'TripAggPickupDrivingTime',
    'AT': 'TripAggAfterTime',
    'BT': 'TripAggBufferedTime',
    'PT': 'TripAggParkingTime',
    'PedT': 'TripAggParkedTime',
    'DT': 'TripAggDockingTime',
    'FDT': 'TripAggForceDockingTime',
    'DedT': 'TripAggDockedTime',
    'FDedT': 'TripAggForceDockedTime',
    'IFT': 'TripAggInFaultTime',
    'IET': 'TripAggInEstopTime',
    }

RST_colors = {
    'PDT': '#00BFFF', # Pickup Driving
    'DDT': '#1E90FF', # Dropoff Driving
    'BT': '#0000CD', # Buffered
    'AT': '#4169E1', # In After
    'PT': '#32CD32', # Parking
    'PedT': '#228B22', # Parked
    'DT': '#FFFF00', # Docking
    'DedT': '#FFD700', # Docked
    'FDT': '#C0C0C0', # Force Docking
    'FDedT': '#808080', # Force Docked
    'IFT': '#FF0000', # In Fault
    'IET': '#8B0000', # In EStop
    }

RST_labels = {
    'PDT': 'Pickup Driving',
    'DDT': 'Dropoff Driving',
    'BT': 'Buffered',
    'AT': 'In After',
    'PT': 'Parking',
    'PedT': 'Parked',
    'DT': 'Docking',
    'DedT': 'Docked',
    'FDT': 'Force Docking',
    'FDedT': 'Force Docked',
    'IFT': 'In Fault',
    'IET': 'In EStop',
}

RST_altcolors = ['#68CEF6', '#32A429', '#EFC050', '#DD4124'] # Blue Green Yellow Red


FRI_colors = {
    'RA': '#00D200', # Robots Available
    'RIP': '#0096FF', # Robots In Progress
    'RUA': '#FF0000', # Robots Unavailable
}


QI_colors = {
    'QS': '#C0C0C0', # Queue Size
    'QSP': '#00D200', # Pending
    'QSIP': '#0096FF', # In Progress
}

#%% Functions Defintion

def prepare_plot(title='', dimensions=(5, 4), hide_axis=False):
    """Function that prepares an empty :py:class:`matplotlib.plt` plot figure.
    """
    fig = Figure(
        dpi=100,
        figsize=dimensions,
        )

    ax = fig.add_subplot()
    ax.set(aspect="auto", title=title)

    if hide_axis:
        ax.axis('off')

    fig.tight_layout()

    return fig, ax

#%% Main Block

class FleetStatsGUI():
    '''Class to be used to generate a Fleet Dashboard.

    Class used to generate a Fleet Dashboard. The dashboard will contain
    three :py:class:`matplotlib.plt` plots, one being a pie chart of the robot states,
    and two being a real-time plots of the robot states and job queue.

    Attributes:        
        FM (:py:class:`~AMRBase.FleetManager`): The :py:class:`~AMRBase.FleetManager` object
            that hold the robot to which statistics will be calculated

        delta_t (:py:class:`int`): The time interval in seconds at which refresh the data and update the plots.
            **Default**: 20 [s]

        reset_trip (:py:class:`bool`): Parameter that controls whether or not to run the (:py:meth:`~AMRBase.MasterAMR.resetTrip`) method
            before starting to plot the data.
    '''

    def __init__(self, FM, delta_t=20, reset_trip=True):
        if type(FM) is not FleetManager:
            raise Exception('Object provided is not a FleetManager object')

        self.FM = FM
        self.delta_t = delta_t
        self.reset_trip = reset_trip

        self.end_thread = False
        self.plot_paused = False

        self.FRI = {
            'RIP': [],
            'RA': [],
            'RUA': [],
            'TimeStamps': [],
        }
        
        self.QI = {
            'QS': [],
            'QSP': [],
            'QSIP': [],
            'TimeStamps': [],
        }


        self.createwindow()
        

        if self.reset_trip:
            self.FM.tripReset()
        
        self.ds1 = self.FM.datastore['FleetRobotInformation']
        self.ds2 = self.FM.datastore['TripStateInformation']
        self.ds3 = self.FM.datastore['QueueInformation']
        
        self.animate1(0)
        self.animate2(0)
        self.animate3(0)
        

        threading.Thread(target=self.update_datastores).start()
        time.sleep(min(0.5, delta_t/2))
        self.ani1 = animation.FuncAnimation(self.fig1, self.animate1, interval=delta_t*1000, blit=False, cache_frame_data=False)
        self.ani2 = animation.FuncAnimation(self.fig2, self.animate2, interval=delta_t*1000, blit=False, cache_frame_data=False)
        self.ani3 = animation.FuncAnimation(self.fig3, self.animate3, interval=delta_t*1000, blit=False, cache_frame_data=False)
        

        self.root.mainloop()
    
    def createwindow(self):
        self.root = tk.Tk()
        self.root.wm_title('Fleet Statistics')
        # self.root.minsize(1200, 750)
        self.root.configure(background='white')


        self.row0 = tk.Frame(master=self.root)
        self.row0.configure(background='white')

        self.reset_btn = tk.Button(master=self.row0, text='Reset Trip', command=self.reset_data, padx=5, pady=3)
        self.reset_btn.grid(row=0, column=0, padx=10, pady=5)
        
        self.pause_btn = tk.Button(master=self.row0, text='Pause Plot', command=self.pause_plot, padx=5, pady=3)
        self.pause_btn.grid(row=0, column=1, padx=10, pady=5)


        self.row1 = tk.Frame(master=self.root)
        self.row1.columnconfigure(0, weight=1)
        self.row1.columnconfigure(1, weight=1)
        self.row1.rowconfigure(0, weight=1)

        self.fig1, self.ax1 = prepare_plot(title='Robots State Count', dimensions=(6, 3.5))
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=self.row1)  # A tk.DrawingArea.

        self.fig2, self.ax2 = prepare_plot(title='Robots Information', dimensions=(6, 3.5), hide_axis=True)
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=self.row1)  # A tk.DrawingArea.

        self.canvas1.get_tk_widget().grid(row=0, column=0, sticky='nswe')
        self.canvas2.get_tk_widget().grid(row=0, column=1, sticky='nswe')

        
        self.row2 = tk.Frame(master=self.root)
        self.row2.configure(background='white')
        self.row2.columnconfigure(0, weight=1)
        self.row2.columnconfigure(1, weight=1)

        self.subrow2_1 = tk.Frame(master=self.row2)
        self.subrow2_1.configure(background='white')
        self.subrow2_1.grid(row=0, column=0)

        self.subrow2_2 = tk.Frame(master=self.row2)
        self.subrow2_2.configure(background='white')
        self.subrow2_2.grid(row=0, column=1)

        self.toolbar1 = NavigationToolbar2Tk(self.canvas1, self.subrow2_1, pack_toolbar=False)
        self.toolbar1.update()
        self.toolbar1.config(background='white')
        self.toolbar1._message_label.destroy()
        for element in self.toolbar1.winfo_children():
            element.config(background='white')
        self.toolbar1.grid(row=0, column=0)

        self.toolbar2 = NavigationToolbar2Tk(self.canvas2, self.subrow2_2, pack_toolbar=False)
        self.toolbar2.update()
        self.toolbar2.config(background='white')
        self.toolbar2._message_label.destroy()
        for element in self.toolbar2.winfo_children():
            element.config(background='white')
        self.toolbar2.grid(row=0, column=0)


        self.row3 = tk.Frame(master=self.root)
        self.row3.columnconfigure(0, weight=1)
        self.row3.rowconfigure(0, weight=1)

        self.fig3, self.ax3 = prepare_plot(title='Robots State Count', dimensions=(12, 3.5))
        self.canvas3 = FigureCanvasTkAgg(self.fig3, master=self.row3)  # A tk.DrawingArea.

        self.canvas3.get_tk_widget().grid(row=0, column=0, sticky='nswe')


        self.row4 = tk.Frame(master=self.root)
        self.row4.configure(background='white')

        self.toolbar3 = NavigationToolbar2Tk(self.canvas3, self.row4, pack_toolbar=False)
        self.toolbar3.update()
        self.toolbar3.config(background='white')
        self.toolbar3._message_label.destroy()
        for element in self.toolbar3.winfo_children():
            element.config(background='white')
        self.toolbar3.grid(row=0, column=0)


        self.root.rowconfigure(0, weight=0)
        self.root.rowconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=0)
        self.root.rowconfigure(3, weight=1)
        self.root.rowconfigure(4, weight=0)
        self.root.columnconfigure(0, weight=1)

        self.row0.grid(row=0, column=0, sticky='nw', padx=10, pady=5)
        self.row1.grid(row=1, column=0, sticky='nswe')
        self.row2.grid(row=2, column=0, sticky='we')
        self.row3.grid(row=3, column=0, sticky='nswe')
        self.row4.grid(row=4, column=0, pady=(0, 10))

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def reset(self):
        self.FM.tripReset()

        self.QI['QS'] = []
        self.QI['QSP'] = []
        self.QI['QSIP'] = []

        self.FRI['RIP'] = []
        self.FRI['RA'] = []
        self.FRI['RUA'] = []

        self.FRI['TimeStamps'] = []
        self.QI['TimeStamps'] = []

    def reset_data(self):
        threading.Thread(target=self.reset).start()

    def pause_plot(self):
        if not self.plot_paused:
            self.ani1.pause()
            self.ani2.pause()
            self.ani3.pause()

            self.plot_paused = True
            self.pause_btn.configure(text='Resume Plot')
        else:
            self.ani1.resume()
            self.ani2.resume()
            self.ani3.resume()

            self.plot_paused = False
            self.pause_btn.configure(text='Pause Plot')

    def update_datastores(self):
        while True:
            try:
                self.ds1 = self.FM.datastore['FleetRobotInformation']
                self.ds2 = self.FM.datastore['TripStateInformation']
                self.ds3 = self.FM.datastore['QueueInformation']
                time.sleep(self.delta_t)
            except:
                pass

            if self.end_thread:
                break
        
    def on_closing(self):
        self.end_thread = True
        self.root.destroy()

    def animate1(self, i):
        try:
            now = datetime.now()

            self.FRI['TimeStamps'].append(now)

            self.FRI['RIP'].append(self.ds1['RobotsInProgress'])
            self.FRI['RA'].append(self.ds1['RobotsAvail'])
            self.FRI['RUA'].append(self.ds1['RobotsUnAvail'])

            self.ax1.cla()

            self.ax1.plot(self.FRI['TimeStamps'], self.FRI['RIP'], FRI_colors['RIP'], alpha=0.8)
            self.ax1.plot(self.FRI['TimeStamps'], self.FRI['RA'], FRI_colors['RA'], alpha=0.8)
            self.ax1.plot(self.FRI['TimeStamps'], self.FRI['RUA'], FRI_colors['RUA'], alpha=0.8)

            xformatter = mdates.DateFormatter('%H:%M')
            self.ax1.xaxis.set_major_formatter(xformatter)

            self.ax1.yaxis.set_major_locator(MaxNLocator(integer=True))
            
            self.ax1.set_title('Robots State Count')
            self.ax1.legend(['In Progress', 'Available', 'Unavailable'],
                    loc='upper right',
                    )
            
        except:
            self.ax1.cla()
            self.ax1.set_title('Robots State Count')


    def animate2(self, i):
        try:
            labels = []
            colors = []
            sizes = []
            altsizes = [0, 0, 0, 0]

            totaltimes = sum([self.ds2[RST_dict[x]] for x in RST_list])

            for RST in RST_list:
                curtime = self.ds2[RST_dict[RST]]
                sizes.append(curtime)

                colors.append(RST_colors[RST])

                if RST in ['DDT', 'PDT', 'AT', 'BT']:
                    altsizes[0] += self.ds2[RST_dict[RST]]
                elif RST in ['PT', 'PedT']:
                    altsizes[1] += self.ds2[RST_dict[RST]]
                elif RST in ['DT', 'FDT', 'DedT', 'FDedT']:
                    altsizes[2] += self.ds2[RST_dict[RST]]
                elif RST in ['IFT', 'IET']:
                    altsizes[3] += self.ds2[RST_dict[RST]]
                
                percentage = round(curtime/totaltimes*100)
                labels.append(f'{RST_labels[RST]} ({percentage}%)')

            self.ax2.cla()

            size = 0.3
            self.ax2.pie(altsizes, radius=1, colors=RST_altcolors,
                wedgeprops=dict(width=size, edgecolor='w'))
            
            self.ax2.set_title('Robots Information', loc='right')
            self.ax2.set(aspect='equal')

            wedges, _ = self.ax2.pie(sizes, radius=1-size, colors=colors,
                wedgeprops=dict(width=size, edgecolor='w'))

            self.ax2.legend(wedges, labels,
                    loc='center left',
                    bbox_to_anchor=(0.95, 0.5),
                    )
            
            self.fig2.subplots_adjust(right=0.7)

        except:
            self.ax2.cla()
            self.ax2.axis('off')
            self.ax2.set_title('Robots Information', loc='right')


    def animate3(self, i):
        try:
            now = datetime.now()

            self.QI['TimeStamps'].append(now)

            self.QI['QS'].append(self.ds3['QueueSize'])
            self.QI['QSP'].append(self.ds3['QueueSizePending'])
            self.QI['QSIP'].append(self.ds3['QueueSizeInProgress'])

            self.ax3.cla()

            self.ax3.plot(self.QI['TimeStamps'], self.QI['QS'], QI_colors['QS'], alpha=0.8)
            self.ax3.plot(self.QI['TimeStamps'], self.QI['QSP'], QI_colors['QSP'], alpha=0.8)
            self.ax3.plot(self.QI['TimeStamps'], self.QI['QSIP'], QI_colors['QSIP'], alpha=0.8)

            xformatter = mdates.DateFormatter('%H:%M')
            self.ax3.xaxis.set_major_formatter(xformatter)

            self.ax3.yaxis.set_major_locator(MaxNLocator(integer=True))

            self.ax3.set_title('Jobs Queue')
            self.ax3.legend(['Queue Size', 'Pending', 'In Progress'],
                    loc='upper right',
                    )

        except:
            self.ax3.cla()
            self.ax3.set_title('Jobs Queue')

#%%
