# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 18:34:01 2024

@author: Florencia D. Choque
Towards laser control
"""

import numpy as np
import time
from datetime import date, datetime
import os
import sys
import matplotlib.pyplot as plt
from matplotlib import cm
import tools.tools as tools
import ctypes as ct
from PIL import Image
from tkinter import Tk, filedialog
import tifffile as tiff
import scipy.optimize as opt
from scipy.signal import find_peaks


from threading import Thread

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import qdarkstyle

from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QTabWidget, QGroupBox
from PyQt5 import QtTest
import tools.PSF as PSF

import drivers.ADwin as ADwin
import tools.viewbox_tools as viewbox_tools
import tools.colormaps as cmaps
from tools.lineprofile import linePlotWidget

from drivers.minilasevo import MiniLasEvo

π = np.pi
   
class Frontend(QtGui.QFrame):
    
    paramSignal = pyqtSignal(dict)
    closeSignal = pyqtSignal()

    def __init__(self, *args, **kwargs):

        super().__init__(*args, **kwargs)
        self.initialDir = r'C:\Data'
        
        # Define status icons dir
        self.ICON_RED_LED = 'icons\led-red-on.png'
        self.ICON_GREEN_LED = 'icons\green-led-on.png'
        
        # set up GUI
        self.setup_gui()
    
    def emit_param(self):
        
        params = dict()        
        self.paramSignal.emit(params)
        
    @pyqtSlot(dict)
    def get_backend_param(self, params):
        pass
     
    @pyqtSlot(bool)    
    def update_led(self, emission):
        if emission:
            led = self.ICON_GREEN_LED
        else:
            led = self.ICON_RED_LED
            
        self.diodeemissionStatus.setPixmap(QtGui.QPixmap(led))
        self.diodeemissionStatus.setScaledContents(True)
        self.diodeemissionStatus.setFixedSize(20, 20)
        
        #whenever diodelaser status is changed,
        #the output power will be set to 0 mW for security reasons
        self.diodepowerSpinBox.setValue(0)
            
    def setup_gui(self):
        
        # diodelaser widget
        diodelaserWidget = QGroupBox('Diodelaser control')
        diodelaserWidget.setFixedHeight(108)   
        
        self.diodelaserButton = QtGui.QPushButton('Laser On')
        self.diodelaserButton.setCheckable(True)
        self.diodeemissionLabel = QtGui.QLabel('Emission')
        
        self.diodeemissionStatus = QtGui.QLabel()
        self.diodeemissionStatus.setPixmap(QtGui.QPixmap(self.ICON_RED_LED))
        self.diodeemissionStatus.setScaledContents(True)
        self.diodeemissionStatus.setFixedSize(20, 20)
        
        self.diodepowerLabel = QtGui.QLabel('Power [mW]')
        self.diodepowerSpinBox = QtGui.QSpinBox()
        self.diodepowerSpinBox.setRange(0, 78) #max value given by manual  
        
        self.diodeShutter = QtGui.QCheckBox('Open')
        
        diode_subgrid = QtGui.QGridLayout()
        diodelaserWidget.setLayout(diode_subgrid)
        
        diode_subgrid.addWidget(self.diodelaserButton, 0, 0)
        diode_subgrid.addWidget(self.diodeShutter, 0, 1)
        diode_subgrid.addWidget(self.diodeemissionLabel, 1, 0)
        diode_subgrid.addWidget(self.diodeemissionStatus, 1, 1)
        diode_subgrid.addWidget(self.diodepowerLabel, 2, 0)
        diode_subgrid.addWidget(self.diodepowerSpinBox, 2, 1)
        
        lowerWidget = QtGui.QFrame()
        lower_subgrid = QtGui.QGridLayout()
        lowerWidget.setLayout(lower_subgrid)
        lowerWidget.setMinimumWidth(450)
        # lower_subgrid.addWidget(self.fileWidget, 0, 0)
        lower_subgrid.addWidget(diodelaserWidget, 0, 0)
      
        # scan GUI layout
        grid = QtGui.QGridLayout()
        self.setLayout(grid)
        
        tabArea = QTabWidget()
        grid.addWidget(tabArea, 0, 0)
        
        paramTab = QtGui.QFrame()
        paramTabGrid = QtGui.QGridLayout()
        # paramTabGrid.addWidget(self.paramWidget, 0, 0)
        paramTabGrid.addWidget(lowerWidget, 1, 0)
        paramTab.setLayout(paramTabGrid)
            
    def make_connection(self, backend):
        
        backend.paramSignal.connect(self.get_backend_param)
        backend.diodelaserEmissionSignal.connect(self.update_led)
        
    def closeEvent(self, *args, **kwargs):

        # Emit close signal
        self.closeSignal.emit()
        scanThread.exit()
        super().closeEvent(*args, **kwargs)
        app.quit()
    
      
class Backend(QtCore.QObject):
    
    paramSignal = pyqtSignal(dict)
    diodelaserEmissionSignal = pyqtSignal(bool)
    
    """
    Signals
    
    - paramSignal:
         To: [frontend]
         Description: 
        
    """
    
    def __init__(self, diodelaser, *args, **kwargs):
        
        super().__init__(*args, **kwargs)
        
        self.diodelas = diodelaser
        self.laserstate = False        
        # initial director
        self.initialDir = r'C:\Data'
        
    @pyqtSlot(dict)
    def get_frontend_param(self, params):      
        self.calculate_derived_param()
          
    def calculate_derived_param(self):
        #TODO: check whether we can delete this. 
        self.viewtimer_time = 0  # timer will timeout as soon after it has executed all functions
    
    def emit_param(self):
        params = dict()
        self.paramSignal.emit(params)
        
    def enableDiodelaser(self, enable):
        
        diodelasID = self.diodelas.idn()
        
        if enable:
            self.laserstate = True
            self.diodelas.enabled = True
            print(datetime.now(), '[scan] Diodelaser started')
            print(datetime.now(), '[scan] Diodelaser-ID:', diodelasID)
            
            time.sleep(4) #according to manual, lasing will start 5s after turning-on
            ans1, ans2 = self.diodelas.status()
            i = 0
            while (ans2 != 'Laser system is active, radiation can be emitted'):
                time.sleep(0.5) #check every 0.5s whether emission started
                ans1, ans2 = self.diodelas.status()
                if i>=12:
                    break #interrupt loop after 10s of waiting for emission, preventing to find no end
                i += 1
                
            if i<12:
                self.diodelaserEmissionSignal.emit(True)
                print(datetime.now(), '[scan] Diodelaser emitting!')
            else:
                print(datetime.now(), '[scan] Diodelaser not able to emit radiation. Check status!')
            
        else:
            self.laserstate = False
            self.setpowerDiodelaser(0)
            self.diodelas.enabled = False
            self.diodelaserEmissionSignal.emit(False)
            print(datetime.now(), '[scan] Diodelaser disabled')
    def update_view(self):
        """Procesa click del timer."""
        self.viewtimer.start(self.viewtimer_time)
    
    def setpowerDiodelaser(self, value):
        if self.diodelas.enabled:
            self.diodelas.power = value
            print(datetime.now(), '[scan] Power of diodelaser set to', str(value), 'mW')
                      
    def make_connection(self, frontend): 
        frontend.paramSignal.connect(self.get_frontend_param)
        print("done1")
        frontend.diodelaserButton.clicked.connect(lambda: self.enableDiodelaser(frontend.diodelaserButton.isChecked()))
        frontend.diodepowerSpinBox.valueChanged.connect(lambda: self.setpowerDiodelaser(frontend.diodepowerSpinBox.value()))
        frontend.closeSignal.connect(self.stop)
        print("donde 2")
    
    def stop(self):
        if self.laserstate:
            self.enableDiodelaser(False)
        self.diodelas.closeLaserPort()
        print(datetime.now(), '[scan] Serial port of diode laser closed')
        

if __name__ == '__main__':

    if not QtGui.QApplication.instance():
        app = QtGui.QApplication([])
    else:
        app = QtGui.QApplication.instance()
        
    #app.setStyle(QtGui.QStyleFactory.create('fusion'))
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
        
    #initialize devices
    miniLasEvoPort, miniLambdaPort = tools.get_MiniLasEvoPort()
    diodelaser = MiniLasEvo(miniLasEvoPort)
    diodelaser_830 = MiniLasEvo(miniLambdaPort)
    print('[scan] MiniLasEvo diode laser ports:', miniLasEvoPort, miniLambdaPort)
    worker = Backend(diodelaser)    
    gui = Frontend()
    
    worker.make_connection(gui)
    gui.make_connection(worker)
    
    gui.emit_param()
    worker.emit_param()
    
    laserThread = QtCore.QThread()
    worker.moveToThread(laserThread)
    worker.viewtimer.moveToThread(laserThread)
    worker.viewtimer.timeout.connect(worker.update_view)
    
    laserThread.start()

    gui.setWindowTitle('Laser Control')
    gui.show()
    app.exec_()
