from PySide6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import numpy as np
import random
import math




class PlotManager:
    def __init__(self, plot_window_1: pg.GraphicsLayoutWidget, plot_window_2: pg.GraphicsLayoutWidget):
        self.plot_win_0 = plot_window_1
        self.plot_win_1 = plot_window_2


        self.v_data_0 = {
                           "time": [],
                           "volts": []}

        self.i_data_0 = {
                           "time": [],
                           "amps": []}

        self.i_data_1 = {
                           "time": [],
                           "amps": []}

        self.plot_win_0.setBackground(QtGui.QColor('#FFFFFF'))
        self.plot_win_1.setBackground(QtGui.QColor('#FFFFFF'))
        
        self.plot_V = self.plot_win_0.addPlot(row=0, col=0, title="RE Voltage")
        self.plot_V.addLegend()
        self.plot_V.getAxis('left').enableAutoSIPrefix(True)
        self.plot_V.setLabel('left', 'Voltage', units='V')
        self.plot_V_curve_0 = self.plot_V.plot([],
                                                [],
                                                pen='r',
                                                symbol='x')
        
        self.plot_I = self.plot_win_0.addPlot(row=1, col=0, title="WE Current")
        self.plot_I.addLegend()
        self.plot_I.getAxis('left').enableAutoSIPrefix(True)
        self.plot_I.setLabel('left', 'Current', units='A')
        self.plot_I_curve_0 = self.plot_I.plot([],
                                                [],
                                                pen='r',
                                                symbol='x')
        self.plot_I_curve_1 = self.plot_I.plot([],
                                                [],
                                                pen='g',
                                                symbol='x')

        self.colours = ['b', 'g', 'r', 'c', 'm', 'y', 'k']


    def update_data(self, msg):
        _ = msg


    def update_plot(self):
        self.plot_V_curve_0.setData(self.v_data_0["time"], self.v_data_0["volts"])
        self.plot_I_curve_0.setData(self.i_data_0["time"], self.i_data_0["amps"])
        self.plot_I_curve_1.setData(self.i_data_1["time"], self.i_data_1["amps"])

    def reset_plots(self):
        print("reset plots")
        self.v_data_0 = {
                           "time": [],
                           "volts": []}

        self.i_data_0 = {
                           "time": [],
                           "amps": []}

        self.i_data_1 = {
                           "time": [],
                           "amps": []}
