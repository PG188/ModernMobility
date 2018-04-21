# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.4.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(600, 400)
        MainWindow.setMinimumSize(QtCore.QSize(600, 400))
        MainWindow.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        
        #####Assist button stuff below#####
        self.AssistModeBtn = QtWidgets.QPushButton(self.centralWidget)
        self.AssistModeBtn.setGeometry(QtCore.QRect(325, 150, 200, 100))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        self.AssistModeBtn.setFont(font)
        #self.AssistModeBtn.setMouseTracking(False)
        #self.AssistModeBtn.setTabletTracking(False)
        self.AssistModeBtn.setObjectName("AssistModeBtn")
        
        #####Free rolling button stuff below#####
        self.FreeRollingBtn = QtWidgets.QPushButton(self.centralWidget)
        self.FreeRollingBtn.setGeometry(QtCore.QRect(75, 150, 200, 100))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        self.FreeRollingBtn.setFont(font)
        #self.FreeRollingBtn.setMouseTracking(False)
        #self.FreeRollingBtn.setTabletTracking(False)
        self.FreeRollingBtn.setObjectName("FreeRollingBtn")
        
        #####Disconnect button stuff below#####
        self.DisconnectBtn = QtWidgets.QPushButton(self.centralWidget)
        self.DisconnectBtn.setGeometry(QtCore.QRect(75, 275, 450, 50))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        self.DisconnectBtn.setFont(font)
        #self.DisconnectBtn.setMouseTracking(False)
        #self.DisconnectBtn.setTabletTracking(False)
        self.DisconnectBtn.setObjectName("DisconnectBtn")
        
        self.label = QtWidgets.QLabel(self.centralWidget)
        self.label.setGeometry(QtCore.QRect(10, 10, 211, 141))
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap("logoresized.png"))
        self.label.setObjectName("label")
        
        self.dateEdit = QtWidgets.QDateEdit(self.centralWidget)
        self.dateEdit.setGeometry(QtCore.QRect(480, 50, 110, 22))
        self.dateEdit.setDate(QtCore.QDate.currentDate())
        self.dateEdit.setObjectName("dateEdit")
        self.timeEdit = QtWidgets.QTimeEdit(self.centralWidget)
        self.timeEdit.setGeometry(QtCore.QRect(470, 20, 118, 22))
        self.timeEdit.setObjectName("timeEdit")
        self.timeEdit.setTime(QtCore.QTime.currentTime())
        
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 600, 26))
        self.menuBar.setObjectName("menuBar")
        self.menuMM = QtWidgets.QMenu(self.menuBar)
        self.menuMM.setObjectName("menuMM")
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtWidgets.QToolBar(MainWindow)
        self.mainToolBar.setObjectName("mainToolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)
        self.menuBar.addAction(self.menuMM.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.AssistModeBtn.setText(_translate("MainWindow", "Set Assist Mode"))
        self.FreeRollingBtn.setText(_translate("MainWindow", "Set Free Rolling Mode"))
        self.DisconnectBtn.setText(_translate("MainWindow", "Disconnect (Disconnect phone first)"))
        self.menuMM.setTitle(_translate("MainWindow", "ModernMobility"))

