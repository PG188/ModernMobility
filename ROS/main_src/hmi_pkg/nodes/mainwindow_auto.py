# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.4.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    #self.brkmode = ""
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 492)
        MainWindow.setMinimumSize(QtCore.QSize(800, 492))
        MainWindow.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        self.BrakeModeBtn = QtWidgets.QPushButton(self.centralWidget)
        self.BrakeModeBtn.setGeometry(QtCore.QRect(390, 200, 161, 111))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        self.BrakeModeBtn.setFont(font)
        #self.BrakeModeBtn.setMouseTracking(False)
        #self.BrakeModeBtn.setTabletTracking(False)
        self.BrakeModeBtn.setObjectName("BrakeModeBtn")
        self.FreeRollingBtn = QtWidgets.QPushButton(self.centralWidget)
        self.FreeRollingBtn.setGeometry(QtCore.QRect(180, 200, 151, 111))
        font = QtGui.QFont()
        font.setFamily("Calibri")
        self.FreeRollingBtn.setFont(font)
        #self.FreeRollingBtn.setMouseTracking(False)
        #self.FreeRollingBtn.setTabletTracking(False)
        self.FreeRollingBtn.setObjectName("FreeRollingBtn")
        self.label = QtWidgets.QLabel(self.centralWidget)
        self.label.setGeometry(QtCore.QRect(10, 10, 211, 141))
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap("logoresized.png"))
        self.label.setObjectName("label")

        self.dateEdit = QtWidgets.QDateEdit(self.centralWidget)
        self.dateEdit.setGeometry(QtCore.QRect(680, 50, 110, 22))
        self.dateEdit.setDate(QtCore.QDate.currentDate())
        self.dateEdit.setObjectName("dateEdit")
        self.timeEdit = QtWidgets.QTimeEdit(self.centralWidget)
        self.timeEdit.setGeometry(QtCore.QRect(670, 20, 118, 22))
        self.timeEdit.setObjectName("timeEdit")
        self.timeEdit.setTime(QtCore.QTime.currentTime())
        
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 800, 26))
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
        self.BrakeModeBtn.setText(_translate("MainWindow", "Set Brake Mode"))
        self.FreeRollingBtn.setText(_translate("MainWindow", "Set Free Rolling Mode"))
        self.menuMM.setTitle(_translate("MainWindow", "ModernMobility"))

