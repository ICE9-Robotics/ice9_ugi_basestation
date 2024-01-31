# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/launcher.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(384, 482)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/res/ice9_logo.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setStyleSheet("background-color: rgb(255,255,255);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet("")
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(0, 0, 382, 460))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout.setObjectName("verticalLayout")
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem1 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setContentsMargins(0, -1, -1, -1)
        self.gridLayout_3.setObjectName("gridLayout_3")
        spacerItem2 = QtWidgets.QSpacerItem(20, 5, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        self.gridLayout_3.addItem(spacerItem2, 1, 0, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_13.setFont(font)
        self.label_13.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_13.setObjectName("label_13")
        self.gridLayout_3.addWidget(self.label_13, 0, 0, 1, 1)
        self.labelMasterPing = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.labelMasterPing.setMinimumSize(QtCore.QSize(65, 0))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.labelMasterPing.setFont(font)
        self.labelMasterPing.setObjectName("labelMasterPing")
        self.gridLayout_3.addWidget(self.labelMasterPing, 0, 1, 1, 1)
        spacerItem3 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem3, 0, 2, 2, 1)
        self.horizontalLayout.addLayout(self.gridLayout_3)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem4)
        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_3.setMaximumSize(QtCore.QSize(195, 50))
        self.label_3.setTextFormat(QtCore.Qt.RichText)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout.addWidget(self.label_3)
        self.verticalLayout.addLayout(self.horizontalLayout)
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem5)
        self.groupBox_2 = QtWidgets.QGroupBox(self.verticalLayoutWidget)
        self.groupBox_2.setMinimumSize(QtCore.QSize(0, 370))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_2.setFont(font)
        self.groupBox_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_2)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(10, 30, 321, 331))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(5, 5, 5, 5)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.horizontalLayout_2.setContentsMargins(-1, 10, -1, 10)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.pushButtonMapvizLaunch = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.pushButtonMapvizLaunch.setEnabled(True)
        self.pushButtonMapvizLaunch.setMinimumSize(QtCore.QSize(95, 0))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.pushButtonMapvizLaunch.setFont(font)
        self.pushButtonMapvizLaunch.setObjectName("pushButtonMapvizLaunch")
        self.horizontalLayout_2.addWidget(self.pushButtonMapvizLaunch)
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem6)
        self.gridLayout_2.addLayout(self.horizontalLayout_2, 0, 0, 1, 2)
        self.checkBoxShowObstacle = QtWidgets.QCheckBox(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.checkBoxShowObstacle.setFont(font)
        self.checkBoxShowObstacle.setObjectName("checkBoxShowObstacle")
        self.gridLayout_2.addWidget(self.checkBoxShowObstacle, 9, 1, 1, 1)
        self.checkBoxShowUnitreeImg = QtWidgets.QCheckBox(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.checkBoxShowUnitreeImg.setFont(font)
        self.checkBoxShowUnitreeImg.setObjectName("checkBoxShowUnitreeImg")
        self.gridLayout_2.addWidget(self.checkBoxShowUnitreeImg, 7, 1, 1, 1)
        self.checkBoxShowCam = QtWidgets.QCheckBox(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.checkBoxShowCam.setFont(font)
        self.checkBoxShowCam.setObjectName("checkBoxShowCam")
        self.gridLayout_2.addWidget(self.checkBoxShowCam, 9, 0, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_10.setMaximumSize(QtCore.QSize(16777215, 25))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        self.gridLayout_2.addWidget(self.label_10, 4, 0, 1, 1)
        self.checkBoxShowTraj = QtWidgets.QCheckBox(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.checkBoxShowTraj.setFont(font)
        self.checkBoxShowTraj.setObjectName("checkBoxShowTraj")
        self.gridLayout_2.addWidget(self.checkBoxShowTraj, 8, 1, 1, 1)
        self.checkBoxShowSatMap = QtWidgets.QCheckBox(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.checkBoxShowSatMap.setFont(font)
        self.checkBoxShowSatMap.setObjectName("checkBoxShowSatMap")
        self.gridLayout_2.addWidget(self.checkBoxShowSatMap, 7, 0, 1, 1)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_9 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_3.addWidget(self.label_9)
        self.lineEditBingAPI = QtWidgets.QLineEdit(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.lineEditBingAPI.setFont(font)
        self.lineEditBingAPI.setObjectName("lineEditBingAPI")
        self.horizontalLayout_3.addWidget(self.lineEditBingAPI)
        self.gridLayout_2.addLayout(self.horizontalLayout_3, 6, 0, 1, 2)
        self.checkBoxShowSlamMap = QtWidgets.QCheckBox(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.checkBoxShowSlamMap.setFont(font)
        self.checkBoxShowSlamMap.setObjectName("checkBoxShowSlamMap")
        self.gridLayout_2.addWidget(self.checkBoxShowSlamMap, 8, 0, 1, 1)
        self.line = QtWidgets.QFrame(self.gridLayoutWidget_2)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.gridLayout_2.addWidget(self.line, 2, 0, 1, 2)
        self.checkBoxShowLidarScan = QtWidgets.QCheckBox(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.checkBoxShowLidarScan.setFont(font)
        self.checkBoxShowLidarScan.setObjectName("checkBoxShowLidarScan")
        self.gridLayout_2.addWidget(self.checkBoxShowLidarScan, 10, 0, 1, 1)
        self.pushButtonMapvizSave = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.pushButtonMapvizSave.setMaximumSize(QtCore.QSize(150, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.pushButtonMapvizSave.setFont(font)
        self.pushButtonMapvizSave.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.pushButtonMapvizSave.setObjectName("pushButtonMapvizSave")
        self.gridLayout_2.addWidget(self.pushButtonMapvizSave, 13, 0, 1, 1)
        spacerItem7 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        self.gridLayout_2.addItem(spacerItem7, 11, 0, 1, 1)
        self.pushButtonMapvizApply = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.pushButtonMapvizApply.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.pushButtonMapvizApply.setFont(font)
        self.pushButtonMapvizApply.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.pushButtonMapvizApply.setObjectName("pushButtonMapvizApply")
        self.gridLayout_2.addWidget(self.pushButtonMapvizApply, 13, 1, 1, 1)
        spacerItem8 = QtWidgets.QSpacerItem(20, 5, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.gridLayout_2.addItem(spacerItem8, 5, 0, 1, 1)
        spacerItem9 = QtWidgets.QSpacerItem(20, 5, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.gridLayout_2.addItem(spacerItem9, 1, 0, 1, 1)
        self.verticalLayout.addWidget(self.groupBox_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.statusbar.setFont(font)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "UGI Launcher"))
        self.label_13.setText(_translate("MainWindow", "Unitree latency:"))
        self.labelMasterPing.setText(_translate("MainWindow", "> 1000 ms"))
        self.label_3.setText(_translate("MainWindow", "<html><head/><body><p><img src=\":/res/ice9_logo_full.png\" height=\"50\"/></p></body></html>"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Mapviz"))
        self.pushButtonMapvizLaunch.setText(_translate("MainWindow", "Launch"))
        self.checkBoxShowObstacle.setText(_translate("MainWindow", "Show Obstacles"))
        self.checkBoxShowUnitreeImg.setText(_translate("MainWindow", "Show Robot Image"))
        self.checkBoxShowCam.setText(_translate("MainWindow", "Show Camera View"))
        self.label_10.setText(_translate("MainWindow", "Configurations"))
        self.checkBoxShowTraj.setText(_translate("MainWindow", "Show Trajectory"))
        self.checkBoxShowSatMap.setText(_translate("MainWindow", "Show Satellite Map"))
        self.label_9.setText(_translate("MainWindow", "Bing Map API:"))
        self.checkBoxShowSlamMap.setText(_translate("MainWindow", "Show SLAM Map"))
        self.checkBoxShowLidarScan.setText(_translate("MainWindow", "Show Lidar Scan"))
        self.pushButtonMapvizSave.setText(_translate("MainWindow", "Save as Defaults"))
        self.pushButtonMapvizApply.setText(_translate("MainWindow", "Apply"))

import resources_rc
