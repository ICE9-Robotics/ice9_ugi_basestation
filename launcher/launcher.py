# TODO add dependency
# TODO set hourglass cursor when ros service call is blocking.

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QCursor
import sys
import os
import subprocess
import psutil
import threading
from time import sleep
import ruamel.yaml as yaml
import rospy
import rosservice

import launcher_ui

MASTER_HOSTNAME = "master"
PING_TIMEOUT = 1000
STATUS_TIMEOUT = 5

def getBasePath():
    config = open('/home/ice9/.config/ice9_basestation/setup.bash', 'r')
    lines = config.readlines()
    for line in lines:
        if 'export BASESTATION_DIR=' in line:
            return line.split('=')[1].strip()

def getPid(pname):
    try:
        for process in psutil.process_iter():
            try:
                if pname.lower() in process.name().lower():
                    return process.pid
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return None
    except Exception as e:
        print("An error occurred: {}".format(e))
        return None

def getMapvizConfig():
    config_path = os.path.join(getBasePath(), 'basestation_ws', 'src', 'ice9_unitree_basestation', 'config', 'unitree.local.mvc')
    if not os.path.exists(config_path):
        return None
    with open(config_path, 'r') as stream:
        try:
            yaml.preserve_quotes = True
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

def saveMapvizConfig(config):
    config_path = os.path.join(getBasePath(), 'basestation_ws', 'src', 'ice9_unitree_basestation', 'config', 'unitree.local.mvc')
    #back up config
    if os.path.exists(config_path):
        os.rename(config_path, config_path + '.bak')
    try:
        with open(config_path, 'w') as stream:
            try:
                yaml.safe_dump(config, stream, default_flow_style=False, allow_unicode=True)
            except yaml.YAMLError as exc:
                print(exc)
    except Exception as e:
        #restore config
        if os.path.exists(config_path + '.bak'):
            os.rename(config_path + '.bak', config_path)
        raise e

def displayErrorBox(msg):
    errBox = QtWidgets.QMessageBox()
    errBox.setIcon(QtWidgets.QMessageBox.Critical)
    errBox.setText(msg)
    errBox.setWindowTitle("Error")
    errBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
    errBox.exec_()

def getMasterState():
    return rospy.get_master().getSystemState()[0]

def pingMaster():
    try:
        result = subprocess.check_output(['fping', '-6', MASTER_HOSTNAME, '-e', '-C', '1', '-t', str(PING_TIMEOUT)])
        result = result.decode('utf-8')
        start = result.find("(")
        end = result.find(" avg", start)
        if "master" in result and "avg" in result:
            return float(result[start + 1:end])
        else:
            return None
    except subprocess.CalledProcessError as e:
        return None
    except Exception as e:
        raise e

class RosHandler():
    def __init__(self, rosStateEventhandler=None):
        self.isRosOk = False
        self.masterPing = None
        self.rosStateEventhandler = rosStateEventhandler
        self.rosOkThreadStopEvent = threading.Event()
        # self.rosPingThread = threading.Thread(target=self.rosPingThreadCallback)
        # self.rosPingThread.start()
        self.rosOkThread = threading.Thread(target=self.rosOkThreadCallback)
        self.rosOkThread.start()

    def __del__(self):
        self.rosOkThreadStopEvent.set()

    def findServiceName(self, service):
        if not self.isRosOk:
            return None
        try:
            services = rosservice.get_service_list()
            for s in services:
                if service in s:
                    return s.split('/')[1]
        except rospy.service.ServiceException as e:
            return None
    
    def callService(self, service, req):
        if not self.isRosOk:
            return None
        try:
            p = subprocess.Popen(['rosservice', 'call', service, req], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            res, err = p.communicate()
            if p.returncode != 0:
                return False, err
            else:
                return True, res
        except Exception as e:
            return False, e
        
    def rosPingThreadCallback(self):
        while not self.rosOkThreadStopEvent.is_set():
            try:
                self.masterPing = pingMaster()
                self.rosStateEventhandler("master_ping")
            except Exception as e:
                print("Waiting for master: ", e)
            finally:
                self.rosStateEventhandler("master_ping")
            sleep(.1)
    
    def rosOkThreadCallback(self):
        while not self.rosOkThreadStopEvent.is_set():
            try:
                self.masterPing = pingMaster()
                self.rosStateEventhandler("master_ping")
                if not self.masterPing:
                    self.isRosOk = False
                    self.rosStateEventhandler("master_state")
                    continue
                state = getMasterState()
                if state:
                    self.isRosOk = True
                else:
                    self.isRosOk = False
            except Exception as e:
                self.isRosOk = False
                print("Waiting for ROS: ", e)
            finally:
                self.rosStateEventhandler("master_state")
            sleep(1)

class Launcher(QtWidgets.QMainWindow, launcher_ui.Ui_MainWindow):
    def __init__(self, parent=None):
        # QT init
        super(Launcher, self).__init__(parent)
        self.setupUi(self)
        self.connectUIControls()

        # Variables
        self.mapvizServiceName = ""
        self.rosHandler = RosHandler(self.rosOkEventHandler)
        self.displayToggles = [ 
                { 'obj': self.checkBoxShowSatMap, 'registered_state': self.checkBoxShowSatMap.isChecked(), 'type': 'mapviz_plugins/tile_map', 'name': 'Sat Map' },
                { 'obj': self.checkBoxShowSlamMap, 'registered_state': self.checkBoxShowSlamMap.isChecked(), 'type': 'mapviz_plugins/occupancy_grid', 'name': 'SLAM Map' },
                { 'obj': self.checkBoxShowCam, 'registered_state': self.checkBoxShowCam.isChecked(), 'type': 'mapviz_plugins/image', 'name': 'Robot Camera' },
                { 'obj': self.checkBoxShowUnitreeImg, 'registered_state': self.checkBoxShowUnitreeImg.isChecked(), 'type': 'mapviz_plugins/robot_image', 'name': 'Robot Image' },
                { 'obj': self.checkBoxShowTraj, 'registered_state': self.checkBoxShowTraj.isChecked(), 'type': 'mapviz_plugins/odometry', 'name': 'Wheel Odom' },
                { 'obj': self.checkBoxShowLidarScan, 'registered_state': self.checkBoxShowLidarScan.isChecked(), 'type': 'mapviz_plugins/laserscan', 'name': 'LiDar Points' },
                { 'obj': self.checkBoxShowObstacle, 'registered_state': self.checkBoxShowObstacle.isChecked(), 'type': 'mapviz_plugins/occupancy_grid', 'name': 'Local Obstacle' }
            ]
        
        # Threads
        self.statusClearTimer = None

        # Initialise UI elements
        self.mapvizConfig = getMapvizConfig()
        self.setUIFromConfig()

        self.ping = 0
        self.upload = 0
        self.download = 0

    def closeEvent(self, event):
        print("Cleaning up...")
        self.rosHandler.__del__()
        if self.statusClearTimer and self.statusClearTimer.isAlive():
            self.statusClearTimer.cancel()

    def connectUIControls(self):
        self.pushButtonMapvizLaunch.clicked.connect(self.mapvizLaunchClickedCallback)

        self.pushButtonMapvizSave.clicked.connect(self.mapvizSaveCallback)
        self.pushButtonMapvizApply.clicked.connect(self.mapvizApplyCallback)

    def setUIFromConfig(self):
        if not self.mapvizConfig:
            displayErrorBox("Error getting Mapviz config. Config file may not exist.")
            return
        try:
            i = 0
            for display in self.mapvizConfig['displays']:
                for toggle in self.displayToggles:
                    if display['name'] == toggle['name']:
                        toggle['obj'].setChecked(display['config']['visible'])
                        i += 1
                        break
                if display['name'] == 'Sat Map':
                    self.lineEditBingAPI.setText(display['config']['bing_api_key'])
                if i >= len(self.displayToggles):
                    break
        except Exception as e:
            displayErrorBox("Error getting Mapviz config: %s" % e)
            return
        
        if i != len(self.displayToggles):
            displayErrorBox("Error getting Mapviz config. Config file may be corrupted. Settings incomplete.")
       
    def toggleMapvizDisplayVisibility(self, type, name, on):
        QtWidgets.QApplication.setOverrideCursor(
            QCursor(QtCore.Qt.WaitCursor))
        try:
            suc, msg = self.toggleMapvizDisplayVisibility_(type, name, on)
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self.mapvizServiceAttempt = 0
        if not suc:
            self.setStatusBarError(msg)
        else:
            self.setStatusBarInfo("Success")
        return suc

    def toggleMapvizDisplayVisibility_(self, type, name, on):
        if not self.mapvizServiceName:
            self.mapvizServiceName = self.rosHandler.findServiceName('mapviz')
            if not self.mapvizServiceName:
                return False, "Error getting Mapviz service name"

        req = "name: '{name}'\ntype: '{type}'\nvisible: {on}".format(name=name, type=type, on=on)
        res = self.rosHandler.callService(self.mapvizServiceName + "/add_mapviz_display", req)
        if not res:
            return False, "Error communicating with ROS"
        if res[0] == True:
            return True, ""
        else:
            # Maybe mapviz service name has changed, retry once.
            if self.mapvizServiceAttempt < 1:
                self.mapvizServiceAttempt += 1
                self.mapvizServiceName = ""
                return self.toggleMapvizDisplayVisibility_(type, name, on)
            return False, "Failed: %s" % res[1]
 
    def rosOkEventHandler(self, event):
        try:
            if event == "master_ping":
                if self.rosHandler.masterPing:
                    self.labelMasterPing.setText("{:.0f} ms".format(self.rosHandler.masterPing))
                else:
                    self.labelMasterPing.setText("> %d ms" % PING_TIMEOUT)
            elif event == "master_state":
                if self.rosHandler.isRosOk:
                    self.labelMasterState.setText("OK")
                    self.labelMasterState.setStyleSheet("color: green")
                    self.pushButtonMapvizLaunch.setEnabled(True)
                else:
                    self.labelMasterState.setText("Unavailable")
                    self.labelMasterState.setStyleSheet("color: red") 
                    self.pushButtonMapvizLaunch.setEnabled(False)
        except Exception as e:
            print("Error updating UI: ", e)

    ################
    ## Status bar ##
    ################

    def startStatusClearTimer(self):
        if self.statusClearTimer and self.statusClearTimer.isAlive():
            self.statusClearTimer.cancel()
        self.statusClearTimer = threading.Timer(STATUS_TIMEOUT, self.setStatusBarInfo, [""])
        self.statusClearTimer.start()

    def setStatusBarInfo(self, msg):
        self.statusbar.showMessage(msg)
        self.statusbar.setStyleSheet("color: black")
        self.startStatusClearTimer()

    def setStatusBarWarning(self, msg):
        self.statusbar.showMessage(msg)
        self.statusbar.setStyleSheet("color: orange")
        self.startStatusClearTimer()

    def setStatusBarError(self, msg):
        self.statusbar.showMessage(msg)
        self.statusbar.setStyleSheet("color: red")
        self.startStatusClearTimer()

    ##################
    ## UI Callbacks ##
    ##################

    def mapvizLaunchClickedCallback(self):
        if not getPid('basestation.desktop.sh'):
            base_path = getBasePath()
            subprocess.Popen(['gnome-terminal', '--', os.path.join(base_path, 'install', 'files', 'basestation.desktop.sh')])
        else:
            self.setStatusBarInfo("Mapviz already running")

    def mapvizSafeQuitClickedCallback(self):
        pid = getPid('basestation.des')
        if pid:
            psutil.Process(pid).terminate()
            self.setStatusBarInfo("Safe quit requested")
        else:
            self.setStatusBarWarning("Mapviz not running")

    def mapvizForceQuitClickedCallback(self):
        pid = getPid('basestation.des')
        if pid:
            psutil.Process(pid).kill()
            self.setStatusBarInfo("Force quit requested")
        else:
            self.setStatusBarWarning("Mapviz not running")

    def mapvizSaveCallback(self):
        if not self.mapvizConfig:
            displayErrorBox("Error getting Mapviz config. Config file may not exist.")
            return
        try:
            i = 0
            for display in self.mapvizConfig['displays']:
                for toggle in self.displayToggles:
                    if display['name'] == toggle['name']:
                        display['config']['visible'] = toggle['obj'].isChecked()
                        i += 1
                        break
                
                if display['name'] == 'Sat Map':
                    display['config']['bing_api_key'] = self.lineEditBingAPI.text().encode('utf-8')
                if i >= len(self.displayToggles):
                    break
        except Exception as e:
            displayErrorBox("Aborted. Error getting Mapviz config: %s" % e)
            return
        if i != len(self.displayToggles):
            displayErrorBox("Aborted. Mapviz config file is ill formated.")
            return
        
        saveMapvizConfig(self.mapvizConfig)
        self.setStatusBarInfo("Success")

    def mapvizApplyCallback(self):
        for toggle in self.displayToggles:
            if toggle['obj'].isChecked() == toggle['registered_state']:
                continue
            if (self.toggleMapvizDisplayVisibility(toggle['type'], toggle['name'], toggle['obj'].isChecked())):
                toggle['registered_state'] = toggle['obj'].isChecked()

def main():
    app = QApplication(sys.argv)
    form = Launcher()
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()