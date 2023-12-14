# TODO network delay test
# TODO speed test for ros
# TODO add dependency
# TODO detect ros master so ros service call won't be blocking.
# TODO set hourglass cursor when ros service call is blocking.

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication
import sys
import os
import subprocess
import psutil
import threading
from time import sleep
import yaml
from speedtest import Speedtest, ConfigRetrievalError
import rospy
import rosservice
from std_srvs.srv import Trigger

import launcher_ui

MASTER_HOSTNAME = "master"
PING_TIMEOUT = 5
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
                yaml.dump(config, stream)
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
        result = subprocess.check_output(['ping', '-6', '-c', '1', '-W', str(PING_TIMEOUT), MASTER_HOSTNAME])
        result = result.decode('utf-8')
        start = result.find("time=")
        end = result.find(" ms", start)

        if "icmp_seq" in result and "time=" in result:
            return float(result[start + 5:end])
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
                    return s
        except rospy.service.ServiceException as e:
            return None
    
    def callService(self, service, req):
        if not self.isRosOk:
            return None
        return rospy.ServiceProxy(service, req).call()
    
    def rosOkThreadCallback(self):
        while not self.rosOkThreadStopEvent.is_set():
            try:
                self.masterPing = pingMaster()
                self.rosStateEventhandler("master_ping")
                if not self.masterPing:
                    self.isRosOk = False
                    continue
                state = getMasterState()
                if state:
                    self.isRosOk = True
                else:
                    self.isRosOk = False
            except Exception as e:
                self.isRosOk = False
                print("Error: %s", e)
            finally:
                self.rosStateEventhandler("master_state")
            sleep(1)

class SpeedtestThread(QtCore.QThread):
    report_ready = QtCore.pyqtSignal(object)

    def __init__(self, parent=None):
        QtCore.QThread.__init__(self, parent)
        self.is_alive = False

    def run(self):
        self.is_alive = True
        try:
            speedtest = Speedtest()
            speedtest.get_best_server()
            self.report_ready.emit((1, speedtest.results))
            speedtest.download()
            self.report_ready.emit((2, speedtest.results))
            speedtest.upload()
            self.report_ready.emit((3, speedtest.results))
        except ConfigRetrievalError as e:
            self.report_ready.emit((0, e))
        except Exception as e:
            self.report_ready.emit((-1, e))
        finally:
            self.is_alive = False

class RemoteSpeedtestReprotThread(QtCore.QThread):
    report_ready = QtCore.pyqtSignal(object)

    def __init__(self, rosHandler, parent=None):
        QtCore.QThread.__init__(self, parent)
        self.rosHandler = rosHandler
        self.is_alive = False
        self.cancel_requested = False
        self.previous_state = -2

    def run(self):
        self.is_alive = True
        while not self.cancel_requested:
            try:
                res = self.rosHandler.callService('/internet_speed_node/get_report', Trigger)
                print(res)
            except rospy.service.ServiceException as e:
                report = { 'status': -3, 'status_description': "Failed to get speed test report, ROS: \n%s" % e}
                self.report_ready.emit(report)
                break
            if not res:
                report = { 'status': -3, 'status_description': "Failed to get speed test report, ROS comm error"}
                self.report_ready.emit(report)
                break
            if res.success == True:
                report = yaml.safe_load(res.message)
                if report['status'] == self.previous_state:
                    continue
                self.previous_state = report['status']
                self.report_ready.emit(report)
                if report['status'] == 3 or report['status'] < 0:
                    break
            sleep(1)
        self.is_alive = False
        self.cancel_requested = False
        self.previous_state = -2
        print("Remote speed test thread terminated")
    
    def cancel(self):
        self.cancel_requested = True

class Launcher(QtWidgets.QMainWindow, launcher_ui.Ui_MainWindow):
    def __init__(self, parent=None):
        # QT init
        super(Launcher, self).__init__(parent)
        self.setupUi(self)
        self.connectUIControls()

        # Variables
        self.mapvizServiceName = ""
        self.rosHandler = RosHandler(self.rosOkEventHandler)

        # Threads
        self.speedTestThread = None
        self.remoteSpeedtestReportThread = None
        self.statusClearTimer = None

        # Initialise UI elements
        self.mapvizConfig = getMapvizConfig()
        self.setUIFromConfig()

        self.ping = 0
        self.upload = 0
        self.download =0

    def closeEvent(self, event):
        print("Cleaning up...")
        self.rosHandler.__del__()
        if self.statusClearTimer and self.statusClearTimer.isAlive():
            self.statusClearTimer.cancel()

    def connectUIControls(self):
        self.pushButtonNetSpdBase.clicked.connect(self.netSpdBaseClickedCallback)
        self.pushButtonNetSpdUnitree.clicked.connect(self.netSpdUnitreeClickedCallback)

        self.pushButtonMapvizLaunch.clicked.connect(self.mapvizLaunchClickedCallback)
        self.pushButtonMapvizSafeQuit.clicked.connect(self.mapvizSafeQuitClickedCallback)
        self.pushButtonMapvizForceQuit.clicked.connect(self.mapvizForceQuitClickedCallback)

        self.checkBoxShowSatMap.toggled.connect(self.showSatMapToggledCallback)
        self.checkBoxShowSlamMap.toggled.connect(self.showSlamMapToggledCallback)
        self.checkBoxShowCam.toggled.connect(self.showCamToggledCallback)
        self.checkBoxShowUnitreeImg.toggled.connect(self.showUnitreeImgToggledCallback)
        self.checkBoxShowTraj.toggled.connect(self.showTrajToggledCallback)
        self.checkBoxShowObstacle.toggled.connect(self.showObstacleToggledCallback)
        self.pushButtonMapvizSave.clicked.connect(self.saveAsDefaultCallback)

    def setUIFromConfig(self):
        if not self.mapvizConfig:
            displayErrorBox("Error getting Mapviz config. Config file may not exist.")
            return
        try:
            setFlag = 0
            for display in self.mapvizConfig['displays']:
                if display['name'] == 'Sat Map':
                    self.checkBoxShowSatMap.blockSignals(True)
                    self.checkBoxShowSatMap.setChecked(display['config']['visible'])
                    self.checkBoxShowSatMap.blockSignals(False)
                    self.lineEditBingAPI.setText(display['config']['bing_api_key'])
                    setFlag += 1
                if display['name'] == 'SLAM Map':
                    self.checkBoxShowSlamMap.blockSignals(True)
                    self.checkBoxShowSlamMap.setChecked(display['config']['visible'])
                    self.checkBoxShowSlamMap.blockSignals(False)
                    setFlag += 1
                if display['name'] == 'Robot Image':
                    self.checkBoxShowUnitreeImg.blockSignals(True)
                    self.checkBoxShowUnitreeImg.setChecked(display['config']['visible'])
                    self.checkBoxShowUnitreeImg.blockSignals(False)
                    setFlag += 1
                if display['name'] == 'Wheel Odom':
                    self.checkBoxShowTraj.blockSignals(True)
                    self.checkBoxShowTraj.setChecked(display['config']['visible'])
                    self.checkBoxShowTraj.blockSignals(False)
                    setFlag += 1
                if display['name'] == 'Local Obstacle':
                    self.checkBoxShowObstacle.blockSignals(True)
                    self.checkBoxShowObstacle.setChecked(display['config']['visible'])
                    self.checkBoxShowObstacle.blockSignals(False)
                    setFlag += 1
                if display['name'] == 'Robot Camera':
                    self.checkBoxShowCam.blockSignals(True)
                    self.checkBoxShowCam.setChecked(display['config']['visible'])
                    self.checkBoxShowCam.blockSignals(False)
                    setFlag += 1
        except Exception as e:
            displayErrorBox("Error getting Mapviz config: %s" % e)
            return
        
        if setFlag != 6:
            displayErrorBox("Error getting Mapviz config. Config file may be corrupted. Settings incomplete.")
       
    def toggleMapvizDisplayVisibility(self, type, name, on):
        suc, msg = self.toggleMapvizDisplayVisibility_(type, name, on)
        self.mapvizServiceAttempt = 0
        if not suc:
            self.setStatusBarError(msg)
        else:
            self.setStatusBarInfo("Success")
        return suc

    def toggleMapvizDisplayVisibility_(self, type, name, on):
        if not self.mapvizServiceName:
            self.mapvizServicename = self.rosHandler.findServiceName('mapviz')
            if not self.mapvizServiceName:
                return False, "Error getting Mapviz service name"
            
        req = "name: '{name}'\ntype: '{type}'\nvisible: {on}".format(name=name, type=type, on=on)
        try:
            res = self.rosHandler.callService(self.mapvizServiceName + "/remove_mapviz_display", req)
        except rospy.service.ServiceException as e:
            # Maybe mapviz service name has changed, retry once.
            if self.mapvizServiceAttempt < 1:
                self.mapvizServiceAttempt += 1
                self.mapvizServiceName = ""
                return self.toggleMapvizDisplayVisibility_(type, name, on)
            return False, "Error communicating with ROS: %s" % e
        if not res:
            return False, "Error communicating with ROS"
        if res.success == True:
            return True, ""
        else:
            return False, "Error returned from Mapviz service: %s" % res.message
 
    def rosOkEventHandler(self, event):
        if event == "master_ping":
            if self.rosHandler.masterPing:
                self.labelMasterPing.setText("{:.0f} ms".format(self.rosHandler.masterPing))
            else:
                self.labelMasterPing.setText("> %d s" % PING_TIMEOUT)
        elif event == "master_state":
            if self.rosHandler.isRosOk:
                self.labelMasterState.setText("OK")
                self.labelMasterState.setStyleSheet("color: green")
                self.pushButtonNetSpdUnitree.setEnabled(True)
            else:
                self.labelMasterState.setText("Unavailable")
                self.labelMasterState.setStyleSheet("color: red")  
                self.pushButtonNetSpdUnitree.setEnabled(False)    

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

    def netSpdBaseClickedCallback(self):
        if not self.speedTestThread:
            self.speedTestThread = SpeedtestThread()
            self.speedTestThread.report_ready.connect(self.baseReportReadyCallback)
        elif self.speedTestThread.is_alive:
            self.setStatusBarWarning("Speed test is already running")
            return
        self.labelNetSpdBaseLatency.setText("")
        self.labelNetSpdBaseDown.setText("")
        self.labelNetSpdBaseUp.setText("")
        self.labelNetSpdBaseTime.setText("Getting test server...")
        self.speedTestThread.start()

    def baseReportReadyCallback(self, report):
        if report[0] == 1:
            self.labelNetSpdBaseLatency.setText("{:.0f} ms".format(report[1].ping))
            self.labelNetSpdBaseTime.setText("Testing download speed...")
        elif report[0] == 2:
            self.labelNetSpdBaseDown.setText("{:.3f} Mbps".format(report[1].download/1000000))
            self.labelNetSpdBaseTime.setText("Testing upload speed...")
        elif report[0] == 3:
            self.labelNetSpdBaseUp.setText("{:.3f} Mbps".format(report[1].upload/1000000))
            self.labelNetSpdBaseTime.setText("%s %s" % (report[1].timestamp.split('T')[0], (report[1].timestamp.split('T')[1]).split('.')[0]))
            self.setStatusBarInfo("Speed test was successful")
        elif report[0] == 0:
            self.labelNetSpdBaseTime.setText("Error!")
            displayErrorBox("No connection to the test server or temporarily blocked because it was run too often: \n%s" % (report[1]))
        elif report[0] == -1:
            self.labelNetSpdBaseTime.setText("Error!")
            displayErrorBox("An unexpected error occured while testing network speed: \n%s" % (report[1]))

    def netSpdUnitreeClickedCallback(self):
        if not self.remoteSpeedtestReportThread:
            self.remoteSpeedtestReportThread = RemoteSpeedtestReprotThread(self.rosHandler)
            self.remoteSpeedtestReportThread.report_ready.connect(self.remoteReportReadyCallback)
        if self.remoteSpeedtestReportThread.is_alive:
            self.setStatusBarWarning("Speed test is already running")
            return
        
        self.labelNetSpdUnitreeLatency.setText("")
        self.labelNetSpdUnitreeDown.setText("")
        self.labelNetSpdUnitreeUp.setText("")
        self.labelNetSpdUnitreeTime.setText("Sending request...")

        res = None
        try:
            res = self.rosHandler.callService('/internet_speed_node/run_speed_test', Trigger)
        except rospy.service.ServiceException as e:
            displayErrorBox("An error occured while communicating with ROS: \n%s" % e)
        if not res:
            self.labelNetSpdUnitreeTime.setText("Error!")
            displayErrorBox("Not connected to ROS")
            return
        if res.success == False:
            self.labelNetSpdUnitreeTime.setText("Error!")
            displayErrorBox("Communication was OK but remote failed to start speed test: \n%s" % res.message)
            return

        self.labelNetSpdUnitreeTime.setText("Getting test server...")
        self.remoteSpeedtestReportThread.run()
        self.setStatusBarInfo("Speed test request was successful")

    def remoteReportReadyCallback(self, report):
        if report['status'] == 1:
            self.labelNetSpdUnitreeLatency.setText("{:.0f} ms".format(report['ping']))
            self.labelNetSpdUnitreeTime.setText("Testing download speed...")
        elif report['status'] == 2:
            self.labelNetSpdUnitreeDown.setText("{:.3f} Mbps".format(report['download_speed']/1000000))
            self.labelNetSpdUnitreeTime.setText("Testing upload speed...")
        elif report['status'] == 3:
            self.labelNetSpdUnitreeUp.setText("{:.3f} Mbps".format(report['upload_speed']/1000000))
            self.labelNetSpdUnitreeTime.setText("%s %s" % (report['timestamp'].split('T')[0], (report['timestamp'].split('T')[1]).split('.')[0]))
            self.setStatusBarInfo("Remote speed test successful")
        elif report['status'] == -1:
            self.labelNetSpdUnitreeTime.setText("Error!")
            displayErrorBox("No connection to the test server or temporarily blocked because it was run too often: \n%s" % (report['status_description']))
        elif report['status'] == -2:
            self.labelNetSpdUnitreeTime.setText("Error!")
            displayErrorBox("An unexpected error occured while testing network speed: \n%s" % (report['status_description']))
        elif report['status'] == -3:
            self.labelNetSpdUnitreeTime.setText("Error!")
            displayErrorBox("%s" % (report['status_description']))

    def mapvizLaunchClickedCallback(self):
        if not getPid('basestation.desktop.sh'):
            base_path = getBasePath()
            subprocess.Popen(['gnome-terminal', '--', os.path.join(base_path, 'install', 'files', 'basestation.desktop.sh')])
        else:
            self.setStatusBarInfo("Mapviz already running")

    def mapvizSafeQuitClickedCallback(self):
        pid = getPid('basestation.desktop.sh')
        if pid:
            psutil.Process(pid).terminate()
            self.setStatusBarInfo("Safe quit was successful")
        else:
            self.setStatusBarWarning("Mapviz not running")

    def mapvizForceQuitClickedCallback(self):
        pid = getPid('basestation.desktop.sh')
        if pid:
            psutil.Process(pid).kill()
            self.setStatusBarInfo("Force quit was successful")
        else:
            self.setStatusBarWarning("Mapviz not running")

    def showCamToggledCallback(self):
        if not self.toggleMapvizDisplayVisibility('mapviz_plugins/image', 'Robot Camera', self.checkBoxShowCam.isChecked()):
            self.checkBoxShowCam.blockSignals(True)
            self.checkBoxShowCam.setChecked(not self.checkBoxShowCam.isChecked())
            self.checkBoxShowCam.blockSignals(False)

    def showSatMapToggledCallback(self):
        if not self.toggleMapvizDisplayVisibility('mapviz_plugins/tile_map', 'Sat Map', self.checkBoxShowSatMap.isChecked()):
            self.checkBoxShowSatMap.blockSignals(True)
            self.checkBoxShowSatMap.setChecked(not self.checkBoxShowSatMap.isChecked())
            self.checkBoxShowSatMap.blockSignals(False)
    
    def showSlamMapToggledCallback(self):
        if not self.toggleMapvizDisplayVisibility('mapviz_plugins/occupancy_grid', 'SLAM Map', self.checkBoxShowSlamMap.isChecked()):
            self.checkBoxShowSlamMap.blockSignals(True)
            self.checkBoxShowSlamMap.setChecked(not self.checkBoxShowSlamMap.isChecked())
            self.checkBoxShowSlamMap.blockSignals(False)

    def showUnitreeImgToggledCallback(self):
        if not self.toggleMapvizDisplayVisibility('mapviz_plugins/image', 'Unitree Image', self.checkBoxShowUnitreeImg.isChecked()):
            self.checkBoxShowUnitreeImg.blockSignals(True)
            self.checkBoxShowUnitreeImg.setChecked(not self.checkBoxShowUnitreeImg.isChecked())
            self.checkBoxShowUnitreeImg.blockSignals(False)
    
    def showTrajToggledCallback(self):
        if not self.toggleMapvizDisplayVisibility('mapviz_plugins/odometry', 'Wheel Odom', self.checkBoxShowTraj.isChecked()):
            self.checkBoxShowTraj.setChecked(not self.checkBoxShowTraj.isChecked())
            self.checkBoxShowTraj.setChecked(not self.checkBoxShowTraj.isChecked())
            self.checkBoxShowTraj.blockSignals(False)

    def showObstacleToggledCallback(self):
        if not self.toggleMapvizDisplayVisibility('mapviz_plugins/occupancy_grid', 'Local Obstacle', self.checkBoxShowObstacle.isChecked()):
            self.checkBoxShowObstacle.blockSignals(True)
            self.checkBoxShowObstacle.setChecked(not self.checkBoxShowObstacle.isChecked())
            self.checkBoxShowObstacle.blockSignals(False)

    def saveAsDefaultCallback(self):
        if not self.mapvizConfig:
            displayErrorBox("Error getting Mapviz config. Config file may not exist.")
            return
        try:
            setFlag = 0
            for display in self.mapvizConfig['displays']:
                if display['name'] == 'Sat Map':
                    display['config']['visible'] = self.checkBoxShowSatMap.isChecked()
                    display['config']['bing_api_key'] = self.lineEditBingAPI.text().encode('utf-8')
                    setFlag += 1
                if display['name'] == 'SLAM Map':
                    display['config']['visible'] = self.checkBoxShowSlamMap.isChecked()
                    setFlag += 1
                if display['name'] == 'Robot Image':
                    display['config']['visible'] = self.checkBoxShowUnitreeImg.isChecked()
                    setFlag += 1
                if display['name'] == 'Wheel Odom':
                    display['config']['visible'] = self.checkBoxShowTraj.isChecked()
                    setFlag += 1
                if display['name'] == 'Local Obstacle':
                    display['config']['visible'] = self.checkBoxShowObstacle.isChecked()
                    setFlag += 1
                if display['name'] == 'Robot Camera':
                    display['config']['visible'] = self.checkBoxShowCam.isChecked()
                    setFlag += 1
        except Exception as e:
            displayErrorBox("Aborted. Error getting Mapviz config: %s" % e)
            return

        if setFlag != 6:
            displayErrorBox("Aborted. Mapviz config file is ill formated.")
            return
        
        saveMapvizConfig(self.mapvizConfig)
        self.setStatusBarInfo("Success")

def main():
    app = QApplication(sys.argv)
    form = Launcher()
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()