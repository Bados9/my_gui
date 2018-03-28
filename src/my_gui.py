#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore, QtNetwork
from art_projected_gui.helpers import ProjectorHelper
from items import ButtonItem
from game import Game

class menuTabs():
    def __init__(self, scene, parent=None):
        self.scene = scene
        self.context = "mainMenu"
        self.mainMenuItems = []
        self.mainMenuItems.append(ButtonItem(self.scene, 0.6, 0.6, "New Game", None, self.launchGame, scale = 3))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.6, 0.5, "Settings", None, self.toSettings, scale = 3))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.6, 0.4, "Exit", None, self.quitApp, scale = 3))

        self.settingsItems = []
        self.settingsItems.append(ButtonItem(self.scene, 0.8, 0.3, "some settings", None, None, scale = 3))
        self.settingsItems.append(ButtonItem(self.scene, 0.2, 0.4, "Back", None, self.toMainMenu, scale = 3))
        for item in self.settingsItems:
            self.scene.removeItem(item)

    def toMainMenu(self, event):
        self.context = "mainMenu"
        for item in self.settingsItems:
            self.scene.removeItem(item)

        for item in self.mainMenuItems:
            self.scene.addItem(item)

    def toSettings(self, event):
        self.context = "settings"
        for item in self.mainMenuItems:
            self.scene.removeItem(item)

        for item in self.settingsItems:
            self.scene.addItem(item)

    def launchGame(self, event):
        if self.context == "mainMenu":
            for item in self.mainMenuItems:
                self.scene.removeItem(item)
        else:
            for item in self.settingsItems:
                self.scene.removeItem(item)

        self.game = Game(self.scene)

    def quitApp(self, event):
        sys.exit(0)

class customGraphicsView(QtGui.QGraphicsView):

    def __init__(self, parent=None):
        QtGui.QGraphicsView.__init__(self, parent)

        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

    def resizeEvent(self, evt=None):

        self.fitInView(self.sceneRect(), QtCore.Qt.KeepAspectRatio)


class MyGui(QtCore.QObject):

    def __init__(self, x, y, width, height, rpm, scene_server_port):

        super(MyGui, self).__init__()

        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.port = scene_server_port

        w = self.width * rpm
        h = self.height / self.width * w

        self.scene = QtGui.QGraphicsScene(0, 0, int(w), int(h))
        self.scene.rpm = rpm
        self.scene.setBackgroundBrush(QtCore.Qt.black)

        self.view = customGraphicsView(self.scene)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QtGui.QGraphicsView.FullViewportUpdate)
        self.view.setStyleSheet("QGraphicsView { border-style: none; }")

        self.tcpServer = QtNetwork.QTcpServer(self)
        if not self.tcpServer.listen(port=self.port):
            rospy.logerr(
                'Failed to start scene TCP server on port ' + str(self.port))

        self.tcpServer.newConnection.connect(self.new_connection)
        self.connections = []

        self.scene_timer = QtCore.QTimer()
        self.connect(
            self.scene_timer,
            QtCore.SIGNAL('timeout()'),
            self.send_to_clients_evt)
        self.scene_timer.start(1.0 / 15 * 1000)

        #self.projectors = [ProjectorHelper("n2")]
        self.projectors = []
        rospy.loginfo("Waiting for projector nodes...")
        for proj in self.projectors:
                proj.wait_until_available()
                if not proj.is_calibrated():
                    rospy.loginfo("Starting calibration of projector: " + proj.proj_id)
                    proj.calibrate(self.calibrated_cb)
                else:
                    rospy.loginfo("Projector " + proj.proj_id + " already calibrated.")
        rospy.loginfo("Done")

        rospy.loginfo("Ready")

    def calibrated_cb(self, proj):

        rospy.loginfo("Projector " + proj.proj_id + " calibrated: " + str(proj.is_calibrated()))

    def new_connection(self):

        rospy.loginfo('Some projector node just connected.')
        self.connections.append(self.tcpServer.nextPendingConnection())
        self.connections[-1].setSocketOption(
            QtNetwork.QAbstractSocket.LowDelayOption, 1)

        # TODO deal with disconnected clients!
        # self.connections[-1].disconnected.connect(clientConnection.deleteLater)

    def send_to_clients_evt(self):

        if len(self.connections) == 0:
            return

        # start = time.time()

        pix = QtGui.QImage(
            self.scene.width(),
            self.scene.height(),
            QtGui.QImage.Format_RGB888)
        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        self.scene.render(painter)
        painter.end()
        pix = pix.mirrored()

        block = QtCore.QByteArray()
        out = QtCore.QDataStream(block, QtCore.QIODevice.WriteOnly)
        out.setVersion(QtCore.QDataStream.Qt_4_0)
        out.writeUInt32(0)

        img = QtCore.QByteArray()
        buffer = QtCore.QBuffer(img)
        buffer.open(QtCore.QIODevice.WriteOnly)
        pix.save(buffer, "JPG", 95)
        out << img

        out.device().seek(0)
        out.writeUInt32(block.size() - 4)

        # print block.size()

        for con in self.connections:
            con.write(block)

    def debug_view(self):
        """Show window with scene - for debugging purposes."""

        self.view.show()

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()


def main(args):

    rospy.init_node('projected_gui_example', anonymous=True, log_level=rospy.DEBUG)

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)

    gui = MyGui(0, 0, 1.35, 0.64, 2000, 1234)
    
    gui.menuTabs = menuTabs(gui.scene)
    gui.debug_view()

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
