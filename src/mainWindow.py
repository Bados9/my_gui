import sys
import signal
import rospy
import rospkg
from PyQt4 import QtGui, QtCore, QtNetwork
from items import *
from game import Game
from mapEditor import MapEditor
from art_msgs.msg import Touch

def QTtoART(x=None,y=None):
    if y is None:
        return x/2000.0
    return 0.60 - y/2000.0

class MainWindow():
    def __init__(self, scene, parent=None):
        rospack = rospkg.RosPack()
        imagesPath = rospack.get_path('my_gui') + '/src/images/'
        self.scene = scene
        TouchTableItem(self.scene, '/art/interface/touchtable/touch')

        self.mainMenuItems = []
        self.mainMenuItems.append(ButtonItem(self.scene, QTtoART(x=950), QTtoART(y=0), "New Game", None, \
            self.launchGame, scale = 3, background_color=QtCore.Qt.transparent,image_path=imagesPath+"button_play.png"))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.5, 0.5, "Map editor", None, self.launchMapEditor, scale = 3))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.5, 0.4, "Settings", None, self.toSettings, scale = 3))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.5, 0.3, "Exit", None, self.quitApp, scale = 3))

        self.settingsItems = []
        self.settingsItems.append(ButtonItem(self.scene, 0.8, 0.3, "some settings", None, None, scale = 3))
        self.settingsItems.append(ButtonItem(self.scene, 0.2, 0.4, "Back", None, self.toMainMenu, scale = 3))
        for item in self.settingsItems:
            self.scene.removeItem(item)

        self.mainMenuItems[0].cursor_click()

    def toMainMenu(self, button=None):
        for item in self.settingsItems:
            self.scene.removeItem(item)

        for item in self.mainMenuItems:
            self.scene.addItem(item)

    def toSettings(self, event):
        for item in self.mainMenuItems:
            self.scene.removeItem(item)

        for item in self.settingsItems:
            self.scene.addItem(item)

    def launchGame(self, event):
        for item in self.mainMenuItems:
            self.scene.removeItem(item)
        self.game = Game(self.scene)
        self.game.nextTurn()

    def launchMapEditor(self, button):
        for item in self.mainMenuItems:
            self.scene.removeItem(item)
        self.editor = MapEditor(self.scene, self)

    def quitApp(self, event):
        sys.exit(0)