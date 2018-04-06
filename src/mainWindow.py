import sys
import signal
import rospy
import rospkg
from PyQt4 import QtGui, QtCore, QtNetwork
from items import *
from game import Game
from art_msgs.msg import Touch

def QTtoART(x=None,y=None):
    if y is None:
        return x/2000.0
    return 0.60 - y/2000.0

class MainWindow():
    def __init__(self, scene, parent=None):
        rospack = rospkg.RosPack()
        imagesPath = rospack.get_path('my_gui') + '/src/images/'

        rospy.Subscriber('/art/interface/touchtable/touch', Touch, self.touch_cb)

        self.scene = scene
        self.context = "mainMenu"
        self.mainMenuItems = []
        self.mainMenuItems.append(ButtonItem(self.scene, QTtoART(x=950), QTtoART(y=0), "New Game", None, \
            self.launchGame, scale = 3, background_color=QtCore.Qt.transparent,image_path=imagesPath+"button_play.png"))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.5, 0.5, "Settings", None, self.toSettings, scale = 3))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.5, 0.4, "Exit", None, self.quitApp, scale = 3))

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
        for item in self.mainMenuItems:
            self.scene.removeItem(item)
        self.context = "game"
        self.game = Game(self.scene)
        #self.game.nextTurn()

    def touch_cb(self, data):
        print(data)
        print("x = " + str(data.point.point.x))
        print("y = " + str(data.point.point.y))
        touch = PointItem(self.scene, data.point.point.x, data.point.point.y, None)
        if self.context == "mainMenu":
            for item in self.mainMenuItems:
                if item.collidesWithItem(touch):
                    print("nastala kolize s tlacitkem " + str(item.caption))
                    item.cursor_click()
                else:
                    print("zadna kolize")
        
        if self.context == "settings":
            for item in self.settingsItems:
                if item.collidesWithItem(touch):
                    print("nastala kolize s tlacitkem " + str(item.caption))
                    item.cursor_click()
                else:
                    print("zadna kolize")

        self.scene.removeItem(touch)

    def quitApp(self, event):
        sys.exit(0)