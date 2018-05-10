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
        self.mapMenuHidden = False
        self.startingMap = "default"
        self.numberOfPlayers = 4
        TouchTableItem(self.scene, '/art/interface/touchtable/touch')

        self.mainMenuItems = []
        self.mainMenuItems.append(ButtonItem(self.scene, QTtoART(x=950), QTtoART(y=0), "New Game", None, \
            self.launchGame, scale = 3))
        mapSelection = DescItem(self.scene, QTtoART(x=950), QTtoART(y=120), None)
        mapSelection.set_content("Map: Default", 3)
        self.mainMenuItems.append(mapSelection)
        self.mainMenuItems.append(ButtonItem(self.scene, 0.5, 0.5, "Map editor", None, self.launchMapEditor, scale = 3))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.5, 0.4, "Settings", None, self.toSettings, scale = 3))
        self.mainMenuItems.append(ButtonItem(self.scene, 0.5, 0.3, "Exit", None, self.quitApp, scale = 3))

        changeMapXCoord = 500
        changeMapYCoord = 300
        changeMapXOffset = 210
        self.changeMapMenuItems = []
        for i in range(5):
            if self.startingMap == i:
                self.changeMapMenuItems.append(ButtonItem(self.scene, QTtoART(x=changeMapXCoord+i*changeMapXOffset), \
                    QTtoART(y=changeMapYCoord), "", None, self.changeStartingMap, scale = 2, data=i, \
                    background_color=QtCore.Qt.red))
                self.changeMapMenuItems[i].set_caption("Slot " + str(i), QTtoART(x=200))
            else:
                self.changeMapMenuItems.append(ButtonItem(self.scene, QTtoART(x=changeMapXCoord+i*changeMapXOffset), \
                    QTtoART(y=changeMapYCoord), "", None, self.changeStartingMap, scale = 2, data=i))
                self.changeMapMenuItems[i].set_caption("Slot " + str(i), QTtoART(x=200))
        
        if self.startingMap == "default":
            self.changeMapMenuItems.append(ButtonItem(self.scene, QTtoART(x=changeMapXCoord+5*changeMapXOffset), \
                QTtoART(y=changeMapYCoord), "Default map", None, self.changeStartingMap, scale = 2, data="default",\
                background_color=QtCore.Qt.red))
        else:
            self.changeMapMenuItems.append(ButtonItem(self.scene, QTtoART(x=changeMapXCoord+5*changeMapXOffset), \
                QTtoART(y=changeMapYCoord), "Default map", None, self.changeStartingMap, scale = 2, data="default"))
        self.toggleChangeMapMenu()

        self.settingsItems = []
        mapLabel = DescItem(self.scene, QTtoART(x=changeMapXCoord-430), QTtoART(y=changeMapYCoord+10), None)
        mapLabel.set_content("Selected map: ", 3)
        self.settingsItems.append(mapLabel)
        
        playersLabel = DescItem(self.scene, QTtoART(x=changeMapXCoord-430), QTtoART(y=changeMapYCoord-100), None)
        playersLabel.set_content("Number of players: ", 3)
        self.settingsItems.append(playersLabel)
        
        self.settingsItems.append(ButtonItem(self.scene, QTtoART(x=changeMapXCoord), \
            QTtoART(y=changeMapYCoord-100), "-", None, self.decreaseNoP, scale = 2))
        
        self.numberLabel = DescItem(self.scene, QTtoART(x=changeMapXCoord+0.5*changeMapXOffset), QTtoART(y=changeMapYCoord-100), None)
        self.numberLabel.set_content(str(self.numberOfPlayers), 3)
        self.settingsItems.append(self.numberLabel)
        
        self.settingsItems.append(ButtonItem(self.scene, QTtoART(x=changeMapXCoord+changeMapXOffset), \
            QTtoART(y=changeMapYCoord-100), "+", None, self.increaseNoP, scale = 2))
        
        self.settingsItems.append(ButtonItem(self.scene, 0.2, 0.4, "Back", None, self.toMainMenu, scale = 3))
        
        for item in self.settingsItems:
            self.scene.removeItem(item)

        #self.mainMenuItems[0].cursor_click()

    def decreaseNoP(self, button):
        self.numberOfPlayers -= 1
        if self.numberOfPlayers < 2:
            self.numberOfPlayers = 2
        self.numberLabel.set_content(str(self.numberOfPlayers), 3)

    def increaseNoP(self, button):
        self.numberOfPlayers += 1
        if self.numberOfPlayers > 4:
            self.numberOfPlayers = 4
        self.numberLabel.set_content(str(self.numberOfPlayers), 3)

    def changeStartingMap(self, button=None):
        self.startingMap = button.data
        for item in self.changeMapMenuItems:
            item.set_background_color(QtCore.Qt.green)
        button.set_background_color(QtCore.Qt.red)

        if button.data == "default":
            self.mainMenuItems[1].set_content("Map: Default", 3)
        else:
            self.mainMenuItems[1].set_content("Map: Slot " + str(button.data), 3)

    def toggleChangeMapMenu(self, button=None):
        if self.mapMenuHidden:
            for item in self.changeMapMenuItems:
                self.scene.addItem(item)
            self.mapMenuHidden = False
        else:
            for item in self.changeMapMenuItems:
                self.scene.removeItem(item)
            self.mapMenuHidden = True

    def toMainMenu(self, button=None):
        for item in self.settingsItems:
            self.scene.removeItem(item)

        for item in self.mainMenuItems:
            self.scene.addItem(item)
        self.toggleChangeMapMenu()

    def toSettings(self, event):
        for item in self.mainMenuItems:
            self.scene.removeItem(item)

        for item in self.settingsItems:
            self.scene.addItem(item)
        self.toggleChangeMapMenu()

    def launchGame(self, event):
        for item in self.mainMenuItems:
            self.scene.removeItem(item)
        self.game = Game(self.scene, self.startingMap, self.numberOfPlayers)
        self.game.nextTurn()

    def launchMapEditor(self, button):
        for item in self.mainMenuItems:
            self.scene.removeItem(item)
        self.editor = MapEditor(self.scene, self)

    def quitApp(self, event):
        sys.exit(0)