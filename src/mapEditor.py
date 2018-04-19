import rospy
import rospkg
from PyQt4 import QtGui, QtCore, QtNetwork
from game import Map
from items import *

def QTtoART(x=None,y=None):
    if y is None:
        #print("x = " + str(x/2000.0))
        return x/2000.0
    #print("y = " + str(0.60 - y/2000.0))
    return 0.60 - y/2000.0

rospack = rospkg.RosPack()
imagesPath = rospack.get_path('my_gui') + '/src/images/'

class MapEditor:
    def __init__(self, scene, mainWindow):
        self.scene = scene
        self.map = self.createEmptyMap()
        self.scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(30,144,255)))
        self.mainWindow = mainWindow

        self.map.drawMap()
        self.drawUI()

    def createEmptyMap(self):
        emptyMap = Map(self.scene, editor=True)
        emptyMap.addTile("NONE",[0,0],0)
        emptyMap.addTile("NONE",[1,0],1)
        emptyMap.addTile("NONE",[2,0],2)
        emptyMap.addTile("NONE",[0,1],3)
        emptyMap.addTile("NONE",[1,1],4)
        emptyMap.addTile("NONE",[2,1],5)
        emptyMap.addTile("NONE",[3,1],6)
        emptyMap.addTile("NONE",[-1,2],7)
        emptyMap.addTile("NONE",[0,2],8)
        emptyMap.addTile("NONE",[1,2],9)
        emptyMap.addTile("NONE",[2,2],10)
        emptyMap.addTile("NONE",[3,2],11)
        emptyMap.addTile("NONE",[0,3],12)
        emptyMap.addTile("NONE",[1,3],13)
        emptyMap.addTile("NONE",[2,3],14)
        emptyMap.addTile("NONE",[3,3],15)
        emptyMap.addTile("NONE",[0,4],16)
        emptyMap.addTile("NONE",[1,4],17)
        emptyMap.addTile("NONE",[2,4],18)
        emptyMap.setTileNumbers([10,2,1,12,6,4,10,9,11,0,3,8,8,3,4,1,5,6,11])
        return emptyMap

    def drawUI(self):
        self.areaTypeButtons = []
        for index, tileType in enumerate(["MOUNTAINS", "PASTURE", "HILLS", "DESERT", "FOREST", "FIELDS"]):
            XCoord = 250 if index < 3 else 1600
            self.areaTypeButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord), QTtoART(y=200+index%3*250), "", None, \
            self.map.changeSelectedAreaType, scale = 8, background_color=QtCore.Qt.transparent, \
            image_path=imagesPath + tileType + ".png", data=tileType))

        self.menuButtons = []
        XCoord = 350
        YCoord = 50
        XOffset = 350
        self.menuButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord), QTtoART(y=YCoord), "New map", None, self.newMap, scale = 3))
        self.menuButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+XOffset), QTtoART(y=YCoord), "Load map", None, self.loadMap, scale = 3))
        self.menuButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+2*XOffset), QTtoART(y=YCoord), "Save map", None, self.saveMap, scale = 3))
        self.menuButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+3*XOffset), QTtoART(y=YCoord), "Exit editor", None, self.exitEditor, scale = 3))

    def newMap(self, button):
        self.map = self.createEmptyMap()
        self.scene.clear()
        self.map.drawMap()
        self.drawUI()

    def loadMap(self, button):
        resPath = rospack.get_path('my_gui') + '/src/'
        file = open(resPath + "testfile.map",'r') 
        temp = file.read().splitlines()
        for tile in self.map.tiles:
            tile.areaType = temp[tile.index]
            print("areaType = " + tile.areaType)
        file.close()
        self.scene.clear()
        self.map.drawMap()
        self.drawUI()

    def saveMap(self, button):
        resPath = rospack.get_path('my_gui') + '/src/'
        file = open(resPath + "testfile.map",'w+') 
 
        for tile in self.map.tiles:
            file.write(tile.areaType + "\n")
         
        file.close()

    def exitEditor(self, button):
        self.scene.clear()
        self.mainWindow.toMainMenu()
        pass
