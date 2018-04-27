import rospy
import rospkg
from collections import Counter
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
        self.noMoreTiles = []

        self.map.drawMap()
        self.drawUI()

    def createEmptyMap(self):
        emptyMap = Map(self.scene, self, editor=True)
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
        emptyMap.setTileNumbers(arg=[-1]*19)
        return emptyMap

    def drawUI(self):
        self.areaTypeButtons = {}
        for index, tileType in enumerate(["MOUNTAINS", "PASTURE", "HILLS", "DESERT", "FOREST", "FIELDS"]):
            XCoord = 250 if index < 3 else 1600
            self.areaTypeButtons[tileType] = ButtonItem(self.scene, QTtoART(x=XCoord), QTtoART(y=200+index%3*250), "", None, \
            self.map.changeSelectedAreaType, scale = 8, background_color=QtCore.Qt.transparent, \
            image_path=imagesPath + tileType + ".png", data=tileType)
            self.areaTypeButtons[tileType].h = 180

        self.menuButtons = []
        XCoord = 350
        YCoord = 50
        XOffset = 350
        self.menuButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord), QTtoART(y=YCoord), "New map", None, self.newMap, scale = 3))
        self.menuButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+XOffset), QTtoART(y=YCoord), "Load map", None, self.drawLoadSlots, scale = 3))
        self.menuButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+2*XOffset), QTtoART(y=YCoord), "Save map", None, self.drawSaveSlots, scale = 3))
        self.menuButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+3*XOffset), QTtoART(y=YCoord), "Exit editor", None, self.exitEditor, scale = 3))
        
        self.invisibleButton = ButtonItem(self.scene, QTtoART(x=2000), QTtoART(y=1500), "", None, self.map.changeSelectedAreaType, data="NONE")
    
    def wrongTile(self, button=None):
        self.info.set_content("You must choose corner tile!")

    def setTileNumbers(self, button=None):
        self.map.scene.removeItem(self.info)
        self.info = None
        self.map.setTileNumbers(button)
        self.scene.clear()
        self.map.drawMap()
        self.drawUI()

    def setNumbers(self, button=None):
        for tile in self.map.tiles:
            if tile.index in [0,2,7,11,16,18]:
                tile.tileBtn.set_cb(self.setTileNumbers)
            else:
                tile.tileBtn.set_cb(self.wrongTile)

    def setNoMore(self, tileType):
        self.areaTypeButtons[tileType].set_enabled(False)
        #TODO udelat lepe

    def removeNoMore(self, tileType):
        self.areaTypeButtons[tileType].set_enabled(True)
        #TODO udelat lepe

    def checkCounts(self):
        typeList = [x.areaType for x in self.map.tiles]
        typeCounts = Counter(typeList)
        for areaType in ["PASTURE","FOREST","FIELDS"]:
            if typeCounts[areaType] > 3:
                self.setNoMore(areaType)
                if areaType not in self.noMoreTiles:
                    self.noMoreTiles.append(areaType)
                    self.invisibleButton.cursor_click()
            elif areaType in self.noMoreTiles:
                self.removeNoMore(areaType)
                self.noMoreTiles.remove(areaType)

        for areaType in ["HILLS", "MOUNTAINS"]:
            if typeCounts[areaType] > 2:
                self.setNoMore(areaType)
                if areaType not in self.noMoreTiles:
                    self.noMoreTiles.append(areaType)
                    self.invisibleButton.cursor_click()
            elif areaType in self.noMoreTiles:
                self.removeNoMore(areaType)
                self.noMoreTiles.remove(areaType)

        for areaType in ["DESERT"]:
            if typeCounts[areaType] > 0:
                self.setNoMore(areaType)
                if areaType not in self.noMoreTiles:
                    self.noMoreTiles.append(areaType)
                    self.invisibleButton.cursor_click()
            elif areaType in self.noMoreTiles:
                self.removeNoMore(areaType)
                self.noMoreTiles.remove(areaType)

        if "NONE" not in [x.areaType for x in self.map.tiles]:
            self.info = DescItem(self.scene, QTtoART(x=900), QTtoART(y=1000), None)
            self.info.set_content("Choose corner tile for start numbering", 3)
            self.setNumbers()

    def newMap(self, button):
        self.map = self.createEmptyMap()
        self.scene.clear()
        self.map.drawMap()
        self.drawUI()

    def drawLoadSlots(self, button):
        self.slotButtons = []
        for btn in self.menuButtons:
            self.scene.removeItem(btn)
        XCoord = 300
        YCoord = 50
        XOffset = 250
        for i in range(5):
            self.slotButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+i*XOffset), QTtoART(y=YCoord), "", None, self.loadMap, scale = 2, data=i))
            self.slotButtons[i].set_caption("Slot " + str(i), QTtoART(x=200))
        self.slotButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+5*XOffset), QTtoART(y=YCoord), "Back", None, self.displayMenu, scale = 2))


    def loadMap(self, button):
        resPath = rospack.get_path('my_gui') + '/src/maps/'
        
        file = open(resPath + "mapfile" + str(button.data) + ".map",'r') 

        temp = file.read().splitlines()
        for tile in self.map.tiles:
            tile.areaType = temp[tile.index]
        numbers = temp[19].split(',')
        numberList = []
        for num in numbers[:19]:
            numberList.append(int(num))
        self.map.setTileNumbers(arg=numberList)
        file.close()
        
        self.scene.clear()
        self.map.drawMap()
        self.drawUI()

    def displayMenu(self, button=None):
        for btn in self.slotButtons:
            self.scene.removeItem(btn)
        for btn in self.menuButtons:
            self.scene.addItem(btn)

    def drawSaveSlots(self, button):
        self.slotButtons = []
        for btn in self.menuButtons:
            self.scene.removeItem(btn)
        XCoord = 300
        YCoord = 50
        XOffset = 250
        for i in range(5):
            self.slotButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+i*XOffset), QTtoART(y=YCoord), "", None, self.saveMap, scale = 2, data=i))
            self.slotButtons[i].set_caption("Slot " + str(i), QTtoART(x=200))
        self.slotButtons.append(ButtonItem(self.scene, QTtoART(x=XCoord+5*XOffset), QTtoART(y=YCoord), "Back", None, self.displayMenu, scale = 2))

    def saveMap(self, button):
        resPath = rospack.get_path('my_gui') + '/src/maps/'
        
        file = open(resPath + "mapfile" + str(button.data) + ".map",'w+') 
 
        for tile in self.map.tiles:
            file.write(tile.areaType + "\n")
        for number in self.map.tileNumbers:
            file.write(str(number) + ",")
        file.write("\n")
        file.close()
        self.displayMenu()

    def exitEditor(self, button):
        self.scene.clear()
        self.mainWindow.toMainMenu()
        pass
