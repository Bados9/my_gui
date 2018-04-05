import sys
import os
import signal
import rospy
import rospkg
import math
import gamePieces
from PyQt4 import QtGui, QtCore, QtNetwork
from art_projected_gui.helpers import ProjectorHelper
from items import *

rospack = rospkg.RosPack()
imagesPath = rospack.get_path('my_gui') + '/src/images/'

def QTtoART(x=None,y=None):
	if y is None:
		return x/2000
	return 0.60 - y/2000

adjBuildings = {0 : [ 0, 1, 2, 8, 9,10],
				1 : [ 2, 3, 4,10,11,12],
				2 : [ 4, 5, 6,12,13,14],
				3 : [ 7, 8, 9,17,18,19],
				4 : [ 9,10,11,19,20,21],
				5 : [11,12,13,21,22,23],
				6 : [13,14,15,23,24,25],
				7 : [16,17,18,27,28,29],
				8 : [18,19,20,29,30,31],
				9 : [20,21,22,31,32,33],
			   10 : [22,23,24,33,34,35],
			   11 : [24,25,26,35,36,37],
			   12 : [28,29,30,38,39,40],
			   13 : [30,31,32,40,41,42],
			   14 : [32,33,34,42,43,44],
			   15 : [34,35,36,44,45,46],
			   16 : [39,40,41,47,48,49],
			   17 : [41,42,43,49,50,51],
			   18 : [43,44,45,51,52,53]}

crossroads = {  0 : [],
				1 : [],
				2 : [],
				3 : [],
				4 : [],
				5 : [],
				6 : [],
				7 : [],
				8 : [],
				9 : [],
			   10 : [],
			   11 : [],
			   12 : [],
			   13 : [],
			   14 : [],
			   15 : [],
			   16 : [],
			   17 : [],
			   18 : [],
			   19 : [],
			   20 : [],
			   21 : [],
			   22 : [],
			   23 : [],
			   24 : [],
			   25 : [],
			   26 : [],
			   27 : [],
			   28 : [],
			   29 : [],
			   30 : [],
			   31 : [],
			   32 : [],
			   33 : [],
			   34 : [],
			   35 : [],
			   36 : [],
			   37 : [],
			   38 : [],
			   39 : [],
			   40 : [],
			   41 : [],
			   42 : [],
			   43 : [],
			   44 : [],
			   45 : [],
			   46 : [],
			   47 : [],
			   48 : [],
			   49 : [],
			   50 : [],
			   51 : [],
			   52 : [],
			   53 : []}

areaSupplyDict = {	"MOUNTAINS" : "ORE",
					"FOREST" : "LUMBER",
					"HILLS" : "BRICK",
					"PASTURE" : "WOOL",
					"FIELDS" : "GRAIN",
					"DESERT" : "NONE"
}

def hexCorners(center, size):
        corners = []
        for i in range(6):
            angle_deg = 60 * i   + 30
            if (size > 400): # hack for bounding hexagon
                angle_deg -= 30
            angle_rad = math.pi / 180 * angle_deg
            corners.append([center[0] + size * math.cos(angle_rad),center[1] + size * math.sin(angle_rad)])
        return corners

#################################################################################################################
#################################					TILE CLASS 					#################################
#################################################################################################################

class Tile:
    def __init__(self, areaType, position, index, scene):
        self.scene = scene
        self.areaType = areaType
        self.number = 0
        self.supplyType = areaSupplyDict[self.areaType]
        self.adjacentBuildings = adjBuildings[index]
        self.position = position #position on map
        self.index = index
        self.coords = [0,0] #in pixels
        self.polygon = None

    def setNumber(self, number):
    	self.number = number

    def drawTile(self, size):
    	vertSkip = size*2
        horSkip = math.sqrt(3)/2 * vertSkip

    	self.coords = [1000-vertSkip,600-2*horSkip]
        self.coords[1] += self.position[1]*horSkip
        self.coords[0] += self.position[0]*vertSkip
        if (self.position[1] & 1):
            self.coords[0] -= vertSkip*0.5

        corners = hexCorners(self.coords, size+size/6.5)
        polygon = QtGui.QPolygon([QtCore.QPoint(corners[0][0],corners[0][1]),QtCore.QPoint(corners[1][0],corners[1][1]), \
                    QtCore.QPoint(corners[2][0],corners[2][1]),QtCore.QPoint(corners[3][0],corners[3][1]), \
                    QtCore.QPoint(corners[4][0],corners[4][1]), QtCore.QPoint(corners[5][0],corners[5][1])])
        polygonF = QtGui.QPolygonF(polygon)
        self.polygon = QtGui.QGraphicsPolygonItem(polygonF)

        if self.areaType == "MOUNTAINS":
            self.polygon.setBrush(QtGui.QBrush(QtGui.QColor(169,169,169)))
        elif self.areaType == "PASTURE":
            self.polygon.setBrush(QtGui.QBrush(QtCore.Qt.green))
        elif self.areaType == "FOREST":
            self.polygon.setBrush(QtGui.QBrush(QtGui.QColor(34,139,34)))
        elif self.areaType == "FIELDS":
            self.polygon.setBrush(QtGui.QBrush(QtGui.QColor(255,255,0)))
        elif self.areaType == "HILLS":
            self.polygon.setBrush(QtGui.QBrush(QtGui.QColor(165,42,42)))
        elif self.areaType == "DESERT":
            self.polygon.setBrush(QtGui.QBrush(QtGui.QColor(255,140,0)))
        elif self.areaType == "NONE":
            self.polygon.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
        else:
            self.polygon.setBrush(QtGui.QBrush(QtCore.Qt.red))

        self.polygon.setPen(QtGui.QPen(QtCore.Qt.black))
        self.scene.addItem(self.polygon)

        self.image = QtGui.QGraphicsPixmapItem(QtGui.QPixmap(QtGui.QImage(imagesPath + self.number + ".png")), scene=self.scene)
        self.image.setPos(self.coords[0],self.coords[1])

    def setFocus(self):
    	self.polygon.setTransformOriginPoint(self.coords[0], self.coords[1]);
    	self.polygon.setScale(1.5)
    	self.scene.removeItem(self.polygon)
    	self.scene.addItem(self.polygon)
    	#TODO transformovat i cislo
    	#TODO pridat buttonky kolem

    def unsetFocus(self):
    	self.polygon.setScale(1)
    	self.scene.removeItem(self.polygon)
    	self.scene.addItem(self.polygon)
    	#TODO transformovat i cislo
    	#TODO oddelat buttonky

    def giveSupplies(self, game):
    	for building in adjacentBuildings:
    		if building is not None:
    		   	game.giveSupplyToPlayer(building.color, self.supplyType)
    			if building.type == "CITY":
    				game.giveSupplyToPlayer(building.color, self.supplyType)

#################################################################################################################
#################################					MAP CLASS 					#################################
#################################################################################################################

class Map:
    def __init__(self, scene):
        self.scene = scene
        self.tiles = [None] * 19
        self.tileNumbers = [None] * 19 #seradi se do zakladu, pak bude mozno posunout
        self.roads = [None] * 70
        self.places = [None] * 54

    def addTile(self, areaType, position, index):
        self.tiles[index] = Tile(areaType, position, index, self.scene)
    
    def setTileNumbers(self, tileNumbers):
        self.tileNumbers = tileNumbers
        for tile in self.tiles:
        	tile.setNumber(tileNumbers[tile.index])

    def addCity(self, city):
        pass

    def addVillage(self, village):
        pass

    def addRoad(self, road):
        pass

    def drawMap(self):
        size = 75
        
        for tile in self.tiles:
            tile.drawTile(size)

        corners = hexCorners([1000, 600],size*5.5)
        boundingPolygon = QtGui.QPolygon([QtCore.QPoint(corners[0][0],corners[0][1]),QtCore.QPoint(corners[1][0],corners[1][1]), \
                    QtCore.QPoint(corners[2][0],corners[2][1]),QtCore.QPoint(corners[3][0],corners[3][1]), \
                    QtCore.QPoint(corners[4][0],corners[4][1]), QtCore.QPoint(corners[5][0],corners[5][1])])
        boundingPolygonF = QtGui.QPolygonF(boundingPolygon)
        self.boundingPolygon = QtGui.QGraphicsPolygonItem(boundingPolygonF)
        self.boundingPolygon.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
        self.boundingPolygon.setPen(QtGui.QPen(QtCore.Qt.black))
        self.scene.addItem(self.boundingPolygon)

    def printMap(self):
        areaTypesArray = [Tile.areaType for Tile in self.tiles]
        print("         " + str(areaTypesArray[:3]) + "\n")
        print("     " + str(areaTypesArray[3:7]) + "\n")
        print(str(areaTypesArray[7:12]) + "\n")
        print("     " + str(areaTypesArray[12:16]) + "\n")
        print("         " + str(areaTypesArray[16:]) + "\n")
        print("TILE NUMBERS: " + str(self.tileNumbers))

#################################################################################################################
#################################					PLAYER CLASS 				#################################
#################################################################################################################

class Player:
    def __init__(self, color, corner):
        self.supplyCards = {"BRICK":0,"GRAIN":0,"LUMBER":0,"ORE":0,"WOOL":0} #TODO nejaky karty na zacatku ma
        self.actionCards = None
        self.color = color #barva figurky
        self.corner = corner
        self.roads = 15
        self.settlements = 5
        self.cities = 4
        self.items = []

    def addSupplyCard(self, supplyType):
        self.supplyCards[supplyType] += 1

    def removeSupplyCard(self, supplyType):
        if (self.supplyCards[supplyType] == 0):
            print("ERROR: no more cards of this type (" + supplyType + ")")
        else:
            self.supplyCards[supplyType] -= 1

    def addActionCard(self, supplyType):
        pass

    def removeActionCard(self, supplyType):
        pass

    #vsechny akce ktere muze hrac provest
    def buyRoad(self):
        pass

    def buyVillage(self):
        pass

    def buyCity(self):
        pass

    def printStatus(self):
        print("."*80)
        print("Player number " + str(self.corner + 1) + " (color: " + self.color + ")")
        self.printSupplyCards()
        self.printBuildings()

    def printSupplyCards(self):
        print("BRICKS: " + str(self.supplyCards["BRICK"]))
        print("GRAIN: " + str(self.supplyCards["GRAIN"]))
        print("LUMBER: " + str(self.supplyCards["LUMBER"]))
        print("ORE: " + str(self.supplyCards["ORE"]))
        print("WOOL: " + str(self.supplyCards["WOOL"]))

    def printBuildings(self):
        pass

    def drawPlayerUI(self, scene):
    	boundingPolygon = QtGui.QPolygon([QtCore.QPoint(0,0),QtCore.QPoint(0,300), QtCore.QPoint(300,300),\
    						QtCore.QPoint(300,0)])
        boundingPolygonF = QtGui.QPolygonF(boundingPolygon)
        self.boundingPolygon = QtGui.QGraphicsPolygonItem(boundingPolygonF)
        self.boundingPolygon.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
        self.boundingPolygon.setPen(QtGui.QPen(QtCore.Qt.black))
        scene.addItem(self.boundingPolygon)

        self.items.append(ButtonItem(scene, QTtoART(x=0), QTtoART(y=0), "New Game", self.boundingPolygon, True))

#################################################################################################################
#################################					GAME CLASS 					#################################
#################################################################################################################

class Game: 
    def __init__(self, scene):
        self.scene = scene
        self.supplyCards = self.createStartingSupplyCards() #pole karet zasob
        self.actionCards = self.createStartingActionCards()
        self.map = self.createDefaultMap() #TODO zmenit
        self.players = self.createPlayers(4) #TODO zmenit
        self.items = []
    
    	#na vymazani pak
        self.rect = QtGui.QGraphicsRectItem(0,0,2000, 1200)
        self.rect.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
        self.scene.addItem(self.rect)
        
        self.map.drawMap()
        self.players["RED"].drawPlayerUI(self.scene)
        # self.image = QtGui.QGraphicsPixmapItem(QtGui.QPixmap(QtGui.QImage(imagesPath + "spade_ace.png")), scene=self.scene)
        # self.image.setPos(0,0)
        # self.pixmap = QtGui.QPixmap("/home/bados/catkin_ws/src/my_gui/src/spade_ace.png")
        # self.pixItem = QtGui.QGraphicsPixmapItem(self.pixmap)
        # self.scene.addItem(self.pixItem)

        # self.button = items.ButtonItem(self.scene, 0.1, 0.2, "BUTTON", None, True, scale=3)
        # self.label = items.DialogItem(self.scene, 0.5, 0.5, "BLABLABLA", ["YES","NO"], None)
        # self.descItem = items.DescItem(self.scene, 0.4, 0.4, self.button)
        # self.descItem.set_content("TESTOVACI TEXT", 3)
        # self.polygon = items.PolygonItem(self.scene, "NECO", [[0,1],[0,0.5]], [[1,1],[1.5,1.5],[0,0]])
        # #self.list = ListItem(self.scene, 0, 0.5, 0.1, ["Item1","Item2","Item3"])

    def createPlayers(self, count):
        colors = ["BLUE", "GREEN", "RED", "YELLOW"]
        playerPool = {}
        for n in range(count):  
            playerPool[colors[n]] = Player(colors[n], n)
        return playerPool

    def createDefaultMap(self):
        defaultMap = Map(self.scene)
        defaultMap.addTile("MOUNTAINS",[0,0],0)
        defaultMap.addTile("PASTURE",[1,0],1)
        defaultMap.addTile("FOREST",[2,0],2)
        defaultMap.addTile("FIELDS",[0,1],3)
        defaultMap.addTile("HILLS",[1,1],4)
        defaultMap.addTile("PASTURE",[2,1],5)
        defaultMap.addTile("HILLS",[3,1],6)
        defaultMap.addTile("FIELDS",[-1,2],7)
        defaultMap.addTile("FOREST",[0,2],8)
        defaultMap.addTile("DESERT",[1,2],9)
        defaultMap.addTile("FOREST",[2,2],10)
        defaultMap.addTile("MOUNTAINS",[3,2],11)
        defaultMap.addTile("FOREST",[0,3],12)
        defaultMap.addTile("MOUNTAINS",[1,3],13)
        defaultMap.addTile("FIELDS",[2,3],14)
        defaultMap.addTile("PASTURE",[3,3],15)
        defaultMap.addTile("HILLS",[0,4],16)
        defaultMap.addTile("FIELDS",[1,4],17)
        defaultMap.addTile("PASTURE",[2,4],18)
        defaultMap.setTileNumbers([10,2,9,12,6,4,10,9,11,0,3,8,8,3,4,5,5,6,11])
        return defaultMap

    def createMap(self):
        #vytvori mapu
        pass

    def createStartingSupplyCards(self):
        #vytvoreni balicku jednotlivych karet zasob
        return {"BRICK":19,"GRAIN":19,"LUMBER":19,"ORE":19,"WOOL":19}

    def addSupplyCard(self, supplyType):
        if (self.supplyCards[supplyType] == 19):
            print("ERROR: more than 19 cards of this type (" + supplyType + ") in the game")
        else:
            self.supplyCards[supplyType] += 1

    def giveSupplyToPlayer(self, color, supplyType):
        if (self.supplyCards[supplyType] == 0):
            #TODO uz nejsou karty -- nikdo nedostane karty tohohle druhu
            print("ERROR: no more cards of this type (" + supplyType + ")")
        else:
            self.supplyCards[supplyType] -= 1
            self.players[color].addSupplyCard(supplyType)

    def createStartingActionCards(self):
        #vytvoreni balicku akcnich karet
        pass

    def distributeSupplies(self, number):
        #podle cisla se rozdeli suroviny hracum
        #TODO cisla tileNumbers musi odpovidat cislum tiles!!!
        indices = [i for i, x in enumerate(self.map.tileNumbers) if x == number]
        

    def nextTurn(self):
        #hod kostkama
        #rozdeleni surovin
        #obchodovani
        #stavba
        pass

    def printGameStatus(self):
        print("-"*80)
        print("Turn xxx")
        print("-"*80)
        self.printMap()
        print("-"*80)
        self.printSupplyCards()
        print("-"*80)
        self.printPlayers()

    def printPlayers(self):
        for player in self.players:
            player.printStatus()

    def printSupplyCards(self):
        print("BRICKS: " + str(self.supplyCards["BRICK"]))
        print("GRAIN: " + str(self.supplyCards["GRAIN"]))
        print("LUMBER: " + str(self.supplyCards["LUMBER"]))
        print("ORE: " + str(self.supplyCards["ORE"]))
        print("WOOL: " + str(self.supplyCards["WOOL"]))

    def printMap(self):
        self.map.printMap()

def main():
    game = Game()
    game.distributeSupplies(5)
    game.printGameStatus()

if __name__ == "__main__":
    main()
