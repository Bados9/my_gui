import sys
import os
import signal
import rospy
import math
import gamePieces
from PyQt4 import QtGui, QtCore, QtNetwork
from art_projected_gui.helpers import ProjectorHelper
import items

class Tile:
    def __init__(self, areaType, position): #mozna zbytecny
        self.areaType = areaType
        self.adjacentRoads = [None]*6
        self.adjacentBuildings = [None]*6

class Map:
    def __init__(self):
        self.tiles = [None] * 19
        #self.tiles = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]
        self.tileNumbers = [None] * 19 #seradi se do zakladu, pak bude mozno posunout
        self.roads = []
        self.places = []

    def addTile(self, areaType, position):
        self.tiles[position] = Tile(areaType, position)
    
    def setTileNumbers(self, tileNumbers):
        self.tileNumbers = tileNumbers

    def addCity(self, city):
        pass

    def addVillage(self, city):
        pass

    def addRoad(self, city):
        pass

    def printMap(self):
        areaTypesArray = [Tile.areaType for Tile in self.tiles]
        print("         " + str(areaTypesArray[:3]) + "\n")
        print("     " + str(areaTypesArray[3:7]) + "\n")
        print(str(areaTypesArray[7:12]) + "\n")
        print("     " + str(areaTypesArray[12:16]) + "\n")
        print("         " + str(areaTypesArray[16:]) + "\n")
        print("TILE NUMBERS: " + str(self.tileNumbers))


class Player:
    def __init__(self, color, corner):
        self.supplyCards = {"BRICK":0,"GRAIN":0,"LUMBER":0,"ORE":0,"WOOL":0} #TODO nejaky karty na zacatku ma
        self.actionCards = None
        self.color = color #barva figurky
        self.corner = corner
        self.roads = 15
        self.settlements = 5
        self.cities = 4


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

class Game: 
    def __init__(self, scene):
        self.scene = scene
        self.supplyCards = self.createStartingSupplyCards() #pole karet zasob
        self.actionCards = self.createStartingActionCards()
        self.map = self.createDefaultMap() #TODO zmenit
        self.players = self.createPlayers(4) #TODO zmenit
        self.items = []
    
        self.rect = QtGui.QGraphicsRectItem(0,0,50,50)
        self.rect.setBrush(QtGui.QBrush(QtCore.Qt.green))
        self.scene.addItem(self.rect)

        polygon = QtGui.QPolygon([QtCore.QPoint(287,250),QtCore.QPoint(200,300), \
                    QtCore.QPoint(113,250),QtCore.QPoint(113,150),QtCore.QPoint(200,100), QtCore.QPoint(287,150)])
        polygonF = QtGui.QPolygonF(polygon)
        self.polygon = QtGui.QGraphicsPolygonItem(polygonF)
        self.polygon.setBrush(QtGui.QBrush(QtCore.Qt.green))
        self.polygon.setPen(QtGui.QPen(QtCore.Qt.red))
        self.scene.addItem(self.polygon)

        print("souradnice:" + "\n" + \
                    self.hex_corner([200,200],100,0) + "\n" + \
                    self.hex_corner([200,200],100,1) + "\n" + \
                    self.hex_corner([200,200],100,2) + "\n" + \
                    self.hex_corner([200,200],100,3) + "\n" + \
                    self.hex_corner([200,200],100,4) + "\n" + \
                    self.hex_corner([200,200],100,5) + "\n")

        self.image = QtGui.QGraphicsPixmapItem(QtGui.QPixmap(QtGui.QImage("/home/bados/catkin_ws/src/my_gui/src/spade_ace.png")), scene=self.scene)
        self.image.setPos(800,200)
        # self.pixmap = QtGui.QPixmap("/home/bados/catkin_ws/src/my_gui/src/spade_ace.png")
        # self.pixItem = QtGui.QGraphicsPixmapItem(self.pixmap)
        # self.scene.addItem(self.pixItem)

        # self.button = items.ButtonItem(self.scene, 0.1, 0.2, "BUTTON", None, True, scale=3)
        # self.label = items.DialogItem(self.scene, 0.5, 0.5, "BLABLABLA", ["YES","NO"], None)
        # self.descItem = items.DescItem(self.scene, 0.4, 0.4, self.button)
        # self.descItem.set_content("TESTOVACI TEXT", 3)
        # self.polygon = items.PolygonItem(self.scene, "NECO", [[0,1],[0,0.5]], [[1,1],[1.5,1.5],[0,0]])
        # #self.list = items.ListItem(self.scene, 1, 0.5, 0.1, ["Item1","Item2","Item3"])

    def hex_corner(self, center, size, i):
        angle_deg = 60 * i   + 30
        angle_rad = math.pi / 180 * angle_deg
        return str(center[0] + size * math.cos(angle_rad)) + " " + str(center[1] + size * math.sin(angle_rad))

    def createPlayers(self, count):
        colors = ["BLUE", "GREEN", "RED", "YELLOW"]
        playerPool = [None]*count
        for n in range(count):  
            playerPool[n] = Player(colors[n], n)
        return playerPool

    def createDefaultMap(self):
        defaultMap = Map()
        defaultMap.addTile("MOUNTAINS",0)
        defaultMap.addTile("PASTURE",1)
        defaultMap.addTile("FOREST",2)
        defaultMap.addTile("FIELDS",3)
        defaultMap.addTile("HILLS",4)
        defaultMap.addTile("PASTURE",5)
        defaultMap.addTile("HILLS",6)
        defaultMap.addTile("FIELDS",7)
        defaultMap.addTile("FOREST",8)
        defaultMap.addTile("DESERT",9)
        defaultMap.addTile("FOREST",10)
        defaultMap.addTile("MOUNTAINS",11)
        defaultMap.addTile("FOREST",12)
        defaultMap.addTile("MOUNTAINS",13)
        defaultMap.addTile("FIELDS",14)
        defaultMap.addTile("PASTURE",15)
        defaultMap.addTile("HILLS",16)
        defaultMap.addTile("FIELDS",17)
        defaultMap.addTile("PASTURE",18)
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

    def giveSupplyToPlayer(self, corner, supplyType):
        if (self.supplyCards[supplyType] == 0):
            #TODO uz nejsou karty -- nikdo nedostane karty tohohle druhu
            print("ERROR: no more cards of this type (" + supplyType + ")")
        else:
            self.supplyCards[supplyType] -= 1
            self.players[corner].addSupplyCard(supplyType)

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