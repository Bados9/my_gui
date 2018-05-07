# -*- coding: utf-8 -*-

import sys
import os
import signal
import time
import rospy
import rospkg
import math
from random import randint
from PyQt4 import QtGui, QtCore, QtNetwork
from art_projected_gui.helpers import ProjectorHelper
from items import *
from gamePieces import *
from recognizer import Recognizer
from art_msgs.msg import Touch


rospack = rospkg.RosPack()
imagesPath = rospack.get_path('my_gui') + '/src/images/'

def QTtoART(x=None,y=None):
    if y is None:
        return x/2000.0
    return 0.60 - y/2000.0

adjBuildings = {0 : [ 0, 1, 2,10, 9, 8],
                1 : [ 2, 3, 4,12,11,10],
                2 : [ 4, 5, 6,14,13,12],
                3 : [ 7, 8, 9,19,18,17],
                4 : [ 9,10,11,21,20,19],
                5 : [11,12,13,23,22,21],
                6 : [13,14,15,25,24,23],
                7 : [16,17,18,29,28,27],
                8 : [18,19,20,31,30,29],
                9 : [20,21,22,33,32,31],
               10 : [22,23,24,35,34,33],
               11 : [24,25,26,37,36,35],
               12 : [28,29,30,40,39,38],
               13 : [30,31,32,42,41,40],
               14 : [32,33,34,44,43,42],
               15 : [34,35,36,46,45,44],
               16 : [39,40,41,49,48,47],
               17 : [41,42,43,51,50,49],
               18 : [43,44,45,53,52,51]}

adjRoads = {    0 : [ 6, 0, 1, 7,12,11],
                1 : [ 7, 2, 3, 8,14,13],
                2 : [ 8, 4, 5, 9,16,15],
                3 : [18,10,11,19,25,24],
                4 : [19,12,13,20,27,26],
                5 : [20,14,15,21,29,28],
                6 : [21,16,17,22,31,30],
                7 : [33,23,24,34,40,39],
                8 : [34,25,26,35,42,41],
                9 : [35,27,28,36,44,43],
               10 : [36,29,30,37,46,45],
               11 : [37,31,32,38,48,47],
               12 : [49,40,41,50,55,54],
               13 : [50,42,43,51,57,56],
               14 : [51,44,45,52,59,58],
               15 : [52,46,47,53,61,60],
               16 : [62,55,56,63,67,66],
               17 : [63,57,58,64,69,68],
               18 : [64,59,60,65,71,70]}

crossroads = {  0 : [0,6],
                1 : [0,1],
                2 : [1,2,7],
                3 : [2,3],
                4 : [3,4,8],
                5 : [4,5],
                6 : [5,9],
                7 : [10,18],
                8 : [6,10,11],
                9 : [11,12,19],
               10 : [7,12,13],
               11 : [13,14,20],
               12 : [8,14,15],
               13 : [15,16,21],
               14 : [9,16,17],
               15 : [17,22],
               16 : [23,33],
               17 : [18,23,24],
               18 : [24,25,34],
               19 : [19,25,26],
               20 : [26,27,35],
               21 : [20,27,28],
               22 : [28,29,36],
               23 : [21,29,30],
               24 : [30,31,37],
               25 : [22,31,32],
               26 : [32,38],
               27 : [33,39],
               28 : [39,40,49],
               29 : [34,40,41],
               30 : [41,42,50],
               31 : [35,42,43],
               32 : [43,44,51],
               33 : [36,44,45],
               34 : [45,46,52],
               35 : [37,46,47],
               36 : [47,48,53],
               37 : [38,48],
               38 : [49,54],
               39 : [54,55,62],
               40 : [50,55,56],
               41 : [56,57,63],
               42 : [51,57,58],
               43 : [58,59,64],
               44 : [52,59,60],
               45 : [60,61,65],
               46 : [53,61],
               47 : [62,66],
               48 : [66,67],
               49 : [63,67,68],
               50 : [68,69],
               51 : [64,69,70],
               52 : [70,71],
               53 : [65,71]}

areaSupplyDict = {  "MOUNTAINS" : "ORE",
                    "FOREST" : "LUMBER",
                    "HILLS" : "BRICK",
                    "PASTURE" : "WOOL",
                    "FIELDS" : "GRAIN",
                    "DESERT" : "NONE",
                    "NONE" : "NONE"
}

def doNothing(button=None):
    pass

def whereIsIt(point):
    if point.x() > 1000:
        if point.y() > 600:
            return "green"
        else:
            return "red"
    else:
        if point.y() > 600:
            return "blue"
        else:
            return "yellow"

def canIBuildThereSettlement(place, game):
    if game.map.places[place] is not None:
        return False
    roads = crossroads[place]
    neighbours = []
    for key, value in crossroads.items():
        a = set(roads)
        b = set(value)
        if len(a.intersection(b)) > 0:
            neighbours.append(key)
    neighbours.remove(place)       
    for neighbour in neighbours:
        if game.map.places[neighbour] is not None:
            return False
    if game.turnNumber > len(game.colors)*2-1:
        for road in crossroads[place]:
            if game.map.roads[road] is not None and game.map.roads[road].color == game.colors[game.turnNumber%len(game.colors)]:
                return True
        return False
    else:
        return True
    
def canIBuildThereCity(place, game):
    if game.map.places[place] is not None and game.map.places[place].color == game.colors[game.turnNumber%len(game.colors)]:
        return True
    return False

def canIBuildThereRoad(road, game):
    if game.map.roads[road] is not None:
        return False
    neighbours = []
    for key, value in crossroads.items():
        if road in value:
            neighbours.append(key)
    for neighbour in neighbours:
        if game.map.places[neighbour] is not None and game.map.places[neighbour].color == \
        game.colors[game.turnNumber%len(game.colors)]:
            return True
    
    adjacentRoads = []
    for key, value in crossroads.items():
        if road in value:
            adjacentRoads.extend(value)
    adjacentRoads.remove(road)
    for adjacentRoad in adjacentRoads:
        if game.map.roads[adjacentRoad] is not None and game.map.roads[adjacentRoad].color == \
        game.colors[game.turnNumber%len(game.colors)]:
            return True
    return False

def hexCorners(center, size, orientation=None):
        corners = []
        for i in range(6):
            angle_deg = 60 * i   + 30
            if orientation is not None: # hack for bounding hexagon
                angle_deg -= 30
            angle_rad = math.pi / 180 * angle_deg
            corners.append([center[0] + size * math.cos(angle_rad),center[1] + size * math.sin(angle_rad)])
        return [corners[3], corners[4], corners[5], corners[0], corners[1], corners[2]]

#################################################################################################################
#################################                   TILE CLASS                  #################################
#################################################################################################################

class Tile:
    def __init__(self, areaType, position, index, scene, myMap, editor=False, editorInstance=None):
        self.scene = scene
        self.areaType = areaType
        self.number = -1
        self.supplyType = areaSupplyDict[self.areaType]
        self.adjacentBuildings = adjBuildings[index]
        self.adjacentRoads = adjRoads[index]
        self.position = position #position on map
        self.index = index
        self.coords = [0,0] #in pixels
        self.polygon = None
        self.map = myMap
        self.focus = False
        self.crossButtons = [None] * 6
        self.roadButtons = [None] * 6
        self.roadIcons = [None] * 6
        self.cityIcons = [None] * 6
        self.settlementIcons = [None] * 6
        self.editor = editor
        self.robberIcon = None
        self.editorInstance = editorInstance

    def setNumber(self, number):
        self.number = number

    def drawTileBtn(self, size):
        vertSkip = size*2
        horSkip = math.sqrt(3)/2 * vertSkip

        self.coords = [910-vertSkip,510-2*horSkip]
        self.coords[1] += self.position[1]*horSkip
        self.coords[0] += self.position[0]*vertSkip
        if (self.position[1] & 1):
            self.coords[0] -= vertSkip*0.5
       
        if self.editor == False:
            self.tileBtn = ButtonItem(self.scene, QTtoART(x=self.coords[0]), QTtoART(y=self.coords[1]), "", None,\
                self.changeFocus, image_path=imagesPath + self.areaType + ".png", scale=8.5,\
                background_color=QtCore.Qt.transparent)
            self.tileBtn.h = 180
        else:
            self.tileBtn = ButtonItem(self.scene, QTtoART(x=self.coords[0]), QTtoART(y=self.coords[1]), "", None,\
                self.setAreaType, image_path=imagesPath + self.areaType + ".png", scale=8.5,\
                background_color=QtCore.Qt.transparent, data=self.index)
            self.tileBtn.h = 180

        
        if self.number > 0:
            XOffset = 40 
            if self.number > 9:
                XOffset = 30
            self.numberBtn = ButtonItem(self.scene, QTtoART(x=self.coords[0]+XOffset), QTtoART(y=self.coords[1]+30), \
                str(self.number), None, self.changeFocus, scale=3,\
                background_color=QtCore.Qt.transparent, data=self.index)
        elif self.number == 0:
            self.numberBtn = ButtonItem(self.scene, QTtoART(x=self.coords[0]+30), QTtoART(y=self.coords[1]+30), \
               "", None, self.changeFocus, scale=3,\
                background_color=QtCore.Qt.transparent, data=self.index)
            self.numberBtn.w = 105
            self.drawRobber()
        else:
            self.numberBtn = None

        if self.editor == True and self.numberBtn != None:
            self.numberBtn.set_cb(self.setAreaType)

    def deleteRobber(self, button=None):
        self.scene.removeItem(self.robberIcon)
        self.robberIcon = None

    def drawRobber(self, button=None):
        self.robberIcon = ButtonItem(self.scene, QTtoART(x=self.coords[0]+40), QTtoART(y=self.coords[1]+30), \
                str(self.number), None, self.changeFocus, scale=3, image_path=imagesPath + "butt_crossroad_black.png",\
                background_color=QtCore.Qt.transparent)

    def setAreaType(self, button=None):
        print("tile areaType changed to " + str(self.map.selectedAreaType))
        self.areaType = self.map.selectedAreaType
        self.editorInstance.checkCounts()
        self.drawTileBtn(75)

    def set_cb_toChangeFocus(self):
        self.tileBtn.set_cb(self.changeFocus)
        if self.numberBtn != None:
            self.numberBtn.set_cb(self.changeFocus)

    def set_cb_toRobber(self):
        self.tileBtn.set_cb(self.map.moveRobber)
        if self.numberBtn != None:
            self.numberBtn.set_cb(self.map.moveRobber)

    def updateTile(self):
            if self.index in [4,10,13]:
                self.unsetFocus()
                return
            self.unsetFocus()
            corners = hexCorners([self.coords[0]+55, self.coords[1]+55], 85)
            for index, corner in enumerate(corners):
                if self.map.places[self.adjacentBuildings[index]] is not None:
                    if self.map.places[self.adjacentBuildings[index]].type == "settlement":
                        if self.settlementIcons[index] is None:
                            self.settlementIcons[index] = (ButtonItem(self.scene, QTtoART(x=corners[index][0]), QTtoART(y=corners[index][1]), "", \
                            None, self.buildCity, scale = 3, background_color=QtCore.Qt.transparent, image_path = imagesPath+"village_" + \
                            self.map.places[self.adjacentBuildings[index]].color + ".png", data=index))
                        self.settlementIcons[index].h = 55

                    elif self.map.places[self.adjacentBuildings[index]].type == "city":
                        if self.cityIcons[index] is None:
                            self.cityIcons[index] = (ButtonItem(self.scene, QTtoART(x=corners[index][0]), QTtoART(y=corners[index][1]), "", \
                                None, doNothing, scale = 3, background_color=QtCore.Qt.transparent, image_path = imagesPath+"city_" + \
                                self.map.places[self.adjacentBuildings[index]].color + ".png"))
                            self.cityIcons[index].h = 55

            corners = hexCorners([self.coords[0]+75, self.coords[1]+55], 75, True)

            for index, corner in enumerate(corners):
                if self.map.roads[self.adjacentRoads[index]] is not None:
                    if self.roadIcons[index] is None:
                        #print("davam cestu na roh " + str(index) + " u policka " + str(self.index))
                        self.roadIcons[index] = (ButtonItem(self.scene, QTtoART(x=corners[index][0]), QTtoART(y=corners[index][1]),"", \
                            None, doNothing, background_color=QtCore.Qt.transparent, image_path = imagesPath+"road_"
                             + self.map.roads[self.adjacentRoads[index]].color + ".png"))
                        self.roadIcons[index].h = 55

            for index, settlement in enumerate(self.settlementIcons):
                if self.cityIcons[index] is not None:
                    if settlement is not None:
                        self.scene.removeItem(settlement)
                        self.settlementIcons[index] = None

    def changeFocus(self, button=None):
        if self.focus == True:
            self.unsetFocus()
        else:
            self.setFocus()
            
    def setFocus(self):
        if self.map.buildMode == "city" or self.map.buildMode == "settlement": #jeste nevim jak s mestem
            corners = hexCorners([self.coords[0]+55, self.coords[1]+55], 85)
            for index, corner in enumerate(corners):
                if self.map.places[self.adjacentBuildings[index]] is None:
                    if canIBuildThereSettlement(self.adjacentBuildings[index], self.map.game):
                        self.crossButtons[index] = ButtonItem(self.scene, QTtoART(x=corners[index][0]), QTtoART(y=corners[index][1]),"", \
                            None, self.buildSettlement, scale = 3, background_color=QtCore.Qt.transparent, data=index, \
                            image_path = imagesPath+"butt_crossroad_red.png")
                        self.crossButtons[index].h = 55

        elif self.map.buildMode == "road":
            corners = hexCorners([self.coords[0]+55, self.coords[1]+55], 75, True)
            for index, corner in enumerate(corners):
                if self.map.roads[self.adjacentRoads[index]] is None:
                    if canIBuildThereRoad(self.adjacentRoads[index], self.map.game):
                        self.roadButtons[index] = ButtonItem(self.scene, QTtoART(x=corners[index][0]), QTtoART(y=corners[index][1]),"", \
                            None, self.buildRoad, scale = 3, background_color=QtCore.Qt.transparent, data=index, \
                            image_path = imagesPath+"butt_crossroad_red.png")

                        self.roadButtons[index].h = 55
        self.focus = True

    def unsetFocus(self): 
        if self.focus == True:      
            for butt in self.crossButtons:
                if butt is not None:
                    self.scene.removeItem(butt)
            self.crossButtons = [None] * 6

            for butt in self.roadButtons:
                if butt is not None:
                    self.scene.removeItem(butt)
            self.roadButtons = [None] * 6
        self.focus = False

    def buildSettlement(self, button=None):
        #print("kliknulo se na krizovatku c." + str(self.adjacentBuildings[button.data]))
        if self.map.buildMode == "settlement":
            self.map.buildSettlement(self.adjacentBuildings[button.data])

    def buildRoad(self, button=None):
        #print("kliknulo se na cestu c. " + str(self.adjacentRoads[button.data]))
        if self.map.buildMode == "road":
            self.map.buildRoad(self.adjacentRoads[button.data])

    def buildCity(self, button=None):
        if self.map.buildMode == "city":
            self.map.buildCity(self.adjacentBuildings[button.data])
            if (self.settlementIcons[button.data]) is not None:
                self.scene.removeItem(self.settlementIcons[button.data])

    def giveSupplies(self, game):
        for index in self.adjacentBuildings:
            if self.map.places[index] is not None:
                game.giveSupplyToPlayer(self.map.places[index].color, self.supplyType)
                if self.map.places[index].type == "city":
                    game.giveSupplyToPlayer(self.map.places[index].color, self.supplyType)

#################################################################################################################
#################################                   MAP CLASS                   #################################
#################################################################################################################

class Map:
    def __init__(self, scene, editorInstance=None, game=None, editor=False):
        self.scene = scene
        self.game = game
        self.tiles = [None] * 19
        self.tileNumbers = [None] * 19 #seradi se do zakladu, pak bude mozno posunout
        self.roads = [None] * 72
        self.places = [None]* 54
        self.buildMode = "settlement"
        self.selectedAreaType = "NONE"
        self.editor = editor
        self.robber = 9
        self.editorInstance = editorInstance

    def changeSelectedAreaType(self, button=None):
        print("selectedArea changed to " + str(button.data))
        self.selectedAreaType = button.data

    def addTile(self, areaType, position, index):
        if self.editor == False:
            self.tiles[index] = Tile(areaType, position, index, self.scene, self)
        else:
            self.tiles[index] = Tile(areaType, position, index, self.scene, self, True, self.editorInstance)
    
    def changeToNormalMode(self):
        for tile in self.tiles:
            tile.set_cb_toChangeFocus()

    def changeToRobberMode(self):
        for tile in self.tiles:
            tile.set_cb_toRobber()
        self.game.announcementArea.set_content("     MOVE ROBBER", 3)

    def moveRobber(self, button=None):
        #print("presouvame z " + str(self.robber) + " na " + str(button.data))
        self.tiles[self.robber].deleteRobber()
        self.robber = button.data
        self.tiles[self.robber].drawRobber()
        self.changeToNormalMode()

    def setTileNumbers(self, button=None, arg=None):
        if isinstance(arg, (list,)):
            self.tileNumbers = arg
        else:
            startingCorner = button.data
            settingDict = { 0 : [0,3,7,12,16,17,18,15,11,6,2,1,4,8,13,14,10,5,9],
                            2 : [2,1,0,3,7,12,16,17,18,15,11,6,5,4,8,13,14,10,9],
                            7 : [7,12,16,17,18,15,11,6,2,1,0,3,8,13,14,10,5,4,9],
                            11: [11,6,2,1,0,3,7,12,16,17,18,15,10,5,4,8,13,14,9],
                            16: [16,17,18,15,11,6,2,1,0,3,7,12,13,14,10,5,4,8,9],
                            18: [18,15,11,6,2,1,0,3,7,12,16,17,14,10,5,4,8,13,9]}
            numberList = [5,2,6,3,8,10,9,12,11,4,8,10,9,4,5,6,3,11]
            for i in range(19):
                if self.tiles[settingDict[startingCorner][i]].areaType != "DESERT":
                    self.tileNumbers[settingDict[startingCorner][i]] = numberList[i]
                else:
                    numberList.insert(i, 0)
                    pass

        for tile in self.tiles:
            tile.setNumber(self.tileNumbers[tile.index])        

    def buildSettlement(self, number):
        self.places[number] = Building("settlement", self.game.colors[self.game.turnNumber%len(self.game.colors)])
        self.updateMap()
        self.game.players[self.game.colors[self.game.turnNumber%len(self.game.colors)]].buildings["settlements"] -= 1
        if self.game.turnNumber <= len(self.game.colors)*2-1:
            self.game.nextTurn()
        else:
            self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "BRICK")
            self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "LUMBER")
            self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "WOOL")
            self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "GRAIN")
            self.game.players[self.game.colors[self.game.turnNumber%len(self.game.colors)]].updatePlayerUI()

        self.game.checkGameEnd()

    def buildCity(self, number):
        if self.game.turnNumber <= len(self.game.colors)*2-1:
            return

        self.places[number] = Building("city", self.game.colors[self.game.turnNumber%len(self.game.colors)])
        self.updateMap()
        self.game.players[self.game.colors[self.game.turnNumber%len(self.game.colors)]].buildings["cities"] -= 1
        self.game.players[self.game.colors[self.game.turnNumber%len(self.game.colors)]].buildings["settlements"] += 1

        self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "ORE")
        self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "ORE")
        self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "ORE")
        self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "GRAIN")
        self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "GRAIN")
        self.game.players[self.game.colors[self.game.turnNumber%len(self.game.colors)]].updatePlayerUI()

        self.game.checkGameEnd()

    def buildRoad(self, number):
        self.roads[number] = Building("road", self.game.colors[self.game.turnNumber%len(self.game.colors)])
        self.updateMap()
        self.game.players[self.game.colors[self.game.turnNumber%len(self.game.colors)]].buildings["roads"] -= 1
        if self.game.turnNumber <= len(self.game.colors)*2-1:
            self.game.nextTurn()
        else:
            self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "BRICK")
            self.game.takeSupplyFromPlayer(self.game.colors[self.game.turnNumber%len(self.game.colors)], "LUMBER")
            self.game.players[self.game.colors[self.game.turnNumber%len(self.game.colors)]].updatePlayerUI()

    def updateMap(self):
        for tile in self.tiles:
            tile.updateTile()

    def drawMap(self):
        size = 75
        
        for tile in self.tiles:
            tile.drawTileBtn(size)

#################################################################################################################
#################################                   PLAYER CLASS                #################################
#################################################################################################################

class Player:
    def __init__(self, color, corner, game):
        self.game = game
        self.supplyCards = {"BRICK":10,"GRAIN":10,"LUMBER":10,"ORE":10,"WOOL":10} 
        self.actionCards = None
        self.color = color #barva figurky
        self.corner = corner
        self.buildings = {"roads":15, "settlements":5, "cities":4}
        self.items = []
        self.cardItems = []
        self.supplyLabels = {}
        self.buildingLabels = {}
        self.points = 0

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

    def countPoints(self):
        settlementsBuild = 5 - self.buildings["settlements"]
        citiesBuild = 4 - self.buildings["cities"]
        self.points = settlementsBuild + 2*citiesBuild

    def buyRoad(self, button=None):
        if self.supplyCards["BRICK"] < 1 or self.supplyCards["LUMBER"] < 1:
            self.game.announcementArea.set_content("You do not have enough resources", 3)
            self.game.announcementArea2.set_content("You do not have enough resources", 3)
            return
        if self.buildings["roads"] == 0:
            self.game.announcementArea.set_content("You do not have any roads left", 3)
            self.game.announcementArea2.set_content("You do not have any roads left", 3)
            return

        buildingPermission = False
        for road in range(72):
            if canIBuildThereRoad(road, self.game) == True:
                buildingPermission = True
                break
        if buildingPermission == False:
            self.game.announcementArea.set_content("     No place to build", 3)
            self.game.announcementArea2.set_content("     No place to build", 3)
            return

        self.game.map.buildMode = "road"
        self.game.announcementArea.set_content("        Choose tile", 3)
        self.game.announcementArea2.set_content("        Choose tile", 3)

    def buySettlement(self, button=None):
        if self.supplyCards["BRICK"] < 1 or self.supplyCards["LUMBER"] < 1 or self.supplyCards["WOOL"] < 1 or\
        self.supplyCards["GRAIN"] < 1:
            self.game.announcementArea.set_content("You do not have enough resources", 2)
            self.game.announcementArea2.set_content("You do not have enough resources", 2)
            return
        if self.buildings["settlements"] == 0:
            self.game.announcementArea.set_content("You do not have any settlements left", 2)
            self.game.announcementArea2.set_content("You do not have any settlements left", 2)
            return

        buildingPermission = False
        for place in range(54):
            if canIBuildThereSettlement(place, self.game) == True:
                buildingPermission = True
                break
        if buildingPermission == False:
            self.game.announcementArea.set_content("     No place to build", 3)
            self.game.announcementArea2.set_content("     No place to build", 3)
            return
        
        self.game.map.buildMode = "settlement"
        self.game.announcementArea.set_content("        Choose tile", 3)
        self.game.announcementArea2.set_content("        Choose tile", 3)
        self.game.drawTurnOfPlayerAnnouncement(self.color, "village")
        
    def buyCity(self, button=None):
        if self.supplyCards["ORE"] < 3 or self.supplyCards["GRAIN"] < 2:
            self.game.announcementArea.set_content("You do not have enough resources", 2)
            self.game.announcementArea2.set_content("You do not have enough resources", 2)
            return
        if self.buildings["cities"] == 0:
            self.game.announcementArea.set_content("You do not have any cities left", 2)
            self.game.announcementArea2.set_content("You do not have any cities left", 2)
            return
        
        buildingPermission = False
        for place in range(54):
            if canIBuildThereCity(place, self.game) == True:
                buildingPermission = True
                break
        if buildingPermission == False:
            self.game.announcementArea.set_content("     No place to build", 3)
            self.game.announcementArea2.set_content("     No place to build", 3)
            return

        self.game.map.buildMode = "city"
        self.game.announcementArea.set_content("        Choose tile", 3)
        self.game.announcementArea2.set_content("        Choose tile", 3)
        self.game.drawTurnOfPlayerAnnouncement(self.color, "city")

    def setResourceForTrade(self, button):
        print("klik na " + str(button.data))
        if self.trade.resourceOne == "NONE":
            self.trade.resourceOne = button.data
            self.trade.resOneCount = 1
            self.dialog.set_caption("TRADING " + str(self.trade.resOneCount) + "x " + self.trade.resourceOne\
                + "  FOR " + str(self.trade.resTwoCount) + "x " + self.trade.resourceTwo \
                + "\n CONFIRM FIRST RESOURCE?")
        else:
            self.trade.resourceTwo = button.data
            self.trade.resTwoCount = 1
            self.dialog.set_caption("TRADING " + str(self.trade.resOneCount) + "x " + self.trade.resourceOne\
                + "  FOR " + str(self.trade.resTwoCount) + "x " + self.trade.resourceTwo \
                + "\n CONFIRM SECOND RESOURCE?")

    def tradeOffer(self, button=None):
        self.trade = Trade()
        coordsDict = {  0:[420,970,50,660,80], #[dialogX, dialogY, cardBtnX, cardBtnY, cardBtnXOffset]
                        1:[1070,970,1550,660,80],
                        2:[1580,230,1950,550,80],
                        3:[930,230,450,550,80]} 
    
        self.dialog = DialogItem(self.game.scene, QTtoART(x=coordsDict[self.corner][0]), QTtoART(y=coordsDict[self.corner][1]), "SELECT FIRST RESOURCE BY TAPPING ON CARD", ["INCREASE", "DECREASE", "CONFIRM", "CANCEL"], self.tradeDialogAnswer)
        if self.corner <= 1:           
            self.cardButtons = []
            for key, item in self.supplyCards.items():
                index = list(self.supplyCards.keys()).index(key)
                self.cardButtons.append(ButtonItem(self.game.scene, \
                    QTtoART(x=coordsDict[self.corner][2]+index*coordsDict[self.corner][4]), \
                    QTtoART(y=coordsDict[self.corner][3]), "", self.boundingRectangle, \
                    self.setResourceForTrade, background_color=QtCore.Qt.transparent, data=key))
                self.cardButtons[index].w = 70
                self.cardButtons[index].h = 110
        else:
            self.dialog.setRotation(180)
            self.cardButtons = [None]*5
            for key, item in reversed(self.supplyCards.items()):
                index = list(self.supplyCards.keys()).index(key)
                self.cardButtons[index] = ButtonItem(self.game.scene, \
                    QTtoART(x=coordsDict[self.corner][2]-index*coordsDict[self.corner][4]), \
                    QTtoART(y=coordsDict[self.corner][3]), "", self.boundingRectangle, \
                    self.setResourceForTrade, background_color=QtCore.Qt.transparent, data=key)
                self.cardButtons[index].w = 70
                self.cardButtons[index].h = 110

    def tradeSupplies(self, trade, player2):
        for i in range(trade.resOneCount):
            self.removeSupplyCard(trade.resourceOne)
            self.game.players[player2].addSupplyCard(trade.resourceOne)

        for i in range(trade.resTwoCount):
            self.game.players[player2].removeSupplyCard(trade.resourceTwo)
            self.addSupplyCard(trade.resourceTwo)

        for cardBtn in self.cardButtons:
            if cardBtn is not None:
                self.game.scene.removeItem(cardBtn)

        self.cardButtons = [None] * 5
        self.updatePlayerUI()
        self.game.players[player2].updatePlayerUI()

    def tradeDialogAnswer(self, number):
        if number == 0:
            if self.trade.resourceTwo == "NONE":
                self.trade.increaseResOne(self.supplyCards[self.trade.resourceOne])
                self.dialog.set_caption("TRADING " + str(self.trade.resOneCount) + "x " + self.trade.resourceOne\
                + "  FOR " + str(self.trade.resTwoCount) + "x " + self.trade.resourceTwo \
                + "\n CONFIRM FIRST RESOURCE?")
            else:
                self.trade.increaseResTwo()
                self.dialog.set_caption("TRADING " + str(self.trade.resOneCount) + "x " + self.trade.resourceOne\
                + "  FOR " + str(self.trade.resTwoCount) + "x " + self.trade.resourceTwo \
                + "\n CONFIRM SECOND RESOURCE?")
        elif number == 1:
            if self.trade.resourceTwo == "NONE":
                self.trade.decreaseResOne()
                self.dialog.set_caption("TRADING " + str(self.trade.resOneCount) + "x " + self.trade.resourceOne\
                + "  FOR " + str(self.trade.resTwoCount) + "x " + self.trade.resourceTwo \
                + "\n CONFIRM FIRST RESOURCE?")
            else:
                self.trade.decreaseResTwo()
                self.dialog.set_caption("TRADING " + str(self.trade.resOneCount) + "x " + self.trade.resourceOne\
                + "  FOR " + str(self.trade.resTwoCount) + "x " + self.trade.resourceTwo \
                + "\n CONFIRM SECOND RESOURCE?")
        elif number == 2:
            if self.trade.resourceTwo == "NONE":
                self.dialog.set_caption("SELECT SECOND RESOURCE")
            else:
                if whereIsIt(self.dialog.pos()) == self.color:
                    self.dialog.set_caption("TRADING " + str(self.trade.resOneCount) + "x " + self.trade.resourceOne\
                    + "  FOR " + str(self.trade.resTwoCount) + "x " + self.trade.resourceTwo \
                    + "\n WAITING FOR AGREEMENT FROM SECOND PLAYER")
                else:
                    self.tradeSupplies(self.trade, whereIsIt(self.dialog.pos()))
                    self.game.scene.removeItem(self.dialog)
                    self.dialog = None    
                    self.trade = None       
        else:
            self.game.scene.removeItem(self.dialog)
            for cardBtn in self.cardButtons:
                if cardBtn is not None:
                    self.game.scene.removeItem(cardBtn)
            self.cardButtons = [None] * 5
            self.dialog = None

    def drawPlayerUI(self, scene):
        if self.corner == 0 or self.corner == 2:
            self.boundingRectangle = QtGui.QGraphicsRectItem(0, 650, 600, 550)
            self.boundingRectangle.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
            self.boundingRectangle.setPen(QtGui.QPen(QtCore.Qt.transparent))
            scene.addItem(self.boundingRectangle)
            XCoord = 50
            YCoord = 820
            YOffset = 50
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord), \
                "Trade", self.boundingRectangle, self.tradeOffer, scale=1.2))
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord+YOffset), \
                "Build road", self.boundingRectangle, self.buyRoad, scale=1.2))
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord+2*YOffset), \
                "Build settlement", self.boundingRectangle, self.buySettlement, scale=1.2))
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord+3*YOffset), \
                "Build city", self.boundingRectangle, self.buyCity, scale=1.2))
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord+4*YOffset), \
                "Buy action card", self.boundingRectangle, doNothing, scale=1.2))
            self.roadIcon = ButtonItem(scene, QTtoART(x=320), QTtoART(y=810), \
                "", self.boundingRectangle, doNothing, scale=2, image_path=imagesPath+'road_blue.png', background_color=QtCore.Qt.transparent)
            self.settlementIcon = ButtonItem(scene, QTtoART(x=320), QTtoART(y=870), \
                "", self.boundingRectangle, doNothing, scale=3, image_path=imagesPath+'village_blue.png', background_color=QtCore.Qt.transparent)
            self.cityIcon = ButtonItem(scene, QTtoART(x=320), QTtoART(y=950), \
                "", self.boundingRectangle, doNothing, scale=3, image_path=imagesPath+'city_blue.png', background_color=QtCore.Qt.transparent)

            for item in self.items:
                item.w = 180

            for key, value in self.supplyCards.items():
                index = list(self.supplyCards.keys()).index(key)
                rect = QtGui.QGraphicsRectItem(50+index*80, 660, 70, 100, self.boundingRectangle)
                if key == "ORE":
                    rect.setBrush(QtGui.QBrush(QtGui.QColor(169,169,169)))
                elif key == "WOOL":
                    rect.setBrush(QtGui.QBrush(QtCore.Qt.green))
                elif key == "LUMBER":
                    rect.setBrush(QtGui.QBrush(QtGui.QColor(34,139,34)))
                elif key == "GRAIN":
                    rect.setBrush(QtGui.QBrush(QtGui.QColor(255,255,0)))
                elif key == "BRICK":
                    rect.setBrush(QtGui.QBrush(QtGui.QColor(165,42,42)))
                self.cardItems.append(rect)

                label = QtGui.QGraphicsTextItem(str(value), self.boundingRectangle, scene)
                label.setPos(60+index*80, 760)
                label.setFont(QtGui.QFont('Arial', 30))
                label.setDefaultTextColor(QtCore.Qt.white)
                self.supplyLabels[key] = label

            for key, value in self.buildings.iteritems():
                index = list(self.buildings.keys()).index(key)
                label = QtGui.QGraphicsTextItem(str(value), self.boundingRectangle, scene)
                label.setPos(260, 820 + index*80)
                label.setFont(QtGui.QFont('Arial', 30))
                label.setDefaultTextColor(QtCore.Qt.white)
                self.buildingLabels[key] = label

        if self.corner == 1 or self.corner == 3:
            self.boundingRectangle = QtGui.QGraphicsRectItem(1400, 650, 600, 550)
            self.boundingRectangle.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
            self.boundingRectangle.setPen(QtGui.QPen(QtCore.Qt.transparent))
            scene.addItem(self.boundingRectangle)
            XCoord = 1750
            YCoord = 820
            YOffset = 50
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord), \
                "Trade", self.boundingRectangle, self.tradeOffer, scale=1.2))
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord+YOffset), \
                "Build road", self.boundingRectangle, self.buyRoad, scale=1.2))
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord+2*YOffset), \
                "Build settlement", self.boundingRectangle, self.buySettlement, scale=1.2))
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord+3*YOffset), \
                "Build city", self.boundingRectangle, self.buyCity, scale=1.2))
            self.items.append(ButtonItem(scene, QTtoART(x=XCoord), QTtoART(y=YCoord+4*YOffset), \
                "Buy action card", self.boundingRectangle, doNothing, scale=1.2))
            self.roadIcon = ButtonItem(scene, QTtoART(x=1600), QTtoART(y=810), \
                "", self.boundingRectangle, doNothing, scale=2, image_path=imagesPath+'road_green.png', background_color=QtCore.Qt.transparent)
            self.settlementIcon = ButtonItem(scene, QTtoART(x=1600), QTtoART(y=870), \
                "", self.boundingRectangle, doNothing, scale=3, image_path=imagesPath+'village_green.png', background_color=QtCore.Qt.transparent)
            self.cityIcon = ButtonItem(scene, QTtoART(x=1600), QTtoART(y=950), \
                "", self.boundingRectangle, doNothing, scale=3, image_path=imagesPath+'city_green.png', background_color=QtCore.Qt.transparent)

            for item in self.items:
                item.w = 180

            for key, value in self.supplyCards.iteritems():
                index = list(self.supplyCards.keys()).index(key)
                rect = QtGui.QGraphicsRectItem(1550+index*80, 660, 70, 100, self.boundingRectangle)
                if key == "ORE":
                    rect.setBrush(QtGui.QBrush(QtGui.QColor(169,169,169)))
                elif key == "WOOL":
                    rect.setBrush(QtGui.QBrush(QtCore.Qt.green))
                elif key == "LUMBER":
                    rect.setBrush(QtGui.QBrush(QtGui.QColor(34,139,34)))
                elif key == "GRAIN":
                    rect.setBrush(QtGui.QBrush(QtGui.QColor(255,255,0)))
                elif key == "BRICK":
                    rect.setBrush(QtGui.QBrush(QtGui.QColor(165,42,42)))
                self.cardItems.append(rect)

                label = QtGui.QGraphicsTextItem(str(value), self.boundingRectangle, scene)
                label.setPos(1560+index*80, 760)
                label.setFont(QtGui.QFont('Arial', 30))
                label.setDefaultTextColor(QtCore.Qt.white)
                self.supplyLabels[key] = label

            for key, value in self.buildings.iteritems():
                index = list(self.buildings.keys()).index(key)
                label = QtGui.QGraphicsTextItem(str(value), self.boundingRectangle, scene)
                label.setPos(1660, 830+index*80)
                label.setFont(QtGui.QFont('Arial', 30))
                label.setDefaultTextColor(QtCore.Qt.white)
                self.buildingLabels[key] = label

        if self.corner == 2:      
            scene.removeItem(self.settlementIcon)
            scene.removeItem(self.roadIcon)
            self.roadIcon = ButtonItem(scene, QTtoART(x=320), QTtoART(y=810), \
                "", self.boundingRectangle, doNothing, scale=2, image_path=imagesPath+'road_red.png', background_color=QtCore.Qt.transparent)
            self.settlementIcon = ButtonItem(scene, QTtoART(x=320), QTtoART(y=870), \
                "", self.boundingRectangle, doNothing, scale=3, image_path=imagesPath+'village_red.png', background_color=QtCore.Qt.transparent)
            self.cityIcon = ButtonItem(scene, QTtoART(x=320), QTtoART(y=950), \
                "", self.boundingRectangle, doNothing, scale=3, image_path=imagesPath+'city_red.png', background_color=QtCore.Qt.transparent)

            self.boundingRectangle.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
            self.boundingRectangle.setTransformOriginPoint(300,975)
            self.boundingRectangle.setRotation(180)
            self.boundingRectangle.setX(1400)
            self.boundingRectangle.setY(-750)

        if self.corner == 3:
            scene.removeItem(self.settlementIcon)
            scene.removeItem(self.roadIcon)
            self.roadIcon = ButtonItem(scene, QTtoART(x=1600), QTtoART(y=810), \
                "", self.boundingRectangle, doNothing, scale=2, image_path=imagesPath+'road_yellow.png', background_color=QtCore.Qt.transparent)
            self.settlementIcon = ButtonItem(scene, QTtoART(x=1600), QTtoART(y=870), \
                "", self.boundingRectangle, doNothing, scale=3, image_path=imagesPath+'village_yellow.png', background_color=QtCore.Qt.transparent)
            self.cityIcon = ButtonItem(scene, QTtoART(x=1600), QTtoART(y=950), \
                "", self.boundingRectangle, doNothing, scale=3, image_path=imagesPath+'city_yellow.png', background_color=QtCore.Qt.transparent)

            self.boundingRectangle.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
            self.boundingRectangle.setTransformOriginPoint(1700,975)
            self.boundingRectangle.setRotation(180)
            self.boundingRectangle.setX(-1400)
            self.boundingRectangle.setY(-750)

    def updatePlayerUI(self):
        for key, label in self.supplyLabels.items():
            label.setPlainText(str(self.supplyCards[key]))
        for key, label in self.buildingLabels.items():
            label.setPlainText(str(self.buildings[key]))

    def disablePlayerUI(self):
        for button in self.items:
            button.set_enabled(False)

    def enablePlayerUI(self):
        for button in self.items:
            button.set_enabled(True)

#################################################################################################################
#################################                   GAME CLASS                  #################################
#################################################################################################################

class Game: 
    def __init__(self, scene, loadMapSlot, numberOfPlayers):
        #print("hra zacina na mape ze slotu " + str(loadMapSlot))
        self.scene = scene
        self.colors = ["blue", "green", "red", "yellow"]
        self.supplyCards = self.createStartingSupplyCards() #pole karet zasob
        self.actionCards = self.createStartingActionCards()
        if loadMapSlot == "default":
            self.map = self.createDefaultMap() #TODO zmenit
        else:
            self.map = self.loadMap(loadMapSlot)
        self.players = self.createPlayers(numberOfPlayers) #TODO zmenit
        self.items = []
        self.turnNumber = -1
        self.endOfTurn = False
        self.recognizer = Recognizer()
        self.playerTurnMarks = [None] * 4

        #TODO na vymazani pak
        self.rect = QtGui.QGraphicsRectItem(0,0,2000, 1200)
        self.rect.setPen(QtGui.QPen(QtCore.Qt.white))
        self.rect.setBrush(QtGui.QBrush(QtCore.Qt.transparent))
        self.scene.addItem(self.rect)
        
        #self.scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(30,144,255)))
        self.scene.setBackgroundBrush(QtCore.Qt.black)

        self.announcementArea = DescItem(self.scene, QTtoART(x=750), QTtoART(y=980), None)

        self.announcementArea2 = DescItem(self.scene, QTtoART(x=1250), QTtoART(y=220), None)
        self.announcementArea2.setRotation(180)

        self.nextTurnBtn = ButtonItem(self.scene, QTtoART(x=920), QTtoART(y=1050), "End turn", None, self.endTurn, scale=2)
        self.nextTurnBtn2 = ButtonItem(self.scene, QTtoART(x=1070), QTtoART(y=150), " End turn", None, self.endTurn, scale=2)
        self.nextTurnBtn2.setRotation(180)

        self.map.drawMap()
        for key, player in self.players.iteritems():
            player.drawPlayerUI(self.scene)

        #TouchTableItem(self.scene, '/art/interface/touchtable/touch')

    def createPlayers(self, count):
        self.colors = self.colors[:count]
        playerPool = {}
        for n in range(count):  
            playerPool[self.colors[n]] = Player(self.colors[n], n, self)
        return playerPool

    def createDefaultMap(self):
        defaultMap = Map(self.scene, game = self)
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
        defaultMap.setTileNumbers(arg=[10,2,9,12,6,4,10,9,11,0,3,8,8,3,4,5,5,6,11])
        return defaultMap

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

    def loadMap(self, loadMapSlot):
        loadedMap = self.createEmptyMap()
        resPath = rospack.get_path('my_gui') + '/src/maps/'
        
        file = open(resPath + "mapfile" + str(loadMapSlot) + ".map",'r') 

        temp = file.read().splitlines()
        for tile in loadedMap.tiles:
            tile.areaType = temp[tile.index]
        numbers = temp[19].split(',')
        numberList = []
        for num in numbers[:19]:
            numberList.append(int(num))
        loadedMap.setTileNumbers(arg=numberList)
        file.close()
        return loadedMap

    def createStartingSupplyCards(self):
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

    def takeSupplyFromPlayer(self, color, supplyType):
        if self.players[color].supplyCards[supplyType] == 0:
            print("ERROR: no more cards of this type (" + supplyType + ")")
        else:
            self.players[color].removeSupplyCard(supplyType)
            self.supplyCards[supplyType] +=1

    def createStartingActionCards(self):
        #vytvoreni balicku akcnich karet
        pass

    def distributeSupplies(self, number):
        indices = [i for i, x in enumerate(self.map.tileNumbers) if x == number]
        for i in indices:
            if self.map.tiles[i].index != self.map.robber:
                self.map.tiles[i].giveSupplies(self)

    def drawTurnOfPlayerAnnouncement(self, color, mode="road"):
        for mark in self.playerTurnMarks:
            if mark != None:
                self.scene.removeItem(mark)

        self.playerTurnMarks[0] = ButtonItem(self.scene, QTtoART(x=500), QTtoART(y=800), "", None, doNothing, \
            image_path=imagesPath + mode + "_" + color + ".png", background_color=QtCore.Qt.transparent, scale=7)
        self.playerTurnMarks[1] = ButtonItem(self.scene, QTtoART(x=1350), QTtoART(y=800), "", None, doNothing, \
            image_path=imagesPath + mode + "_" + color + ".png", background_color=QtCore.Qt.transparent, scale=7)
        self.playerTurnMarks[2] = ButtonItem(self.scene, QTtoART(x=1350), QTtoART(y=200), "", None, doNothing, \
            image_path=imagesPath + mode + "_" + color + ".png", background_color=QtCore.Qt.transparent, scale=7)
        self.playerTurnMarks[3] = ButtonItem(self.scene, QTtoART(x=500), QTtoART(y=200), "", None, doNothing, \
            image_path=imagesPath + mode + "_" + color + ".png", background_color=QtCore.Qt.transparent, scale=7)
        for mark in self.playerTurnMarks:
            if mark != None:
                mark.h = 150

    def checkGameEnd(self):
        for key, player in self.players.items():
            player.countPoints()
            if player.points >= 10:
                self.announcementArea.set_content("THE END", 3)
                self.announcementArea2.set_content("THE END", 3)

    def endTurn(self, button=None):
        self.nextTurnBtn.set_caption("Next turn")
        self.nextTurnBtn2.set_caption("Next turn")
        self.nextTurnBtn.set_cb(self.nextTurn)
        self.nextTurnBtn2.set_cb(self.nextTurn)
        self.players[self.colors[self.turnNumber%len(self.colors)]].disablePlayerUI()
        #self.nextTurn()

    def nextTurn(self, button=None):
        self.nextTurnBtn.set_caption("End turn")
        self.nextTurnBtn2.set_caption("End turn")
        self.nextTurnBtn.set_cb(self.endTurn)
        self.nextTurnBtn2.set_cb(self.endTurn)

        self.map.updateMap()
        for tile in self.map.tiles:
            tile.unsetFocus()
        self.turnNumber += 1
        if self.endOfTurn:
            self.turnNumber -= 1
        if self.turnNumber <= len(self.colors)*2-1:
            if self.endOfTurn == False:
                for key, player in self.players.items():
                    player.disablePlayerUI()
                self.map.buildMode = "settlement"
                for tile in self.map.tiles:
                    tile.setFocus()
                self.drawTurnOfPlayerAnnouncement(self.colors[self.turnNumber%len(self.colors)])
                self.announcementArea.set_content(self.colors[self.turnNumber%len(self.colors)].upper() + " : " + \
                    "Choose starting position", 2)
                self.announcementArea2.set_content(self.colors[self.turnNumber%len(self.colors)].upper() + " : " + \
                    "Choose starting position", 2)
                self.endOfTurn = True
            else:
                self.map.buildMode = "road"
                for tile in self.map.tiles:
                    tile.setFocus()
                self.endOfTurn = False

        else:           
            #TODO kontrola konce hry
            self.announcementArea.set_content("")
            self.announcementArea2.set_content("")
            self.players[self.colors[self.turnNumber%len(self.colors)]].enablePlayerUI()
            print("Turn of " + self.colors[self.turnNumber%len(self.colors)] + " player.")
            #hod kostkama
            # number = self.recognizer.getDicesValue()
            number1 = randint(1,6)
            number2 = randint(1,6)
            number = number1 + number2
            if number == 7:
                self.map.changeToRobberMode()
            # rozdeleni surovin
            self.distributeSupplies(number)
            #obchodovani
            for key, player in self.players.items():
                player.updatePlayerUI()
            #stavba

def main():
    game = Game()
    game.distributeSupplies(5)
    game.printGameStatus()

if __name__ == "__main__":
    main()
