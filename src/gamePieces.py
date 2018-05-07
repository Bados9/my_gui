class ActionCard:
    def __init__(self, faceUp, actionType):
        self.actionType = actionType
        self.faceUp = faceUp

    def turn(self):
        if self.faceUp:
            self.faceUp = False
        else:
            self.faceUp = True

class Building:
    def __init__(self, buildingType, color):
        self.color = color
        self.type = buildingType #city nebo settlement nebo road?

class Trade:
    def __init__(self):
        self.resourceOne = "NONE"
        self.resourceTwo = "NONE"
        self.resOneCount = 0
        self.resTwoCount = 0

    def decreaseResOne(self):
        self.resOneCount -= 1
        if self.resOneCount < 0:
            self.resOneCount = 0

    def decreaseResTwo(self):
        self.resTwoCount -= 1
        if self.resTwoCount < 0:
            self.resTwoCount = 0

    def increaseResOne(self, maxCount):
        self.resOneCount += 1
        if self.resOneCount > maxCount:
            self.resOneCount = maxCount
    def increaseResTwo(self):
        self.resTwoCount += 1

def main():
    pass
    
if __name__ == "__main__":
    main()