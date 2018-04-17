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

def main():
	pass
	
if __name__ == "__main__":
    main()