class ActionCard:
	def __init__(self, faceUp, actionType):
		super().__init__(faceUp)
		self.actionType = actionType
		self.faceUp = faceUp

	def turn(self):
		if self.faceUp:
			self.faceUp = False
		else:
			self.faceUp = True

def main():
	pass
	
if __name__ == "__main__":
    main()