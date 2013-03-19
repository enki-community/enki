import pyenki
import random

class MyEPuck(pyenki.EPuck):
	
	def __init__(self):
		super(MyEPuck, self).__init__()
		self.timeout = 10
		
	def controlStep(self, dt):
		if self.timeout == 0:
			self.leftSpeed = random.uniform(-100,100)
			self.rightSpeed = random.uniform(-100,100)
			self.timeout = random.randint(1,10)
		else:
			self.timeout -= 1
		#print('Control step')
		#print('pos: ' + str(self.pos))
		#print('IR dists: ' + str(self.proximitySensorDistances))
		#print('IR values: ' + str(self.proximitySensorValues))
		#print('Cam image: ' + str(self.cameraImage))
		#print len(self.cameraImage), self.cameraImage[0]
		print id(self), self.pos


w = pyenki.World()

for i in range(0,10):
	for j in range(0,10):
		e = MyEPuck()
		e.pos = (i*10,j*10)
		w.addObject(e)

w.runInViewer()

