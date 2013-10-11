import pyenki
import math

class MyEPuck(pyenki.EPuck):
	def controlStep(self, dt):
		self.leftSpeed = 0.1
		self.rightSpeed = 0.2
		print('Control step')
		print('pos: ' + str(self.pos))
		print('IR dists: ' + str(self.proximitySensorDistances))
		assert(not any(map(math.isnan, self.proximitySensorDistances)))
		print('IR values: ' + str(self.proximitySensorValues))
		assert(not any(map(math.isnan, self.proximitySensorValues)))
		print('Cam image: ' + str(self.cameraImage))
		print len(self.cameraImage), self.cameraImage[0]

w = pyenki.World()
e = MyEPuck()
#e = pyenki.EPuck()
w.addObject(e)

for i in range(10):
	w.step(0.05)
	print ''