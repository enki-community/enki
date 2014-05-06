#!/usr/bin/env python
import argparse
import numpy as np
from scipy.optimize import fmin
from scipy.optimize import fmin_bfgs
from scipy.optimize import anneal

def main():
	# parse arguments
	parser = argparse.ArgumentParser(description='Tool to compute IR sensors from ')
	parser.add_argument('distances', help='Array (a,b,...) of distances')
	parser.add_argument('activations', help='Array (a,b,...) of activation values')
	args = parser.parse_args()
	x = map(float, args.distances.split(','))
	y = map(float, args.activations.split(','))
	if len(x) != len(y):
		raise RuntimeError('distances array size %d is different than activations array size %d' % (len(x), len(y)))
	print x,y
	
	# function to optimise
	def F(v):
		error = 0.
		a = v[0]
		b = v[1]
		c = v[2]
		for (xi,yi) in zip(x,y):
			denom = (xi*xi + b*xi + c)
			if abs(denom) < 1e-30:
				denom = math.copysign(1e-30, denom)
			err = (yi - a / denom)
			error += err * err
		#print error
		return error
	
	# run optimisation
	#res, Jmin = anneal(F, [10.,10.,10.], lower=-1e5, upper=1e5, maxiter=100000)
	#print res, Jmin
	best_val = 1e100
	best_x = None
	for i in range(1000):
		init_x = np.random.normal(scale=1000,size=3)
		#print init_x
		res = fmin(F, init_x)
		res_val = F(res)
		if res_val < best_val:
			print 'new best val ', best_val
			best_val = res_val
			best_x = res
	print best_x, best_val

if __name__ == '__main__':
	main()