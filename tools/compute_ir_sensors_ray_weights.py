#!/usr/bin/env python
import argparse
import numpy as np
import math
from scipy.optimize import fmin
from scipy.optimize import fmin_bfgs
from scipy.optimize import anneal

def main():
	# parse arguments
	parser = argparse.ArgumentParser(description='Tool to compute ray weights for 3 rays, using the well-known Colas wall-sliding method')
	parser.add_argument('params', help='Parameters for activation function F')
	parser.add_argument('dist', help='distance to the wall', type=float)
	parser.add_argument('dmax', help='distance measured when not seeing anything', type=float)
	parser.add_argument('angles', help='angles used for measurements')
	parser.add_argument('distances', help='perceived distances at angles')
	args = parser.parse_args()
	p = map(float, args.params.split(','))
	if len(p) != 3:
		raise RuntimeError('you must provide 3 parameters for activation function F, but you provided %d' % p)
	x = map(float, args.angles.split(','))
	y = map(float, args.distances.split(','))
	if len(x) != len(y):
		raise RuntimeError('angle array size %d is different than distance array size %d' % (len(x), len(y)))
	print p,x,y
	
	# activation function
	def F(x):
		return p[0] / (x*x + p[1]*x + p[2])
	f_d0 = F(args.dist)
	f_dmax = F(args.dmax)
	
	# function to optimise
	def E(v,verbose=False):
		error = 0.
		angle = v[0]
		w0 = v[1]
		w1 = v[2]
		f_d1 = F(args.dist / math.cos(math.radians(angle)))
		def F_dsim(a):
			if a < -angle:
				return w1*f_d1 + w0*f_d0 + w1*f_d1
			elif a < 0:
				return w1*f_dmax + w0*f_d0 + w1*f_d1
			elif a < angle:
				return w1*f_dmax + w0*f_dmax + w1*f_d1
			else:
				return w1*f_dmax + w0*f_dmax + w1*f_dmax
		for (xi,yi) in zip(x,y):
			err = F(yi) - F_dsim(xi)
			if verbose:
				print('Angle: %f, dist: %f, F(dist): %f,  F_dsim: %f' % (xi, yi, F(yi), F_dsim(xi)))
			error += err * err
		#print error
		return error
	
	best_val = 1e100
	best_x = None
	for i in range(100):
		init_x = [max(np.abs(x))/3., abs(np.random.normal(scale=1)), abs(np.random.normal(scale=1))]
		print init_x
		res = fmin(E, init_x)
		res_val = E(res)
		if res_val < best_val:
			print 'new best val ', best_val
			best_val = res_val
			best_x = res
	print best_x, best_val
	E(best_x, True)

if __name__ == '__main__':
	main()