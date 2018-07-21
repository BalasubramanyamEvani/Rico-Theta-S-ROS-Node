#!/usr/bin/env python

'''

@Author: Balasubramanyam Evani
Manipal University Jaipur

This is just the python translation of MATLAB code given by David Scaramuzza (original source), 

refer https://sites.google.com/site/scarabotix/ocamcalib-toolbox

'''

import math
import numpy as np

def get_ocam_model(filename): 				## get_ocam_model -> gets parameters obtained after running OcamCalib MATLAB toolbox

	ocam_struct = {} 				## a dictionary to store values
	with open(filename) as f:			## openning the file containing the calib results
		'''
		The following just stores the values
		'''		
		lines = [line for line in f]
		data_pols = lines[2].split()
		data_invpols = lines[6].split()
		data_imageCenter = lines[10].split()
		data_affine = lines[14].split()
		data_dims = lines[18].split()

		ocam_struct['length_pol'] = int(data_pols[0])
		ocam_struct['pols'] = [float(val) for val in data_pols[1:]]
		ocam_struct['length_invpol'] = int(data_invpols[0])
		ocam_struct['inv_pols'] = [float(val) for val in data_invpols[1:]]
		ocam_struct['xc'] = float(data_imageCenter[0])
		ocam_struct['yc'] = float(data_imageCenter[1])
		ocam_struct['c'] = float(data_affine[0])
		ocam_struct['d'] = float(data_affine[1])
		ocam_struct['e'] = float(data_affine[2])
		ocam_struct['height'] = int(data_dims[0])
		ocam_struct['width'] = int(data_dims[1])

	return ocam_struct 				## return the dictionary of values 

def cam2world(point2D , ocam_struct):			## Method implementing the MATLAB version of cam2world, 2D -> 3D
	
	point3D = []
	pols = ocam_struct['pols']
	xc = ocam_struct['xc']
	yc = ocam_struct['yc']
	c = ocam_struct['c']
	d = ocam_struct['d']
	e = ocam_struct['e']
	length_pol = ocam_struct['length_pol']
	inv_det = 1/(c - d*e)
	xp = inv_det * ( ( point2D[0]  - xc) - d * (point2D[1] - yc) )
	yp = inv_det * ( -e * (point2D[0] - xc) + c * (point2D[1] - yc) )
	r = math.sqrt(xp**2 + yp**2)
	zp  = pols[0]
	r_i = 1.0
	for i in range(1 , length_pol):
		r_i = r_i * r
		zp += r_i * pols[i]
	invnorm = 1 / math.sqrt( xp**2 + yp**2 )
	point3D.append(invnorm * xp)
	point3D.append(invnorm * yp)
	point3D.append(invnorm * zp)

	return point3D

def world2cam(point3D , ocam_struct):			## Method implementing the world2cam function of MATLAB, 3D -> 2D
	
	invpols = ocam_struct['inv_pols']
	xc = ocam_struct['xc']
	yc = ocam_struct['yc']
	c = ocam_struct['c']
	d = ocam_struct['d']
	e = ocam_struct['e']
	width = ocam_struct['width']
	height = ocam_struct['height']
	length_invpol = ocam_struct['length_invpol']
	norm = math.sqrt(point3D[0]**2 + point3D[1]**2)
	if norm != 0:
		theta = np.arctan(point3D[2] / norm)
	point2D = []
	if norm != 0:
		invnorm = 1 / norm
		t = theta
		rho = invpols[0]
		t_i = 1
		for i in range(1 , length_invpol):
			t_i = t_i * t
			rho += t_i * invpols[i]
		x = point3D[0] * invnorm * rho
		y= point3D[1] * invnorm * rho

		point2D.append(x*c + y*d + xc)
		point2D.append(x*e + y + yc)

	else:
		point2D.append(xc)
		point2D.append(yc)

	return point2D

'''
Undistortion functions mentioned in the MATLAB toolbox

mapx,mapy is used for remapping in publisher.py

'''

def create_perspective_undistortion_LUT(ocam_struct , sf):
	
	mapx = np.zeros((ocam_struct['height'] , ocam_struct['width']))
	mapy = np.zeros((ocam_struct['height'] , ocam_struct['width']))
	Nxc = ocam_struct['height'] / 2.0
	Nyc = ocam_struct['width'] / 2.0
	Nz = -(ocam_struct['width'])/sf
	for i in range(ocam_struct['height']):
		for j in range(ocam_struct['width']):
			M = []
			M.append(i - Nxc)
			M.append(j - Nyc)
			M.append(Nz)
			m = world2cam(M , ocam_struct)
			mapx[i][j] = m[1]
			mapy[i][j] = m[0]

	return mapx , mapy			

def create_panoramic_undistortion_LUT(Rmin , Rmax , ocam_struct):
	
	mapx = np.zeros((ocam_struct['height'] , ocam_struct['width']))
	mapy = np.zeros((ocam_struct['height'] , ocam_struct['width']))
	xc = ocam_struct['xc']
	yc = ocam_struct['yc']
	for i in range(ocam_struct['height']):
		for j in range(ocam_struct['width']):
			theta = -(float(j) / (ocam_struct['width'] * 2 * np.pi ))
			rho = Rmax - (Rmax - Rmin) / height * i
			mapx[i][j] = yc + rho * np.sin(theta)
			mapy[i][j] = xc + rho * np.cos(theta)

	return mapx , mapy
