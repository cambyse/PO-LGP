from flask import Flask, request, send_from_directory, jsonify

import numpy as np
import datetime, time
from threading import Thread
import logging

from sortedcontainers import SortedListWithKey
from filters import Kalman, Lowpass

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)

clients = {}

class Object(object):
	pass


def _getclient():
	""" Create a new client based on the client ip address
	The idea was to be able to support multiple clients
	and control multiple robot arms in the future
	"""
	global clients
	cid = request.remote_addr.split('.')[-1]
	if cid not in clients:
		print 'New Client: %s' % cid
		c = Object()

		# client id.  The last section of ipv4
		c.cid = cid

		# last acceleration timestamp
		c.a_ts = None
		# last gyro timestamp
		c.g_ts = None
		# first data point timestamp
		c.first = None

		# current acceleration based on phone coordinates
		c.a = np.zeros([4])
		# current gyro data
		c.g = np.zeros([4])
		# current position based on double integration
		c.p = np.zeros([3])
		# last gyro data.  Used to calculat angular velocity
		c.lg = c.g
		# point on a sphere based on gyro data
		c.gp = np.matrix([[0.],[1.],[0.]])
		# velocity by integrating the acceleration data
		c.v = np.zeros([3])
		# change in gyro (angular velocity)
		c.dg = np.zeros([3])
		# radius from velocity calculated from acceleration divided by angular acceleration
		c.r = 1.0

		clients[cid] = c

		B = np.eye(9)
		x = np.matrix([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T # initial state
		P = np.eye(9)*0.1
		R = np.matrix(np.eye(3)*0.001)	

		c.kalman = Kalman(B=B, x=x, P=P, R=R)

	return clients[cid]


def _getfloat(key):
	val = request.form.get(key, None)
	return float(val) if val else None


def _getradian(key):
	val = _getfloat(key)
	return np.radians(val) if val else None


@app.route('/')
def root():
    return app.send_static_file('index.html')


@app.route('/js/<path:path>')
def js(path):
    return send_from_directory('static/js', path)


@app.route('/<path:path>.html')
def html(path):
    return send_from_directory('static/', path+'.html')


@app.route('/data')
def data():
	""" Return accelerometer and gyroscope data in JSON format
	"""
	cid = request.args.get('cid', '')
	if cid:
		c = clients.get(cid, None)
	elif len(clients):
		c = clients.itervalues().next()
	else:
		c = None

	if not c:
		return '', 404

	return jsonify(dict(g=c.g.tolist(), a=c.a.tolist()))


@app.route('/position')
def position():
	""" Return the position data
	"""
	cid = request.args.get('cid', '')
	if cid:
		c = clients.get(cid, None)
	elif len(clients):
		c = clients.itervalues().next()
	else:
		c = None

	if not c:
		return '', 404

	# position data on the gyro sphere
	p = c.gp

	# position data with gyro and radius calculated by velocity
	#p = c.gp * c.r

	# position data using kalman filter
	#p = c.kalman.state()[:3]

	return jsonify(p.T[0].tolist()[0])


@app.route('/app/g', methods=['POST', 'GET'])
def gyro():
	""" Get gyro data from client
	"""
	c = _getclient()

	alpha = _getradian('alpha') 
	beta = _getradian('beta')
	gamma = _getradian('gamma')
	ts = _getfloat('ts')

	if not c.first:
		c.first = ts

	if ts > c.g_ts:
		if c.g_ts==None:
			c.g_ts = ts
		else:
			dt = (ts - c.g_ts) / 1000 #compute dt and convert to seconds from milliseconds
			c.g_ts = ts # update last gyro time stamp
			c.lg = c.g # save last gyro data

			c.g = np.array([alpha, beta, gamma, ts-c.first])

			# calculate the change in angle
			da = np.linalg.norm(c.g[0:3] - c.lg[0:3])

			if da<0.05:
			 	#if no angular displacement, then velocity is set to 0.
			 	c.v = np.zeros([3])
			 	c.kalman.state()[3:6] = np.matrix([[0.,0.,0.]]).T
			else:
				# other compute the angular velocity and 
				# attempt to find distance by velocity / angular velocity 
				c.dg = (c.g - c.lg) /dt

				r1 = c.v[0]/(c.dg[0])
				r2 = c.v[1]/(c.dg[1])

				c.r = np.sqrt(r1*r1+r2*r2)

			# compute the angular point on a sphere with radius of 1.0
			c.gp = np.matrix([[
					-np.sin(c.g[0])*np.cos(c.g[1]),
					np.cos(c.g[0])*np.cos(c.g[1]),
					np.sin(c.g[1])
				]]).T
	else:
		print 'Gyro Out of Order'

	return ''


@app.route('/app/a', methods=['POST', 'GET'])
def accelerometer():
	""" Get accelerometer data from client
	"""
	c = _getclient()

	ddx = _getfloat('x')
	ddy = _getfloat('y')
	ddz = _getfloat('z')
	ts = _getfloat('ts')

	if not c.first:
		c.first = ts

	if ts > c.a_ts:
		if c.a_ts==None:
			c.a_ts = ts
			c.la = [ddx, ddy, ddz, ts-c.first]
		else:
			dt = (ts-c.a_ts)/1000.0 # get delta time and convert milliseconds to seconds 
			c.a_ts = ts

			a = np.array([ddx, ddy, ddz])

			# convert phone coordinates to world coordinates
			wa = np.linalg.inv(rot_matrix(c.g)).dot(a)

			c.a = np.array([ddx, ddy, ddz, ts-c.first])

			# --------------------------------------------------------------------------
			# calculating velocity and position by double integration
			c.a = np.array([wa[0], wa[1], wa[2], ts-c.first]) 
			#c.a = Lowpass.filter('a', [wa[0], wa[1], wa[2], ts-c.first], 19)
			v = c.v + c.a[:3] * dt
			c.p = c.p + (c.v+v) * dt / 2.0 
			c.v = v

			# --------------------------------------------------------------------------
			# calculate position using kalman filter

			# motion matrix
			u = np.matrix(np.zeros([9])).T
			u[6] = wa[0]
			u[7] = wa[1]
			u[8] = wa[2]

			# set up state transition matrix with dt
			A = np.eye(9)
			A[0][3] = A[1][4] = A[2][5] = A[3][6]= A[4][7] = A[5][8] = dt 
			A[0][6] = A[1][7] = A[2][8] = dt*dt / 2.0 
			A[6][6] = A[7][7] = A[8][8] = 0.0  #acceleration is measured so do not use for state computation

			d = np.linalg.norm(c.kalman.state()[:3])

			# set up obervation matrix.
			# We are observing the gyroscope so the position needs to be normalized
			H = np.matrix([
				[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				]) / d

			# do kalman step
			x = c.kalman.step(A=A, H=H, u=u, M=c.gp).tolist()
			#print 'P: %s' % x[:3]
			#print 'V: %s' % x[3:6]
			#print 'A: %s' % x[6:]
	else:
		print 'Accelerometer Out of Order'

	return ''


def rot_matrix(g):
	""" create rotation matrix using gyro data
	"""
	ca = np.cos(g[0])
	sa = np.sin(g[0])
	cb = np.cos(g[1])
	sb = np.sin(g[1])
	cc = np.cos(g[2])
	sc = np.sin(g[2])

	return np.array([
		[ca*cc-sa*cb*sc, -ca*sc-sa*cb*cc, sb*sa],
		[sa*cc+ca*cb*sc, -sa*sc+ca*cb*cc, -sb*ca],
		[sb*sc, sb*cc, cb],
		])

if __name__ == "__main__":
    app.run(threaded=True, host='0.0.0.0')
