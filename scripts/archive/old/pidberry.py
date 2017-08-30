import time

kp = {
	'roll': 	0.1,
	'pitch': 	0.1,
	'yaw': 		0.1,
	'alt': 		0.1
}

ki = {
	'roll': 	0.0,
	'pitch':	0.0,
	'yaw': 		0.0,
	'alt': 		0.0
}

kd = {
	'roll': 	0.0,
	'pitch': 	0.0,
	'yaw': 		0.0,
	'alt': 		0.0
}


# these positions are global (ie what comes out of the motion tracker)
sp_global  = {'x': 0.0, 'z': 0.0, 'alt': 0.0, 'yaw': 0.0} # set point
pos_global = {'x': 0.0, 'z': 0.0, 'alt': 0.0, 'yaw': 0.0} # set point

# these positions are definted relative to the orientation of the quad
sp 	= {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0} # set point
pos = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0} # current position

output = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}

err   = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Pterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Iterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}
Dterm = {'fb': 0.0, 'lr': 0.0, 'alt': 0.0, 'yaw': 0.0}

time_prev = millis()

while(1):
	time_elapsed = millis() - time_prev
	time_prev = millis()

	updatePos() # get information on where the drone is currently
	updateSP() # update the SP if it has changed

	# convert to the quad's frame of reference from the global
	sp['fb'] = math.cos(sp_global['yaw']) * sp_global['z'] + math.sin(sp_global['yaw']) * sp_global['x']
	sp['lr'] = math.sin(sp_global['yaw']) * sp_global['z'] + math.cos(sp_global['yaw']) * sp_global['x']

	pos['fb'] = math.cos(pos_global['yaw']) * pos_global['z'] + math.sin(pos_global['yaw']) * pos_global['x']
	pos['lr'] = math.sin(pos_global['yaw']) * pos_global['z'] + math.cos(pos_global['yaw']) * pos_global['x']

	sp = sp_global['yaw'] - pos_global['yaw']
	pos['yaw'] = 0.0

	sp = sp_global['alt'] - pos_global['alt']
	pos['alt'] = 0.0


	old_err = err
	for key in sp.keys(): 
		err[key] = sp[key] - pos[key] # update the error

		# calc the PID components of each axis
		Pterm[key] = err[key]
		Iterm[key] += err[key] * time_elapsed
		Dterm[key] = (err[key] - old_err[key])/time_elapsed

		output[key] = Pterm[key] * kp[key] + Iterm[key] * kI[key] + Dterm[key] * kd[key]



millis = lambda: int(round(time.time() * 1000))
