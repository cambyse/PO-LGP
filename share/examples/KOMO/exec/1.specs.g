repeats = 1

KOMO{
	model = '0.pose.g'
	T = 30
	duration = 5
}

Task step{
	map={ type=qItself }
	order=1
	scale=1
}

Task finalHandPosition{
	map={ type=posDiff ref1=endeff vec1=[.1 0 0] ref2=bar1 vec2=[0.1 0 0] }
	time=[1 1]
	scale=100
	type=equal
}

Task midHandPosition{
	map={ type=posDiff ref1=endeff vec2=[0 0 1.5] }
	time=[.5 .5]
	scale=100
}

Task collisions{
	map={ type=collisionIneq margin=0.05 }
	type=inequal
	scale = 1.
}

KinematicSwitch{
	type=addRigidRel
	timeOfApplication=1
	from=endeff
	to=bar1
}