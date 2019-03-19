from ftplib import FTP
import pathfinder as pf
import math
points = [
	pf.Waypoint(0, 0, math.radians(0)),
	pf.Waypoint(1, 1, math.radians(90))
]
info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
dt=0.05, # 50ms
max_velocity=24.0,
max_acceleration=10.0,
max_jerk=60.0
	)
	pf.serialize("newTrajectory.bin",trajectory)
with FTP("roboRIO-2577-frc.local") as ftp:
	ftp.login()
	ftp.storbinary('STOR newTrajectory.bin',open("newTrajectory.bin",mode='rb'))
