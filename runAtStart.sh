v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=10
python3 /home/ubuntu/Desktop/Vision/lineFollowingTest.py
