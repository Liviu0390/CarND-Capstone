# enforce tensorflow-gpu version 1.3.0
if [[ $(pip show tensorflow-gpu | grep 1.3.0) ]]; then
  echo "Tensorflow-gpu version 1.3.0 found."
else
  echo "Wrong tensorflow-gpu version, reinstalling."
  pip install --force-reinstall tensorflow-gpu==1.3.0
fi

# source script and launch
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
