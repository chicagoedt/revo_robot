sudo apt-get update
rosdep update
rosdep install --from-paths ./ --ignore-src --rosdistro indigo -y

cd .git/hooks

echo "#!/bin/bash" >> post-receive
echo "rosdep install --from-paths src --ignore-src --rosdistro indigo -y" >> post-receive
echo "#!/bin/bash" >> post-update
echo "rosdep install --from-paths src --ignore-src --rosdistro indigo -y" >> post-update

sudo chmod +x post-receive
sudo chmod +x post-update

cd ../../../

catkin_make clean
catkin_make


echo "Putting setup.bash into your .bashrc..."
cd devel/

path="$(pwd)"

echo "source $path/setup.bash" >> ~/.bashrc

echo "Sourcing the ~/.bashrc..."
source ~/.bashrc

echo "Executing 'roscd'..." 
roscd
echo "You should now be in the /devel/ folder."

echo "Now grabbing scipio's gazebo meshes from the EDT wiki..."
cd ..
cd src/simulation/scipio_simulation
mkdir meshes
cd meshes

wget http://wiki.chicagoedt.org/images/7/70/Back_left_wheel_link.STL.zip
unzip Back_left_wheel_link.STL.zip
rm Back_left_wheel_link.STL.zip

wget http://wiki.chicagoedt.org/images/7/71/Base_link.DAE.zip
unzip Base_link.DAE.zip
rm Base_link.DAE.zip

wget http://wiki.chicagoedt.org/images/9/9b/Front_left_wheel_link.STL.zip
unzip Front_left_wheel_link.STL.zip
rm Front_left_wheel_link.STL.zip

wget http://wiki.chicagoedt.org/images/9/95/Back_right_wheel_link.DAE.zip
unzip Back_right_wheel_link.DAE.zip
rm Back_right_wheel_link.DAE.zip

wget http://wiki.chicagoedt.org/images/a/aa/Back_right_wheel_link.STL.zip
unzip Back_right_wheel_link.STL.zip
rm Back_right_wheel_link.STL.zip

wget http://wiki.chicagoedt.org/images/d/d0/Front_right_wheel_link.STL.zip
unzip Front_right_wheel_link.STL.zip
rm Front_right_wheel_link.STL.zip

wget http://wiki.chicagoedt.org/images/d/d4/Front_left_wheel_link.DAE.zip
unzip Front_left_wheel_link.DAE.zip
rm Front_left_wheel_link.DAE.zip

wget http://wiki.chicagoedt.org/images/1/10/Base_link.STL.zip
unzip Base_link.STL.zip
rm Base_link.STL.zip

wget http://wiki.chicagoedt.org/images/f/fa/Front_right_wheel_link.DAE.zip
unzip Front_right_wheel_link.DAE.zip
rm Front_right_wheel_link.DAE.zip

wget http://wiki.chicagoedt.org/images/5/5a/Back_left_wheel_link.DAE.zip
unzip Back_left_wheel_link.DAE.zip
rm Back_left_wheel_link.DAE.zip

roscd scipio_simulation/..

cd grass/materials/
mkdir textures
cd textures
wget http://wiki.chicagoedt.org/images/6/63/Igvc_lines_4000x4000cm.png
mv Igvc_lines_4000x4000cm.png igvc_lines_4000x4000cm.png

cd ../../meshes/
mkdir images
cd images
wget http://wiki.chicagoedt.org/images/c/c8/0_Grass0081_L.jpg
wget http://wiki.chicagoedt.org/images/c/c9/1_igvc_lines.png

roscd scipio_simulation/..
if [ ! -d ~/.gazebo/models/ ]; then
        mkdir -p ~/.gazebo/models/
        cp -rf scipio_simulation ~/.gazebo/models
        cp -rf grass ~/.gazebo/models/
	cp -rf lcsr_camera_models ~/.gazebo/models
        echo "Made directory"
else
        cp -rf scipio_simulation ~/.gazebo/models
        ls -l ~/.gazebo/models/scipio_simulation
        echo "Tried to copy to scipio_simulation"
        cp -rf grass ~/.gazebo/models/
        ls -l ~/.gazebo/models/grass
        echo "Tried to copy to grass"
	cp -rf lcsr_camera_models ~/.gazebo/models
fi

roscd 
cd ..
catkin_make run_tests

echo "Finished Installing EDT's IGVC Repository."
