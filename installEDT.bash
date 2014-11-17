rosdep install --from-paths ./ --ignore-src --rosdistro indigo

cd .git/hooks

echo "#!/bin/bash" >> post-receive
echo "rosdep install --from-paths src --ignore-src --rosdistro indigo" >> post-receive
echo "#!/bin/bash" >> post-update
echo "rosdep install --from-paths src --ignore-src --rosdistro indigo" >> post-update

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
echo "You should now be in the /devel/ folder.

echo "Finished Installing EDT's IGVC Repository."
