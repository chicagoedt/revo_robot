sudo apt-get update
rosdep update

rosdep install --from-paths ./ --ignore-src --rosdistro indigo -y

cd ../../

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

echo "Finished Installing EDT's IGVC Repository."
