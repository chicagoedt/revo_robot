rosdep install --from-paths ./ --ignore-src --rosdistro indigo

cd .git/hooks
echo "#!/bin/bash" >> post-receive
echo "rosdep install --from-paths src --ignore-src --rosdistro indigo" >> post-receive
echo "#!/bin/bash" >> post-update
echo "rosdep install --from-paths src --ignore-src --rosdistro indigo" >> post-update
sudo chmod +x post-receive
sudo chmod +x post-update
cd ../../
