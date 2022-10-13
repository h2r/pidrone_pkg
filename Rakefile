task :build do
  sh "docker build . --build-arg hostuid=$(id -u) --build-arg hostgid=$(id -g) --build-arg hostuser=$(whoami) --build-arg hostgroup=$(whoami) --tag pidrone_pkg:ente --build-arg hostname=$(hostname) --build-arg i2cgid=$(getent group i2c | cut -d: -f3)  --build-arg dialoutgid=$(getent group dialout | cut -d: -f3)  --build-arg videogid=$(getent group video | cut -d: -f3)"
end

task :create do
  sh "docker stop pidrone_pkg || true"
  sh "docker rm pidrone_pkg || true"
  sh "docker create -i -t --name pidrone_pkg --network=host --privileged --volume /home/$USER/catkin_ws:/home/$USER/catkin_ws pidrone_pkg:ente"
end

task :start do
  sh "docker start --attach -i pidrone_pkg"
end
