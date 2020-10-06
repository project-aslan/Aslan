# ASLAN docker

## Prerequisites

- **Nvidia drivers**:  
Install the Nvidia drivers to enable the graphics inside the docker. Make sure to install the drivers that match the hardware on your target machine. Following the instructions [here](http://www.linuxandubuntu.com/home/how-to-install-latest-nvidia-drivers-in-linux).  
After the installation has finished, reboot your machine `sudo reboot`.
Check that nvidia has been properly installed by typing: `nvidia-smi`.

- **Docker and Nvidia-docker**:

```
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y docker-ce nvidia-docker
```
For more information please visit [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-1.0))

## Build Aslan docker

**You can pull a pre-built image from the Project Aslan [DockerHub](https://hub.docker.com/r/projaslan/aslan) and skip this step.**  

To build a docker image locally (default image name: aslan_docker), execute the following script. It may take a few minutes:
```
cd aslan_docker/
./build.bash
```

## Run Aslan docker

Pull and run a docker image from the DockerHub:
```
cd aslan_docker/
sudo ./run.bash
```
To run another docker image, replace the last line on the `run.bash` script, `projaslan/aslan:melodic`,  with the name of the image you would like to run eg. `aslan_docker`.

To open a new terminal from inside the docker:
```
sudo nvidia-docker exec -it aslan_container bash
```

## Copy files 

To copy files from the local machine to the docker container:
```
sudo nvidia-docker cp <file_path>/file_name aslan_container:/home/$USERNAME/Aslan/file_name
```

To copy files from the docker container to the local machine:
```
sudo nvidia-docker cp aslan_container:/home/$USERNAME/<file_path>/file_name <file_path>/file_name
```
**$USERNAME** default value is `aslan_user` in the Dockerfile.

## Save docker image from container

To save a docker image from a docker container:  
```
sudo nvidia-docker commit aslan_container <docker_image_name>
```

## Save docker image locally

To save an image as rar file:
```
sudo nvidia-docker save aslan_docker -o aslan_docker.rar
```

## Additional resources

Please visit [here](https://docs.docker.com/engine/reference/commandline/docker/)