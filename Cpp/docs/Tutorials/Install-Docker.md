# Installing with Docker

Using [Docker](https://www.docker.com) is an easy way to install and use Klamp't on your machine.  We have a pre-built Docker image
for Jupyter Notebook, which lets you visualize Klampt through your web browser. 
We also have a very out-of-date image that uses X11 (maintained by Steve Kuznetsov).

If you are not familiar with Docker, it is a lightweight Virtual Machine (VM) that helps deploy cross-platform code without requiring
users to install complex dependencies. HOWEVER, data can be a little harder to input / extract from a VM. As a result, you will put
all your work in a "Work" folder which links up to a "Work" folder in the container.  You should put Klampt-examples, etc in that folder.
SAVE ANY FILES YOU WISH TO KEEP IN YOUR "WORK" FOLDER.  ALL OTHER FILES WILL NOT BE SAVED IF YOU RESTART THE CONTAINER.

## Installing Docker

### Unix

1. Follow the instructions on Docker's website to install Docker for Linux
   https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-repository
2. Follow the instructions on Docker's website to install Docker-compose for Linux
   https://docs.docker.com/compose/install/#install-compose

### Windows

1. Register a Docker ID. You may need one to download Docker.

2. Follow the instructions on Docker's website to install Docker on Windows

    a. If you have Windows 10 Pro, Enterprise, or Education, lookup instructions for Downloading and Installing Docker

    b. If you have Windows 10 Home or a different edition not mentioned above, download Docker Toolbox

### Mac

1. Register a Docker ID - you need one to download Docker from the Docker store for Mac

2. Follow the instructions on Docker's website to install Docker on Mac

    a. If you have a Mac from after 2010, you should be able to just install Docker

    b. If you have a Mac from earlier, you may need to install Docker-toolbox. Read the instructions for doing so



## Running the Jupyter Notebook image (recommended)

Once you have installed Docker, open the Docker command line and run

> docker pull hpbader93/jupyter-klampt

This will take some time as it downloads the container.

Afterwards, navigate to where you want the "Work" directory, and run

> mkdir Work

> git clone https://github.com/krishauser/Klampt-examples

> mv Klampt-examples Work

This will give you the Klampt-examples folder, which will contain some Jupyter Notebook examples.  Then, launch the Docker container with

> docker run -p 8888:8888 -v  "${PWD}/Work":/home/klamptuser/Work -t hpbader93/jupyter-klampt

It may be useful for you set a command line alias or shortcut, replacing ${PWD} with the absolute path for the directory holds the "Work" folder on your computer.

Finally, open up a web browser and navigate to http://192.168.99.100:8888/?token=Klampt.  This may or may not be the IP address of Docker,
and if it doesn't work, navigate to the top of your Docker console and make a note of the IP address at the top.

You should be able to run a Jupyter notebook containing Klamp't, which should look something like this:

![Jupyter image](../../../Python/docs/source/_static/images/jupyter.png)

## Running the X11 Image (out of date)

Once you've installed Docker, run the container. 

On RPM Linux (like Red Hat or Fedora), use:

```sh
$ docker run -it -e DISPLAY=unix$DISPLAY -w /etc/Klampt --name klampt -v /tmp/.X11-unix:/tmp/.X11-unix stevekuznetsov/klampt:latest
```

On Debian Linux (like Ubuntu), use:

```sh
$ docker run -it -e DISPLAY=unix$DISPLAY -w /etc/Klampt --name klampt -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/home/Klampt/.Xauthority -e XAUTHORITY=/home/Klampt/.Xauthority --net=host stevekuznetsov/klampt:latest
```

On Windows, you need to get [`Xming`](http://sourceforge.net/projects/xming/) and install it, then run:

```
> Xming.exe :0 -multiwindow -clipboard -ac
> docker run -it -e DISPLAY=192.168.99.1:0 -w /etc/Klampt --name klampt stevekuznetsov/klampt:latest
```

On Mac, run the following in iTerm.app or Terminal.app:

```sh
$ brew install socat
$ brew cask install xquartz
$ socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
```

Then, in whatever terminal you like (can be the same as above), but *not* in the Docker terminal, run:

```sh
$ docker run -it -e DISPLAY=192.168.99.1:0 -w /etc/Klampt --name klampt stevekuznetsov/klampt:latest
```

Finally, if `XQuartz` did not start automatically, start it:

```sh
$ open -a XQuartz
```

The parts of the `docker run` command are explained below. Not all parts apply to every operating system.

  Command                                       | Description                                     | Operating Systems |
| --------------------------------------------- | ----------------------------------------------- | ----------------- |
`-it`                                           | keeps the container active                      | all               |
`-e DISPLAY=unix$DISPLAY`                       | connects displays                               | all               |
`--name klampt`                                 | names the container                             | all               | 
`-w /etc/Klampt`                                | sets working directory                          | all               |
`-v /tmp/.X11-unix:/tmp/.X11-unix`              | mounts the x11 socket                           | all Linux         |
`-v $HOME/.Xauthority:/home/Klampt/.Xauthority` | mounts the Xauthority files                     | Debian Linux      |
`-e XAUTHORITY=/home/Klampt/.Xauthority`        | allows the container to connect to the X server | Debian Linux      |
`--net=host`                                    | places container inside host's network stack    | Debian Linux      |
`stevekuznetsov/klampt:latest`                  | decides which image to run                      | all               |

#### Using Klamp't 

Once you've created your own scripts that you wish to run, save them in some directory on your host, and amend the `docker run` command used above to include `-v /path/to/your/data:/home/Klampt/data`. For instance, the RPM Linux `run` command, if there is data at `/home/myuser/klamptdata`, would look like:

```sh
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --name klampt \
	-v /home/myuser/klamptdata:/home/Klampt/data \ # shares data with the container
	stevekuznetsov/klampt:latest
```

Then, use the internal tools inside of the container (i.e. `SimTest`, `RobotTest`, etc.) on your files. They will be found in `/home/Klampt/data`. 



