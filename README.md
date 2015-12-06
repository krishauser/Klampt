#Klamp't

A robot simulation, planning, and control package from Indiana University /
Duke University.

More information can be found on the website:

   http://klampt.org

The manual is also available in the Documentation folder.

## Installation
### Using Docker

Using [Docker](https://www.docker.com) to run this application is the easiest way to install and use Klamp't on your machine. In order to use this method, you must download and set up Docker. Tutorials are available for [OSX](http://docs.docker.com/mac/started/), [Linux](http://docs.docker.com/linux/started) and [Windows](http://docs.docker.com/windows/started). 

#### Running the Image

Once you've installed Docker, pull the Klamp't image:

```sh
$ docker pull stevekuznetsov/klampt
```

Next, run the container. On RPM Linux (like Red Hat or Fedora), use:

```sh
$ docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix  -e DISPLAY=unix$DISPLAY --name klampt stevekuznetsov/klampt:latest
```

The command is explained below:

```sh
$ docker run -it \                      # keeps the container active
	-v /tmp/.X11-unix:/tmp/.X11-unix \  # mounts the x11 socket
	-e DISPLAY=unix$DISPLAY \           # connects displays
	--name klampt \                     # names the container
	stevekuznetsov/klampt:latest
```

On Debian Linux (like Ubuntu), use:

```sh
$ docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix  -e DISPLAY=unix$DISPLAY -v $HOME/.Xauthority:/home/Klampt/.Xauthority -e XAUTHORITY=/home/Klampt/.Xauthority --net=host --name klampt stevekuznetsov/klampt:latest
```

The command is explained below:

```sh
$ docker run -it \                                  # keeps the container active
	-v /tmp/.X11-unix:/tmp/.X11-unix \              # mounts the x11 socket
	-e DISPLAY=unix$DISPLAY \                       # connects displays
	-v $HOME/.Xauthority:/home/Klampt/.Xauthority \ # mounts the Xauthority files
	-e XAUTHORITY=/home/Klampt/.Xauthority \        # allows the container to connect to the X server
	--net=host                                      # places container inside host's network stack
	--name klampt \                                 # names the container
	stevekuznetsov/klampt:latest
```

On Windows, you need to get [`Xming`](http://sourceforge.net/projects/xming/) and install it, then run:

```
> Xming.exe :0 -multiwindow -clipboard -ac
> docker run -e DISPLAY=192.168.99.1:0 stevekuznetsov/klampt:latest
```

On Mac, run the following in iTerm.app or Terminal.app:

```sh
$ brew install socat
$ brew cask install xquartz
$ socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
```

Then, in whatever terminal you like (can be the same as above):

```sh
$ docker run -e DISPLAY=192.168.99.1:0 stevekuznetsov/klampt:latest
```

Finally, if `XQuartz` did not start automatically, start it:

```sh
$ open -a XQuartz
```

#### Using Klamp't 

Once you've created your own scripts that you wish to run, save them in some directory on your host, and amend the `docker run` command used above to include `-v /path/to/your/data:/etc/Klampt/data/usr`. For instance, the RPM Linux `run` command, if there is data at `/home/myuser/klamptdata`, would look like:

```sh
docker run -it \                                 # keeps the container active
	-v /tmp/.X11-unix:/tmp/.X11-unix \           # mounts the x11 socket
	-e DISPLAY=unix$DISPLAY \                    # connects displays
	--name klampt \                              # names the container
	-v /home/myuser/klamptdata:/home/Klampt/data # shares data with the container
	stevekuznetsov/klampt:latest
```

Then, use the internal tools inside of the container (i.e. `SimTest`, `RobotTest`, etc.) on your files. They will be found in `/home/Klampt/data/usr`. 

### Local Installation 

A local installation tutorial for Linux and Windows can be found here:

   http://motion.pratt.duke.edu/klampt/tutorial_install.html


## Contact

Kris Hauser
Duke University
kris.hauser@duke.edu

