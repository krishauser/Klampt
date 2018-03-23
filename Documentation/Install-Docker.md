#Klamp't [![Build Status](https://travis-ci.org/krishauser/Klampt.svg?branch=master)](https://travis-ci.org/arocchi/Klampt)

A robot simulation, planning, and control package from Indiana University /
Duke University.

More information can be found on the website:

   http://klampt.org

The manual is also available in the Documentation folder.

## Installation

More information on installing from source on Linux or Mac OSX and binaries on Windows can be found here:

   http://motion.pratt.duke.edu/klampt/tutorial_install.html

### Using Docker

Using [Docker](https://www.docker.com) to run this application is the easiest way to install and use Klamp't on your machine. In order to use this method, you must download and set up Docker. Tutorials are available for [OSX](http://docs.docker.com/mac/started/), [Linux](http://docs.docker.com/linux/started) and [Windows](http://docs.docker.com/windows/started). 

#### Running the Image

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




## Contact

Kris Hauser
Duke University
kris.hauser@duke.edu

