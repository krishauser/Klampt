#!/bin/bash

# 
# We will almost certainly will mount an external volume to hold 
# the 'guest' user directory (so that their work persists between
# container restarts). However, there is no guarantee that the
# container user owns or can write to that external volume, so
# make sure they own their home directory
#
chown -R klamptuser ~klamptuser ; chgrp -R users ~klamptuser

#
# set the password for the user 'klamptuser' based on environment variable. 
#

if [ ! -z $USERPASS ] 
then
  /bin/echo "klamptuser:$USERPASS" | /usr/sbin/chpasswd
  unset USERPASS
fi

exit 0
