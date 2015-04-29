make unpack-deps
make deps
#let the ODE library be loadable from inside this path without installing
#which might override some other ODE installation 
echo "Adding local ODE path to ld.so.conf.d"
echo ${PWD}/ode-0.11.1/ode/src/.libs/ > ode.conf
sudo mv ode.conf /etc/ld.so.conf.d 

