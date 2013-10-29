#!/usr/bin/env python

from distutils.core import setup,Extension
import distutils.util
import os
import glob

#these probably do not need to be changed
klamptDir = '../..'
krisLibraryDir = 'Library/KrisLibrary'
odedir = 'Library/ode-0.11'
#odedir = 'Library/ode-0.12'
gluidir = 'Library/glui-2.36'
#optional
assimpDir='Library/assimp--3.0.1270-sdk'
tinyxmlDir = 'Library/tinyxml'

includeDirs = [klamptDir,krisLibraryDir,tinyxmlDir,odedir+'/include','/usr/include','.']

tinyxmlLibDir = tinyxmlDir
#odelibdir = odedir+'/ode/src/.libs'
odelibdir = '/usr/local/lib'
gluilibdir = gluidir+'/src/lib'
assimpLibDir = assimpDir+'/lib'

libdirs = [klamptDir+'/lib',krisLibraryDir+'/lib',tinyxmlLibDir,odelibdir,gluilibdir,assimpLibDir,'/usr/lib']


on_cygwin = distutils.util.get_platform().startswith('cygwin')

commonfiles = ['pyerr.cpp']

rssourcefiles = commonfiles + ['robotsim.cpp','robotik.cpp','robotsim_wrap.cxx']
mpsourcefiles = commonfiles + ['motionplanning.cpp','motionplanning_wrap.cxx']
cosourcefiles = commonfiles + ['collide.cpp','collide_wrap.cxx']
rfsourcefiles = commonfiles + ['rootfind.cpp','pyvectorfield.cpp','rootfind_wrap.cxx']

#needed for KrisLibrary to link
kllibs = ['KrisLibrary','tinyxml','glpk','glui','GL']
if on_cygwin:
    kllibs[-1] = 'opengl32'
    kllibs.append('glut32')

#uncomment this if KrisLibrary was built with GSL support
kllibs += ['gsl']

#needed for Klampt to link
libs = ['Klampt']+kllibs+['ode']
#switch to this if assimp support is desired
#libs = ['RobotSim']+kllibs+['ode','assimp']

setup(name='RobotSim',
      version='0.2',
      description='RobotSim extension module',
      author='Kris Hauser',
      author_email='hauserk@indiana.edu',
      url='https://github.com/krishauser/KrisLibrary',
      ext_modules=[Extension('_robotsim',
                             [os.path.join('src',f) for f in rssourcefiles],
                             include_dirs=includeDirs,
                             define_macros=[('TIXML_USE_STL',None),('dDOUBLE',None)],
                             library_dirs=libdirs,
                             libraries=libs,
                             language='c++'),
                   Extension('_motionplanning',
                             [os.path.join('src',f) for f in mpsourcefiles],
                             include_dirs=includeDirs,
                             library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++'),
                   Extension('_collide',
                             [os.path.join('src',f) for f in cosourcefiles],
                             include_dirs=includeDirs,
                             library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++'),
                   Extension('_rootfind',
                             [os.path.join('src',f) for f in rfsourcefiles],
                             include_dirs=includeDirs,
                             library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++')],
      py_modules=['robotsim.py','motionplanning.py','collide.py','rootfind.py']
     )

