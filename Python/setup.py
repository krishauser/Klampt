#!/usr/bin/env python

from distutils.core import setup,Extension
import distutils.util
import os
import glob


on_win32 = (distutils.util.get_platform()=='win32')
on_win64 = (distutils.util.get_platform() in ['win-amd64','win-ia64'])
on_cygwin = distutils.util.get_platform().startswith('cygwin')

if on_win64:
	raise ValueError("Unable to compile on win64 yet")

#these probably do not need to be changed
klamptDir = '..'
krisLibraryDir = klamptDir+'/'+'Library/KrisLibrary'
odedir = klamptDir+'/'+'Library/ode-0.11.1'
#odedir = klamptDir+'/'+'Library/ode-0.12'
gluidir = klamptDir+'/'+'Library/glui-2.36'
#optional
assimpDir=klamptDir+'/'+'Library/assimp--3.0.1270-sdk'
haveassimp = os.path.isdir(assimpDir)
tinyxmlDir = klamptDir+'/'+'Library/tinyxml'
#if ODE_DOUBLE is set to true, turn this to true
odedouble = True

includeDirs = [klamptDir,krisLibraryDir,tinyxmlDir,odedir+'/include','/usr/local/include','/usr/include','.']

tinyxmlLibDir = tinyxmlDir
odelibdir = odedir+'/ode/src/.libs'
#uncomment this if you used "make install" for ODE
#odelibdir = '/usr/local/lib'
gluilibdir = gluidir+'/src/lib'
assimpLibDir = assimpDir+'/lib'

libdirs = [klamptDir+'/lib',krisLibraryDir+'/lib',tinyxmlLibDir,odelibdir,gluilibdir,assimpLibDir,'/usr/lib','/usr/local/lib']


commonfiles = ['pyerr.cpp']

rssourcefiles = commonfiles + ['robotsim.cpp','robotik.cpp','robotsim_wrap.cxx']
mpsourcefiles = commonfiles + ['motionplanning.cpp','motionplanning_wrap.cxx']
cosourcefiles = commonfiles + ['collide.cpp','collide_wrap.cxx']
rfsourcefiles = commonfiles + ['rootfind.cpp','pyvectorfield.cpp','rootfind_wrap.cxx']

#compilation defines
commondefines = []
if on_win32:
	commondefines.append(('WIN32',None))
rsdefines = commondefines+[('TIXML_USE_STL',None)]
if odedouble:
    rsdefines.append(('dDOUBLE',None))
else:
    rsdefines.append(('dSINGLE',None))
if on_win32:
	libdirs = [klamptDir+'/msvc/Release',klamptDir+'/Library',assimpLibDir+'/assimp_release-dll_win32']

#needed for KrisLibrary to link
kllibs = ['KrisLibrary','tinyxml','glpk','glui','GL']
if on_cygwin:
    kllibs[-1] = 'opengl32'
    kllibs.append('glut32')
#different naming, no TINYXML on windows
if on_win32:
	kllibs=['KrisLibrary','tinyxml_STL','glui32','glut32','opengl32','winmm','user32']

#uncomment this if KrisLibrary was built with GSL support
#kllibs += ['gsl']

#needed for Klampt to link
libs = ['Klampt']+kllibs+['ode']
extra_data = []
#different naming on windows
if on_win32:
	libs[-1]='ode_double'
#link to assimp if assimp support is desired
if haveassimp:
    libs.append('assimp')
    if on_win32:
	    extra_data.append('Assimp32.dll')


setup(name='Klampt',
      version='0.5',
      description="PyKlamp't",
      author='Kris Hauser',
      author_email='hauserk@indiana.edu',
      url='https://github.com/krishauser/Klampt',
      ext_modules=[Extension('klampt._robotsim',
                             [os.path.join('klampt/src',f) for f in rssourcefiles],
                             include_dirs=includeDirs,
                             define_macros=rsdefines,
                             library_dirs=libdirs,
                             libraries=libs,
                             language='c++'),
                   Extension('klampt._motionplanning',
                             [os.path.join('klampt/src',f) for f in mpsourcefiles],
                             include_dirs=includeDirs,
							 define_macros=commondefines,
                             library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++'),
                   Extension('klampt._collide',
                             [os.path.join('klampt/src',f) for f in cosourcefiles],
                             include_dirs=includeDirs,
							 define_macros=commondefines,
                             library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++'),
                   Extension('klampt._rootfind',
                             [os.path.join('klampt/src',f) for f in rfsourcefiles],
                             include_dirs=includeDirs,
							 define_macros=commondefines,
                             library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++')],
      py_modules=['klampt.robotsim','klampt.motionplanning','klampt.collide','klampt.rootfind'],
      packages=['klampt'],
      package_data={'klampt':extra_data}
     )

