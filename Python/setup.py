from distutils.core import setup,Extension
import distutils.util
import os
import glob

#Parse settings from CMake
includeDirs = '/usr/include;/home/hauser/gitlibs/Klampt/Library/glui-2.36/src/include;/usr/include;/usr/include;/usr/include;/home/hauser/gitlibs/tinyxml;/home/hauser/gitlibs/Klampt/Library/assimp--3.0.1270-sdk/include;/usr/local/include/KrisLibrary;/usr/local/include;/home/hauser/gitlibs/Klampt'.split(';')
libs = '/home/hauser/gitlibs/Klampt/lib/libKlampt.a;/usr/local/lib/libKrisLibrary.a;/usr/lib/libboost_thread-mt.dll.a;/usr/lib/libboost_system-mt.dll.a;/home/hauser/gitlibs/Klampt/Library/glui-2.36/src/lib/libglui.a;/usr/lib/libglut.dll.a;/usr/lib/libXmu.dll.a;/usr/lib/libXi.dll.a;/usr/lib/libGLU.dll.a;/usr/lib/libGL.dll.a;/usr/lib/libSM.dll.a;/usr/lib/libICE.dll.a;/usr/lib/libX11.dll.a;/usr/lib/libXext.dll.a;/usr/lib/libglpk.dll.a;/home/hauser/gitlibs/tinyxml/libtinyxml.a;/home/hauser/gitlibs/Klampt/Library/assimp--3.0.1270-sdk/lib/libassimp.dll.a;/usr/local/lib/libode.a'.split(';')
defs = '-DUSE_BOOST_THREADS=1;-DHAVE_GLUI=1;-DHAVE_GLUT=1;-DHAVE_GLPK=1;-DHAVE_TIXML=1;-DTIXML_USE_STL;-DHAVE_ASSIMP=1;-I/usr/local/include -DdSINGLE'.split(';')

#Parse libraries
libdirs = []
libnames = []
for l in libs:
	path,fn = os.path.split(l)
	if path not in libdirs:
		libdirs.append(path)
	if fn.startswith('lib'):
		name = fn.split('.')[0]
		libnames.append(name[3:])
libs = libnames

#Parse definitions
commondefines = []
#split whitespace
defs = sum([d.split() for d in defs],[])
#parse -D or -I defines
for d in defs:
	if d.startswith("-I"):
		includeDirs.append(d[2:])
	elif d.startswith("-D"):
		d = d[2:]
		if d.find("=") >= 0:
			commondefines.append(tuple(d.split("=")))
		else:
			commondefines.append((d,None))
	else:
		commondefines.append((d,None))

commonfiles = ['pyerr.cpp']
rssourcefiles = commonfiles + ['robotsim.cpp','robotik.cpp','robotsim_wrap.cxx']
mpsourcefiles = commonfiles + ['motionplanning.cpp','motionplanning_wrap.cxx']
cosourcefiles = commonfiles + ['collide.cpp','collide_wrap.cxx']
rfsourcefiles = commonfiles + ['rootfind.cpp','pyvectorfield.cpp','rootfind_wrap.cxx']

setup(name='Klampt',
      version='0.6.0',
      description="PyKlamp't",
      author='Kris Hauser',
      author_email='hauserk@indiana.edu',
      url='https://github.com/krishauser/Klampt',
      ext_modules=[Extension('klampt._robotsim',
                             [os.path.join('klampt/src',f) for f in rssourcefiles],
                             include_dirs=includeDirs,
                             define_macros=commondefines,
                             library_dirs=libdirs,
                             libraries=libs,
                             language='c++'),
                   Extension('klampt._motionplanning',
                             [os.path.join('klampt/src',f) for f in mpsourcefiles],
                             include_dirs=includeDirs,
                             define_macros=commondefines,
                             library_dirs=libdirs,
                             libraries=libs,
                             language='c++'),
                   Extension('klampt._collide',
                             [os.path.join('klampt/src',f) for f in cosourcefiles],
                             include_dirs=includeDirs,
                             define_macros=commondefines,
                             library_dirs=libdirs,
                             libraries=libs,
                             language='c++'),
                   Extension('klampt._rootfind',
                             [os.path.join('klampt/src',f) for f in rfsourcefiles],
                             include_dirs=includeDirs,
                             define_macros=commondefines,
                             library_dirs=libdirs,
                             libraries=libs,
                             language='c++')],
      py_modules=['klampt.robotsim','klampt.motionplanning','klampt.collide','klampt.rootfind'],
      packages=['klampt']
     )

