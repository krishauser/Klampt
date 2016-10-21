REM 
REM script that builds and uploads everything on Windows Visual Studio 2015
REM (assumes CMake is set up properly, Python is installed, etc)

REM configuration variables
SET buildfolder=msvc14
SET python=C:\Python27\python.exe
SET klamptversion=0.6.2

REM update KrisLibrary, build both release and debug and copy into Library directory
cd Library\KrisLibrary
git pull
if %errorlevel% neq 0 exit /b %errorlevel%
devenv %buildfolder%\KrisLibrary.sln /build Release
if %errorlevel% neq 0 exit /b %errorlevel%
devenv %buildfolder%\KrisLibrary.sln /build Debug
if %errorlevel% neq 0 exit /b %errorlevel%
copy /Y lib\Release\KrisLibrary.lib ..\
copy /Y lib\Debug\KrisLibrary.lib ..\KrisLibraryd.lib
cd ..\..\

REM update and build Klamp't
git pull
if %errorlevel% neq 0 exit /b %errorlevel%
devenv %buildfolder%\Klampt.sln /build Release
REM (python doesnt build right here...) if %errorlevel% neq 0 exit /b %errorlevel%
devenv %buildfolder%\Klampt.sln /build Release /project PACKAGE
REM (python doesnt build right here...) if %errorlevel% neq 0 exit /b %errorlevel%

REM build Klamp't Python bindings
cd Python
%python% setup.py build_ext
if %errorlevel% neq 0 exit /b %errorlevel%
%python% setup.py install
%python% setup.py bdist_wininst
if %errorlevel% neq 0 exit /b %errorlevel%
cd ..

REM zip dependency libraries
REM   release
mkdir Klampt-%klamptversion%.win32-deps-vs2015
cd Library
for %I in (assimp--3.0.1270-sdk\lib\assimp_release-dll_win32\* Assimp32.dll glpk_4_52.dll glpk_4_52.lib glui32.lib glut32.dll KrisLibrary.lib ode_double.lib tinyxml_STL.lib) do copy /Y %I ..\Klampt-%klamptversion%.win32-deps-vs2015
if %errorlevel% neq 0 exit /b %errorlevel%
cd ..\Klampt-%klamptversion%.win32-deps-vs2015
zip ..\Klampt-%klamptversion%.win32-deps-vs2015.zip *
cd ..
if %errorlevel% neq 0 exit /b %errorlevel%

REM   debug
mkdir Klampt-%klamptversion%.win32-deps-vs2015d
cd Library
for %I in (assimp--3.0.1270-sdk\lib\assimp_debug-dll_win32\* Assimp32d.dll  glpk_4_52.dll glpk_4_52.lib glui32d.lib "glui-2.36\src\msvc\Debug\_glui library.pdb" glut32.dll KrisLibraryd.lib ode_doubled.lib ode-0.14\lib\DebugDoubleLib\ode.pdb tinyxmld_STL.lib) do copy /Y %I ..\Klampt-%klamptversion%.win32-deps-vs2015d
if %errorlevel% neq 0 exit /b %errorlevel%
cd ..\Klampt-%klamptversion%.win32-deps-vs2015d
zip ..\Klampt-%klamptversion%.win32-deps-vs2015d.zip *
cd ..
if %errorlevel% neq 0 exit /b %errorlevel%

REM upload files to motion website
cd %buildfolder%
scp Klampt-%klamptversion%-win32.msi hauser@motion.pratt.duke.edu:software/
if %errorlevel% neq 0 exit /b %errorlevel%
cd ..
scp Klampt-%klamptversion%.win32-deps-vs2015.zip hauser@motion.pratt.duke.edu:software/
scp Klampt-%klamptversion%.win32-deps-vs2015d.zip hauser@motion.pratt.duke.edu:software/
if %errorlevel% neq 0 exit /b %errorlevel%
cd Python\dist
scp Klampt-%klamptversion%.win32-py2.7.exe hauser@motion.pratt.duke.edu:software/
cd ..\..\


