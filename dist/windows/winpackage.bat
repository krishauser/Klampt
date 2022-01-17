:: 
:: script that builds and uploads everything on Windows Visual Studio 2015
:: (assumes CMake is set up properly, Python is installed, etc)
:: (assumes this is run on an administrator developer command prompt from the Klampt folder)
:: (assumes the zip command line tool is available.  See GnuWin32 zip.)

:: configuration variables
SET klamptversion=0.9.0
:: dependency libraries may be kept back to a prior version
SET klamptdepversion=0.9.0
::    this is used for Python build (VS 2015)
SET VS90COMNTOOLS=%VS140COMNTOOLS%
SET PYTHON35_32=D:\Python35-32\python.exe
SET PYTHON35_64=D:\Python35\python.exe
SET PYTHON36_32=D:\Python36-32\python.exe
SET PYTHON36_64=D:\Python36\python.exe
SET PYTHON37_32=D:\Python37-32\python.exe
SET PYTHON37_64=D:\Python37\python.exe
SET PYTHON38_32=D:\Python38-32\python.exe
SET PYTHON38_64=D:\Python38\python.exe
SET PYTHON39_32=D:\Python39-32\python.exe
SET PYTHON39_64=D:\Python39\python.exe
SET PYTHON_32_VERSIONS=%PYTHON35_32% %PYTHON36_32% %PYTHON37_32% %PYTHON38_32% %PYTHON39_32%
SET PYTHON_64_VERSIONS=%PYTHON35_64% %PYTHON36_64% %PYTHON37_64% %PYTHON38_64% %PYTHON39_32%

for %%P in (%PYTHON_32_VERSIONS%) do (
  %%P --version
  )
for %%P in (%PYTHON_64_VERSIONS%) do (
  %%P --version
  )

:: update Klampt
:: git pull
:: if %errorlevel% neq 0 exit /b %errorlevel%

:: update KrisLibrary
pushd Cpp\Dependencies\KrisLibrary
:: git pull
if %errorlevel% neq 0 exit /b %errorlevel%
popd

:: 32 bit build
SET buildfolder=msvc

:: build KrisLibrary, both release and debug and copy into Cpp/Dependencies directory
pushd Cpp\Dependencies\KrisLibrary
devenv %buildfolder%\KrisLibrary.sln /build Release
if %errorlevel% neq 0 exit /b %errorlevel%
devenv %buildfolder%\KrisLibrary.sln /build Debug
if %errorlevel% neq 0 exit /b %errorlevel%
copy /Y %buildfolder%\lib\Release\KrisLibrary.lib ..\
copy /Y %buildfolder%\lib\Debug\KrisLibraryd.lib ..\
popd

:: build Klamp't
devenv %buildfolder%\Klampt.sln /build Release
:: (python doesnt build right here...) if %errorlevel% neq 0 exit /b %errorlevel%
devenv %buildfolder%\Klampt.sln /build Release /project PACKAGE
:: (python doesnt build right here...) if %errorlevel% neq 0 exit /b %errorlevel%

SET errorlevel=0

:: build Klamp't Python bindings
copy /y %buildfolder%\Python\setup.py Python\
pushd Python
for %%P in (%PYTHON_32_VERSIONS%) do (
    %%P setup.py build_ext
    if %errorlevel% neq 0 exit /b %errorlevel%
    %%P setup.py install
    if %errorlevel% neq 0 exit /b %errorlevel%
    %%P setup.py bdist_wheel
    if %errorlevel% neq 0 exit /b %errorlevel%
  )
popd

:: 64 bit build
SET buildfolder=msvc64

:: build KrisLibrary, both release and debug and copy into Cpp/Dependencies directory
pushd Cpp\Dependencies\KrisLibrary
devenv %buildfolder%\KrisLibrary.sln /build Release
if %errorlevel% neq 0 exit /b %errorlevel%
devenv %buildfolder%\KrisLibrary.sln /build Debug
if %errorlevel% neq 0 exit /b %errorlevel%
copy /Y %buildfolder%\lib\Release\KrisLibrary.lib ..\x64
copy /Y %buildfolder%\lib\Debug\KrisLibraryd.lib ..\x64
popd

:: build Klampt
:: Qt5 doesn't have a 64-bit build, don't build apps
devenv %buildfolder%\Klampt.sln /build Release /project Klampt
:: (python doesnt build right here...) if %errorlevel% neq 0 exit /b %errorlevel%
:: devenv %buildfolder%\Klampt.sln /build Release /project PACKAGE
:: (python doesnt build right here...) if %errorlevel% neq 0 exit /b %errorlevel%

:: build Klamp't Python bindings
copy /y %buildfolder%\Python\setup.py Python\
pushd Python
for %%P in (%PYTHON_64_VERSIONS%) do (
    %%P setup.py build_ext
    if %errorlevel% neq 0 exit /b %errorlevel%
    %%P setup.py install
    if %errorlevel% neq 0 exit /b %errorlevel%
    %%P setup.py bdist_wheel
    if %errorlevel% neq 0 exit /b %errorlevel%
  )
popd


:: zip dependency libraries
::   release
set depfolder=Klampt-%klamptdepversion%.win32-deps-vs2015
mkdir %depfolder%
pushd Cpp\Dependencies
for %%I in (assimp.dll glpk_4_61.dll glpk_4_61.lib glew32.dll glew32.lib KrisLibrary.lib ode_double.lib tinyxml_STL.lib libcurl.lib) do copy /Y %%I ..\..\%depfolder%
if %errorlevel% neq 0 exit /b %errorlevel%
popd
pushd %depfolder%
zip ..\%depfolder%.zip *
popd
if %errorlevel% neq 0 exit /b %errorlevel%

::   debug
set depfolder=Klampt-%klamptdepversion%.win32-deps-vs2015d
mkdir %depfolder%
pushd Cpp\Dependencies
for %%I in (assimpd.dll  glpk_4_61.dll glpk_4_61.lib glew32.dll glew32.lib KrisLibraryd.lib ode_doubled.lib ode-0.14\lib\DebugDoubleLib\ode.pdb tinyxmld_STL.lib libcurl.lib) do copy /Y %%I ..\..\%depfolder%
if %errorlevel% neq 0 exit /b %errorlevel%
popd
pushd %depfolder%
zip ..\%depfolder%.zip *
popd
if %errorlevel% neq 0 exit /b %errorlevel%

::   release x64
set depfolder=Klampt-%klamptdepversion%.win64-deps-vs2015
mkdir %depfolder%
pushd Cpp\Dependencies
for %%I in (x64\assimp.dll glpk_4_61.dll glpk_4_61.lib glew32.dll glew32.lib x64\KrisLibrary.lib x64\ode_double.lib x64\tinyxml_STL.lib x64\libcurl.lib ) do copy /Y %%I ..\..\%depfolder%
if %errorlevel% neq 0 exit /b %errorlevel%
popd
pushd %depfolder%
zip ..\%depfolder%.zip *
popd
if %errorlevel% neq 0 exit /b %errorlevel%

::   debug x64
set depfolder=Klampt-%klamptdepversion%.win64-deps-vs2015d
mkdir %depfolder%
pushd Cpp\Dependencies
for %%I in (x64\assimpd.dll glpk_4_61.dll glpk_4_61.lib glew32.dll glew32.lib x64\KrisLibraryd.lib x64\ode_doubled.lib x64\tinyxmld_STL.lib x64\libcurl.lib ) do copy /Y %%I ..\..\%depfolder%
if %errorlevel% neq 0 exit /b %errorlevel%
popd
pushd %depfolder%
zip ..\%depfolder%.zip *
popd
if %errorlevel% neq 0 exit /b %errorlevel%



:: upload files to motion website
copy msvc\Klampt-%klamptversion%-win32.msi d:\iml-webpage\software\klampt\0.9\
:: Qt5 doesn't have a 64-bit version
:: copy msvc64/Klampt-%klamptversion%-win64.msi d:\iml-webpage\software\klampt\0.9\
if %errorlevel% neq 0 exit /b %errorlevel%
copy Klampt-%klamptdepversion%.win32-deps-vs2015.zip d:\iml-webpage\software\klampt\0.9\
copy Klampt-%klamptdepversion%.win32-deps-vs2015d.zip d:\iml-webpage\software\klampt\0.9\
copy Klampt-%klamptdepversion%.win64-deps-vs2015.zip d:\iml-webpage\software\klampt\0.9\
copy Klampt-%klamptdepversion%.win64-deps-vs2015d.zip d:\iml-webpage\software\klampt\0.9\
if %errorlevel% neq 0 exit /b %errorlevel%



