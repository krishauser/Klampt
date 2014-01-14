
cd ..\Library\KrisLibrary\msvc
msbuild KrisLibrary.sln /property:Configuration=Debug
if %errorlevel% neq 0 exit /b %errorlevel% 
msbuild KrisLibrary.sln /property:Configuration=Release
if %errorlevel% neq 0 exit /b %errorlevel% 

cd ..\..\..\msvc
msbuild Klampt.sln /property:Configuration=Debug
if %errorlevel% neq 0 exit /b %errorlevel% 
msbuild Klampt.sln /property:Configuration=Release
if %errorlevel% neq 0 exit /b %errorlevel% 

set PATH=C:\Python27\;%PATH%
set VS90COMNTOOLS=%VS100COMNTOOLS%
cd ..\Python
python setup.py install
if %errorlevel% neq 0 exit /b %errorlevel% 
python setup.py bdist_wininst

cd ..\msvc