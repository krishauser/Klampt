mkdir ToZip

REM cp Debug/*.exe Debug/*.lib Debug/*.dll ToZip
REM if %errorlevel% neq 0 exit /b %errorlevel% 
REM zip.vbs ToZip ..\Klampt-0.5.win32-vs2010d.zip
REM if %errorlevel% neq 0 exit /b %errorlevel% 
REM rm ToZip/*

REM cp Release/*.exe Release/*.lib Release/*.dll Release\ToZip
REM if %errorlevel% neq 0 exit /b %errorlevel% 
REM zip.vbs ToZip ..\Klampt-0.5.win32-vs2010.zip
REM if %errorlevel% neq 0 exit /b %errorlevel% 
REM rm ToZip/*

cp ../Library/assimp--3.0.1270-sdk/lib/assimp_debug-dll_win32/* ToZip
cp ../Library/Assimp32d.dll ToZip
cp ../Library/glpk_4_52.dll ../Library/glpk_4_52.lib ToZip
cp ../Library/glui-2.36/src/msvc/Debug/_glui\ library.pdb ToZip
cp ../Library/glui32d.lib ToZip 
cp ../Library/KrisLibraryd.lib ToZip
cp ../Library/ode-0.11.1/lib/DebugDoubleLib/*.pdb ToZip
cp ../Library/ode_doubled.lib ToZip
cp ../Library/tinyxmld_STL.lib ToZip
if %errorlevel% neq 0 exit /b %errorlevel% 
zip.vbs ToZip ..\Klampt-0.5.win32-deps-vs2010d.zip
rm ToZip/*

cp ../Library/assimp--3.0.1270-sdk/lib/assimp_release-dll_win32/* ToZip
cp ../Library/Assimp32.dll ToZip
cp ../Library/glpk_4_52.dll ../Library/glpk_4_52.lib ToZip
cp ../Library/glui32.lib ToZip 
cp ../Library/KrisLibrary.lib ToZip
cp ../Library/ode_double.lib ToZip
cp ../Library/tinyxml_STL.lib ToZip
if %errorlevel% neq 0 exit /b %errorlevel% 
zip.vbs ToZip ..\Klampt-0.5.win32-deps-vs2010.zip
rm ToZip/*

rmdir /Q /S ToZip
