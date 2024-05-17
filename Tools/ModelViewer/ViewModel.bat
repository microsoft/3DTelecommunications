set blender_path="C:\Program Files\Blender Foundation\"
set loader_script="%~dp0___blender_model_loader.py"
set default_model="%~dp0Patient3DModel-HighQuality.obj"

if exist "%blender_path%":

    for /f "delims=" %%F in ('dir %blender_path%* /b /o-n') do set version=%%F
    set "blender_path=%blender_path%%version%\blender.exe"
    set blender_path=%blender_path:"=%

    
    if "%~1"=="" ( "%blender_path%" -P %loader_script% -- %default_model% ) else ( "%blender_path%" -P %loader_script% -- %1 )

    
PAUSE