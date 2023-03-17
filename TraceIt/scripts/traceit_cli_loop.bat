set PNG_DIR=C:\Daten\Proj\TraceIt\out\build\x64-release\TraceIt

cd %PNG_DIR%
ffmpeg -framerate 30 -i wh_%%d.png -c:v libx264 -pix_fmt yuv420p interstellar_wormhole.mp4

pause
