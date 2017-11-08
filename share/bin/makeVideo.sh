ffmpeg -f image2 -framerate 12 -i z.path.%03d.ppm -c:v libx264 -r 24 vid.mp4
