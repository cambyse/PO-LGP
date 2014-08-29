TEMPLATE = subdirs
SUBDIRS = $$system( if [ "$HOSTNAME" = "TheBox" -o "$HOSTNAME" = "ThinkPet" ]; then echo "MazeGUI GTest BatchWorker_quiet BatchWorker_verbose"; else echo "BatchWorker_quiet BatchWorker_verbose"; fi )
message("$$TARGET contains:")
message("--> $$SUBDIRS")
