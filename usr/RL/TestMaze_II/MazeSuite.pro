TEMPLATE = subdirs
SUBDIRS = $$system( if [ "$HOSTNAME" = "snuffleupagus" -o "$HOSTNAME" = "gonzo"  -o "$HOSTNAME" = "cloudworker" ]; then echo "BatchWorker_quiet BatchWorker_verbose"; else echo "MazeGUI GTest BatchWorker_quiet BatchWorker_verbose"; fi )
message("$$TARGET contains:")
message("--> $$SUBDIRS")
