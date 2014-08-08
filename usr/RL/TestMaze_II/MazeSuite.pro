TEMPLATE = subdirs
SUBDIRS = $$system( if [ "$(echo $HOSTNAME)" = "snuffleupagus" ]; then echo "BatchWorker_quiet BatchWorker_verbose"; else echo "MazeGUI GTest BatchWorker_quiet BatchWorker_verbose"; fi )
message("$$TARGET contains:")
message("--> $$SUBDIRS")
