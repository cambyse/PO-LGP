#!/bin/sh

rostopic pub -r 10 /hello_world std_msgs/String "Hello world..."
