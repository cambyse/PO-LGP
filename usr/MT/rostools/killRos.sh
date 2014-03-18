#!/bin/sh
kill -9 `ps -ef | grep ros | grep -v grep | awk '{print $2}'`
