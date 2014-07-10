#!/bin/sh
ps -ef | grep ros | grep -v grep | awk '{print $2}'
kill -9 `ps -ef | grep ros | grep -v grep | awk '{print $2}'`
