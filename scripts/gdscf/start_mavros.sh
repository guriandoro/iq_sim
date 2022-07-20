#!/bin/bash

sleep 30

roslaunch mavros apm.launch fcu_url:=udp://:14551@
