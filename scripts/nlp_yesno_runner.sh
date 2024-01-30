#!/bin/bash
python3 $1 $2

python_status=$?

current_date=`date "+%D %T"`

msg="A script has ran to error:  

Server: $HOSTNAME

Script Name: $1 

Time: $current_date"

echo "$python_status"
echo "$msg"
