#!/bin/sh
#
### set the paths
gawk="/usr/local/bin/gawk"
rrdtool="/usr/local/bin/rrdtool"
db="/usr/local/network/misc/softub.rrd"
 

output=`curl --ipv4 --connect-timeout 10 http://softub.local/stats 2>/dev/null`
  
IFS=$'\n' lines=($output)

#for (( i=0; i<${#lines[@]}; i++ ))
#do
#    echo "$i: ${lines[$i]}"
#done

# update the database
if [ ${#lines[@]} -ge 4 ]
then
    $rrdtool update $db --template temp:set:running:validtemp "N:${lines[0]}:${lines[1]}:${lines[2]}:${lines[3]}"
fi

