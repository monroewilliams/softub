#!/bin/sh
#
rrdtool="/usr/local/bin/rrdtool"
db="/usr/local/network/misc/softub.rrd"
SITE="/usr/local/network/site/misc"
 
for DURATION in 1h 1d 1w 1m
do

# Height of the graph, in pixels.
GRAPH_HEIGHT=300
# Minimum temperature we will display
MIN_TEMP=50
# Amount of the graph (in degrees) to use for the duty cycle display
# (keep this below 10 to avoid a confusing label in this space)
DUTY_CYCLE_SPACE=9

## Graph for last day
$rrdtool graph ${SITE}/softub_${DURATION}.png \
-w 800 -h ${GRAPH_HEIGHT} -a PNG \
--slope-mode \
--start now-${DURATION} --end now \
--font DEFAULT:9: \
--title "Softub" \
--watermark "`date`" \
--vertical-label "Temp (°F)" \
--lower-limit `expr ${MIN_TEMP} - ${DUTY_CYCLE_SPACE}` --rigid \
--y-grid 10:1 \
--right-axis 1:0 \
DEF:rawTemp=${db}:temp:AVERAGE \
DEF:rawSet=${db}:set:AVERAGE \
DEF:rawValidtemp=${db}:validtemp:AVERAGE \
DEF:running=${db}:running:AVERAGE \
CDEF:minTemp=running,DUP,-,${MIN_TEMP},+ \
CDEF:dutyCycleSpace=running,DUP,-,${DUTY_CYCLE_SPACE},+ \
CDEF:temp=rawTemp,minTemp,INF,LIMIT \
CDEF:set=rawSet,minTemp,INF,LIMIT \
CDEF:validtemp=rawValidtemp,minTemp,INF,LIMIT \
VDEF:tempMax=temp,MAXIMUM \
VDEF:setMax=set,MAXIMUM \
VDEF:validMax=validtemp,MAXIMUM \
CDEF:max=set,POP,tempMax,setMax,validMax,3,SMAX \
CDEF:tempHeight=max,minTemp,- \
CDEF:height=tempHeight,dutyCycleSpace,+ \
CDEF:pixel=height,${GRAPH_HEIGHT},/ \
CDEF:dutyCycleHeight=dutyCycleSpace,pixel,2,*,- \
CDEF:dutyCycleBottom=minTemp,dutyCycleSpace,-,pixel,+ \
CDEF:dutyCycle=running,dutyCycleHeight,*,dutyCycleBottom,+ \
CDEF:stackValidTemp=validtemp,minTemp,- \
LINE3:set#ff7f00be:"Temperature Setting" \
LINE1:temp#00be00:"Measured Temperature" \
LINE1:minTemp#bebebe: \
AREA:stackValidTemp#3f3fff7f:"Water Temperature":STACK \
LINE1:dutyCycle#ff0000:"Pump" \
COMMENT:"\c" \
GPRINT:rawSet:LAST:"Set point\: %.0lf°F" \
GPRINT:rawTemp:LAST:"Current temp\: %5.2lf°F" \
CDEF:dutyCyclePercent=running,100,* \
GPRINT:dutyCyclePercent:AVERAGE:"Duty cycle\: %.0lf%%\c" \

done
