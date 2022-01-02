#!/bin/sh

rrdtool create softub.rrd \
--step 1 \
--start N \
DS:temp:GAUGE:5m:0:24000 \
DS:set:GAUGE:5m:0:24000 \
DS:running:GAUGE:5m:0:24000 \
DS:validtemp:GAUGE:5m:0:24000 \
DS:cputemp:GAUGE:5m:0:24000 \
RRA:AVERAGE:0.5:1s:10d \
RRA:AVERAGE:0.5:1m:90d \
RRA:AVERAGE:0.5:1h:18M \
RRA:AVERAGE:0.5:1d:10y

# Add a new datasource:
# rrdtool tune softub.rrd DS:validtemp:GAUGE:5m:0:24000
