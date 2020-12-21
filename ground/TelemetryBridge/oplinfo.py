#!/usr/bin/python3

import sys
import struct
filename=sys.argv[1]

fd=open(filename,'rb')

first=None
time=None
count=0
maxn=0
minn=float('inf')
avg=0.0
while (True):
    header=fd.read(12)
    if (len(header)<12):
            break
    time=(struct.unpack('i',header[0:4])[0])/1000.0
    size=struct.unpack('q',header[4:12])[0]
    count+=1
    if (first==None):
        first=time
    if (size>maxn):
        maxn=size
    if (size<minn):
        minn=size
    avg+=size
    fd.seek(size,1)
last=time
if (count>0):
    avg/=count
print("%s has %i packets from %.2fs to %.2fs\nSize Min/Max/Avg: %i/%i/%.2f"%(filename,count,first,last,minn,maxn,avg))




