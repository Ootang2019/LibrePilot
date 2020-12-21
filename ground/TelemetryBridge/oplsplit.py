#!/usr/bin/python3

import sys
import struct
filename=sys.argv[1]
ofilename=sys.argv[2]
start=float(sys.argv[3])
end=float(sys.argv[4])


fd=open(filename,'rb')
fo=open(ofilename,'wb')

first=None
time=None
count=0
copycount=0
maxn=0
minn=float('inf')
ofirst=None
olast=None
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
    if (time<start or time>end):
        fd.seek(size,1)
    else:
        if (ofirst==None):
            ofirst=time
        olast=time
        oheader=bytearray(header)
        oheader[0:4]=struct.pack('i',(int((time-start)*1000.0)))
        fo.write(oheader)
        fo.write(fd.read(size))
        copycount+=1

last=time
if (count>0):
    avg/=count
print("Input %s has %i packets from %.2fs to %.2fs\nSize Min/Max/Avg: %i/%i/%.2f"%(filename,count,first,last,minn,maxn,avg))
if (copycount>0):
    print("Output %s has %i packets from %.2fs to %.2fs"%(ofilename,copycount, ofirst,olast))
else:
    print("No data between %.2fs and %.2fs! Output file is empty!"%(start,end))





