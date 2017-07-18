import time

timestr = time.strftime("%m%d-%H%M")

f = open('/mnt/hgfs/Virtual Machines/data/'+timestr+'.txt', 'w')
print >> f, 'Dot product, r_forearm_link:'  # or f.write('...\n')
f.close()