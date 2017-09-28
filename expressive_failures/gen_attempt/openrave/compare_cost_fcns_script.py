#!/usr/bin/env python

import h5py, os, subprocess, time
from compare_cost_fcns import LOGFILE, writelog

CURRDIR = os.path.dirname(__file__)
PYSCRIPT = os.path.join(CURRDIR, "compare_cost_fcns.py")

TASK_NAMES = ['push', 'lift']

def main():
    writelog("Starting")
    h5_fname = "output_20170927_220543_411510.h5"

    for task_name in TASK_NAMES:
        print "Task:", task_name
        while True:
            f = h5py.File(h5_fname, 'r')
            if "done" in f and task_name in f["done"]:
                print "\tdone"
                break
            f.close()

            pout = subprocess.Popen("python " + PYSCRIPT + " " + task_name, shell=True, \
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = pout.communicate()
            writelog("Restarting")

if __name__ == "__main__":
    main()
