#!/bin/bash
gdb --args `rospack find ocl`/bin/deployer-gnulinux -linfo -s run.ops
