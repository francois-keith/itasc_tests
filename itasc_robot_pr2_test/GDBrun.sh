#!/bin/bash
gdb --args `rospack find ocl`/bin/deployer-gnulinux -lwarning -s run.ops
