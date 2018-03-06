#! /usr/bin/python3
from __future__ import print_function
import subprocess

if __name__ == '__main__':
  subprocess.call("htop", shell=True)
  print("Done")