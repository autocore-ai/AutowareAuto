import sys
import os
import subprocess

# hack for readthedocs to cause it to run doxygen first
# https://github.com/rtfd/readthedocs.org/issues/388
on_rtd = os.environ.get('READTHEDOCS', None) == 'True'
if on_rtd:
    subprocess.call('.doxygen/build.py', shell=True)

master_doc = 'placeholder'
