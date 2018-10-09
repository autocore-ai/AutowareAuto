#!/usr/bin/env python3
#-*- coding: utf-8 -*-
# Copyright 2018  Ternaris.
# SPDX-License-Identifier: Apache-2.0

"""Compiler wrapper for creating .clang_complete"""

import os
import subprocess
import sys

def find_root():
    """Find root of colcon workspace"""
    path = os.path.dirname(os.path.abspath(__file__))
    while path != '/':
        if os.path.exists(os.path.join(path, 'build', 'COLCON_IGNORE')):
            return path
        path = os.path.dirname(path)
    raise RuntimeError('Colcon workspace not found')


def wrapper():
    """Update .clang_complete and forward call"""

    cache = os.path.join(find_root(), ".clang_complete")
    binary = os.path.basename(__file__)

    addargs = set()
    args = iter(sys.argv[1:])
    for arg in args:
        if arg.startswith('-D'):
            addargs.add('-D{}'.format(arg[2:] or next(args)))
        elif arg.startswith('-I'):
            addargs.add('-I{}'.format(arg[2:] or next(args)))
        elif arg == '-isystem':
            addargs.add('-I{}'.format(next(args)))

    try:
        with open(cache, 'r') as fobj:
            lines = {l.strip() for l in fobj.readlines() if l.strip()}
    except IOError:
        lines = set()

    with open(cache, 'w') as fobj:
        fobj.writelines(l + '\n' for l in sorted(lines | addargs))

    call = subprocess.Popen([binary, *sys.argv[1:]])
    sys.exit(call.wait())


if __name__ == '__main__':
    wrapper()
