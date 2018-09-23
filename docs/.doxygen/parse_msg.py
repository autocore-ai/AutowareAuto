#!/usr/bin/env python3

'''Custom doxygen parser for ROS msg files'''

import os
import sys


def cat(msg_file):
    print("/// \\verbatim")
    with open(msg_file, 'r') as msg:
        for line in msg:
            # Doxygen doesn't like hashes
            print(line.replace('#', '//').rstrip())
    print("\\endverbatim")


if len(sys.argv) != 2 and not sys.argv[1].endwith('.msg'):
    print("Usage: {} <filename>.msg".format(os.path.basename(__file__)))
    sys.exit(1)

# Assuming <path-to-pkg>/<pkg_name>/msg/<msg_file>
msg_file = sys.argv[1]
filename = os.path.basename(msg_file)
pkg_name = os.path.basename(os.path.dirname(os.path.dirname(msg_file)))

print("/// \\file")
print("/// \\brief The {} file in {}".format(filename, pkg_name))
cat(msg_file)
print("/// \\addtogroup autoware-msgs Autoware Message Definitions")
print("/// \\brief A grouping of all .msg files")
print("/// @{")
print("/// \\addtogroup {}-msgs {} message definitions".format(pkg_name, pkg_name))
print("/// \\brief A grouping of .msg files in {}".format(pkg_name))
print("///")
print("/// @{")
print("/// {}/msg/{}".format(pkg_name, filename))
cat(msg_file)
print("/// @}")
print("/// @}")
