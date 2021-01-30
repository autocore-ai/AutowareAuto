#!/usr/bin/env python3
#
# Copyright 2018 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

import argparse
import os
import shutil
import sys

from ament_index_python import get_package_share_directory


def copy(src, dest):
    try:
        shutil.copytree(src, dest)
    except OSError as e:
        print('Directory not copied. Error: %s' % e)


def find_and_replace(filename, pkg_name, email, description=None, maintainer=None):
    username = email.split('@')[0]

    # Read in the file
    with open(filename, 'r') as f:
        filedata = f.read()

    camelcase_pkg_name = ''.join([
      word.capitalize()
      for word in pkg_name.split('_')
    ])

    # Replace the target string
    filedata = filedata.replace('hello_world', pkg_name)
    filedata = filedata.replace('HELLO_WORLD', pkg_name.upper())
    filedata = filedata.replace('HelloWorld', camelcase_pkg_name)
    filedata = filedata.replace('MAINTAINER_EMAIL', email)
    filedata = filedata.replace('USERNAME', username)

    # Fill in package information, if applicable
    if description:
        filedata = filedata.replace('INSERT_DESCRIPTION', description)
    if maintainer:
        filedata = filedata.replace('MAINTAINER_NAME', maintainer)

    # Write the file out again
    with open(filename, 'w') as f:
        f.write(filedata)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination",
                        required=False,
                        default=".",
                        help="Path to new package (without package name)."
                             "e.g., ~/autoware_ws/src")
    parser.add_argument("--pkg-name",
                        required=True,
                        help="Name of the new package")
    parser.add_argument("--maintainer",
                        required=True,
                        help="Name of maintainer (for package.xml)")
    parser.add_argument("--email",
                        required=True,
                        help="Email of the maintainer (for package.xml)")
    parser.add_argument("--description",
                        required=True,
                        help="Description of package (for package.xml)")
    args = parser.parse_args()

    dest = os.path.join(args.destination, args.pkg_name)

    # Copy the template package to the desired location
    template_pkg_dir = os.path.join(
      get_package_share_directory('autoware_auto_create_pkg'), 'template')
    copy(template_pkg_dir, dest)

    # Remove COLCON_IGNORE
    try:
        os.remove(os.path.join(dest, 'COLCON_IGNORE'))
    except FileNotFoundError:
        pass

    # Find and replace all hello_world and HELLO_WORLD with pkg_name and PKG_NAME
    for root, _, files in os.walk(dest):
        for filename in files:
            if filename == 'package.xml':
                find_and_replace(os.path.join(root, filename),
                                 args.pkg_name,
                                 args.email,
                                 description=args.description,
                                 maintainer=args.maintainer)
            else:
                find_and_replace(os.path.join(root, filename), args.pkg_name, args.email)
            if 'hello_world' in filename:
                new_filename = filename.replace('hello_world', args.pkg_name)
                os.rename(os.path.join(root, filename), os.path.join(root, new_filename))

    # Rename directories
    for root, dirs, _ in os.walk(dest):
        for dirname in dirs:
            if 'hello_world' in dirname:
                new_dirname = dirname.replace('hello_world', args.pkg_name)
                os.rename(os.path.join(root, dirname), os.path.join(root, new_dirname))

    print("Package {} has been generated in {}.".format(args.pkg_name, args.destination))

    return 0


if __name__ == "__main__":
    sys.exit(main())
