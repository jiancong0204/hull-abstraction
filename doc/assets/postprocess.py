#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2020 bormann, IGMR - RWTH Aachen University
# It is ok to add yourself as author, but otherwise please
# DO NOT CHANGE THIS BLOCK; IT IS AUTOMATICALLY MAINTAINED
"""Postprocess doxygen HTML output.
"""

import os
from os import path
import re
import shutil
import tempfile

# Following utility functions taken from the IGMR command line tools.


def replace_patterns_in_file(file_path, patterns, once=True):
    """Replace regular expressions in a file.

    Args:
        file_path: File to act on.
        patterns: Can take several formats:
            - (str, str) replaces first pattern by second string.
            - [(str, str), ...] does the same for several patterns
            - [(line, str, str), (line1, line2, str, str), ...] does the same but only in certain lines
            Patterns are applied in order. You can use substitution references to subgroups like \g<1>.
        once: Only apply one (the first matching) pattern per line.

    Returns:
        The number of substitutions.
    """
    if type(patterns) is tuple:
        patterns = [patterns]
    # Compile patterns
    for (i, pat) in enumerate(patterns):
        if len(pat) == 2:
            patterns[i] = (re.compile(pat[0]), pat[1])
        elif len(pat) == 3:
            patterns[i] = (pat[0], re.compile(pat[1]), pat[2])
        elif len(pat) == 4:
            patterns[i] = (pat[0], pat[1], re.compile(pat[2]), pat[3])

    oldperms = os.stat(file_path).st_mode
    tmpname = tempfile.mktemp(prefix='.tmp', dir='.')
    count = 0
    with open(file_path, 'r') as i:
        with open(tmpname, 'x') as o:
            for (n, line) in enumerate(i.readlines()):
                n += 1
                old = line
                for pat in patterns:
                    if len(pat) == 2:
                        line = re.sub(pat[0], pat[1], line)
                        if once and (line != old or re.search(pat[0], line)):
                            break
                    elif len(pat) == 3 and n == pat[0]:
                        line = re.sub(pat[1], pat[2], line)
                        if once and (line != old or re.search(pat[1], line)):
                            break
                    elif len(pat) == 4 and n >= pat[0] and n <= pat[1]:
                        line = re.sub(pat[2], pat[3], line)
                        if once and (line != old or re.search(pat[2], line)):
                            break
                o.write(line)
                if old != line:
                    count += 1
    os.rename(tmpname, file_path)
    os.chmod(file_path, oldperms)
    return count


def find_files(ending='html', dir='html'):
    filelist = []
    for (dirpath, dirnames, filenames) in os.walk('.'):
        filelist.extend([
            path.join(dirpath, fn)
            for fn in filter(lambda fn: fn.endswith(ending), filenames)
        ])
    return filelist


def patch_html_dead_links(filepath):
    return replace_patterns_in_file(filepath, [
        ('@ref ros_msg', 'not_found.html'),
        ('@ref ros_action', 'not_found.html'),
        ('@ref ros_node', 'not_found.html'),
        ('@ref ros_srv', 'not_found.html'),
    ])


def postprocess_non_found_entities():
    """For references to nodes/actions/etc., replace an invalid link with a nice "Not Found" page."""
    shutil.copy('assets/not_found.html', 'html')
    files = find_files()
    total = len(files)
    for i, f in enumerate(files):
        nicefile = '/'.join(path.split(f)[1:])
        print('Patching file {:d}/{:d} for dead links: {:s}'.format(
            i + 1, total, nicefile))
        patch_html_dead_links(f)


def main():
    if not os.access('Doxyfile', os.F_OK):
        raise Exception('must be run within doxygen directory (doc/)')
    postprocess_non_found_entities()


if __name__ == '__main__':
    main()
