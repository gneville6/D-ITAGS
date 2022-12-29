import re
import shutil
from typing import List

# Local
from .hunk import Hunk
from .patch import Patch
from .utilities import HookException


class PatchSet:
    '''Contains a set of patches with one for each file to be patched'''

    def __init__(self, patches: List[Patch] = []):
        self.patches = patches

    def append(self, patch: Patch) -> None:
        self.patches.append(patch)

    def isEmpty(self) -> bool:
        return len(self.patches) == 0

    def __patchStream(self, stream, hunks: List[Hunk]) -> List[str]:
        new_lines = []
        source_linenumber = 1

        for hunk_number, hunk in enumerate(hunks):
            # skip to just before hunk
            while source_linenumber < hunk.source_line_start:
                new_lines.append(stream.readline())
                source_linenumber += 1

            for hunk_line in hunk.text:
                # get rid of deleted lines
                if hunk_line.startswith('-'):
                    stream.readline()
                    source_linenumber += 1
                    continue
                else:
                    if not hunk_line.startswith('+'):
                        stream.readline()
                        source_linenumber += 1
                    #  Ignore the '+'
                    new_lines.append(hunk_line[1:])
        # keep any lines after all the hunk
        for line in stream:
            new_lines.append(line)
        return new_lines

    def __writeHunks(self, patch: Patch) -> None:
        with open(patch.source_filename, 'r') as source:
            lines = self.__patchStream(source, patch.hunks)

        with open(patch.target_filename, 'w') as target:
            target.writelines(lines)

        shutil.copymode(patch.source_filename, patch.target_filename)

    def apply(self) -> None:
        for patch in self.patches:
            filename = patch.source_filename
            # test if exists and if not try the target filename

            hunk_number = 0
            hunk = patch.hunks[hunk_number]
            with open(filename) as f:
                # validate
                for linenumber, line in enumerate(f):
                    if linenumber + 1 < hunk.source_line_start:
                        continue
                    if linenumber + 1 == hunk.source_line_start:
                        # grab all the lines that start with '-' or ' '
                        hunk_find = [x[1:].rstrip('\r\n') for x in hunk.text if x[0] in ' -']
                        # grab all the lines that start with '+' or ' '
                        hunk_replace = [x[1:].rstrip('\r\n') for x in hunk.text if x[0] in ' +']

                        hunk_linenumber = 0

                    # Check if the hunk matches the source file
                    if linenumber + 1 < hunk.source_line_start + len(hunk_find) - 1:
                        if line.rstrip('\r\n') == hunk_find[hunk_linenumber]:
                            hunk_linenumber += 1
                        else:
                            raise HookException(f'Hunk does not match source fine {filename}')

                    if linenumber + 1 == hunk.source_line_start + len(hunk_find) - 1:
                        hunk_number += 1
                        if hunk_number < len(patch.hunks):
                            hunk = patch.hunks[hunk_number]
                if hunk_number < len(patch.hunks):
                    raise HookException(f'Premature end of source file {filename}')
            self.__writeHunks(patch)

    @staticmethod
    def parse(stream):
        '''Parses a unified diff'''
        hunk_header_pattern = re.compile(r'^@@ -(\d+)(,(\d+))? \+(\d+)(,(\d+))? @@')
        source_filename_pattern = re.compile(r'^--- (\S*)\t\(original\)')
        target_filename_pattern = re.compile(r'^\+\+\+ (\S+)\t\(reformatted\)')

        class State:
            SCAN_FILENAME = 0
            HUNK_HEADER = 1
            HUNK_BODY = 2
            SCAN_NEXT_STATE = 3

        source_filename = None
        target_filename = None
        actual_source_lines = None
        actual_target_lines = None

        patch_set = PatchSet()
        patch = None
        hunk = None
        parse_state = State.SCAN_FILENAME
        for line_number, line in enumerate(stream):
            # determine if new file or just new hunk
            if parse_state == State.SCAN_NEXT_STATE:
                # new file
                if re.match(source_filename_pattern, line):
                    patch_set.append(patch)
                    patch = None
                    parse_state = State.SCAN_FILENAME
                # new hunk
                elif re.match(hunk_header_pattern, line):
                    parse_state = State.HUNK_HEADER

            if parse_state == State.SCAN_FILENAME:
                if line.startswith('---'):
                    if source_filename is not None:
                        raise HookException(f'False patch with double sources: {source_filename}')
                    match = re.match(source_filename_pattern, line)
                    if match:
                        source_filename = match.group(1).strip()
                    else:
                        raise HookException('Unable to parse source filename from diff')
                elif line.startswith('+++'):
                    if target_filename is not None:
                        raise HookException(f'False patch with double targets: {target_filename}')

                    match = re.match(target_filename_pattern, line)
                    if match:
                        target_filename = match.group(1).strip()
                        patch = Patch()
                        patch.source_filename = source_filename
                        patch.target_filename = target_filename
                        source_filename = None
                        target_filename = None
                    else:
                        raise HookException('Unable to parse target filename from diff')

                    parse_state = State.HUNK_HEADER
            elif parse_state == State.HUNK_HEADER:
                match = re.match(hunk_header_pattern, line)
                if not match:
                    raise HookException(f'Unable to parse hunk header for file {filename}')
                hunk = Hunk()
                hunk.source_line_start = int(match.group(1))
                hunk.source_line_count = int(match.group(3)) if match.group(3) else 1
                hunk.target_line_start = int(match.group(4))
                hunk.target_line_count = int(match.group(6)) if match.group(6) else 1
                actual_source_lines = actual_target_lines = 0
                parse_state = State.HUNK_BODY
            elif parse_state == State.HUNK_BODY:
                if line.startswith('-'):
                    actual_source_lines += 1
                elif line.startswith('+'):
                    actual_target_lines += 1
                else:  # Line in both
                    actual_source_lines += 1
                    actual_target_lines += 1
                hunk.text.append(line)

                if actual_source_lines > hunk.source_line_count or actual_target_lines > hunk.target_line_count:
                    raise HookException(f'extra lines for hunk for target {target_filename}')
                elif actual_source_lines == hunk.source_line_count and actual_target_lines == hunk.target_line_count:
                    patch.hunks.append(hunk)
                    parse_state = State.SCAN_NEXT_STATE
        if patch:
            patch_set.append(patch)

        return patch_set
