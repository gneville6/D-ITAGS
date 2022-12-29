import fnmatch
import git
import re
from pathlib import Path
from typing import List

# Local
from .diff_utils import DiffUtils


class GitDiffFormat:
    def __init__(self, staged_only: bool = True):
        self.repo = git.Repo(Path.cwd())

        # Get the difference between the staged files and their versions at the previous commit
        if staged_only:
            self.diff = self.repo.index.diff('HEAD')
        # Get the difference between the current files and their versions at the previous commit
        else:
            self.diff = self.repo.head.commit.tree.diff(None)

    def updateStage(self):
        self.repo.git.add(u=True)

    def getNewFiles(self, excludes: List[str] = None, extensions: List[str] = None):
        '''
            Add the newly create files that have been staged
            :param excludes A list of patterns for the directories/files that should be ignored
            :param extensions A list of the accepted extensions for files that should be formatted
        '''
        if extensions is None:
            extensions = []

        if excludes is None:
            excludes = []

        self.add_files = []
        for da in self.diff.iter_change_type('A'):
            filepath = Path(da.a_path)

            # Ignore the excludes
            should_exclude = False
            for pattern in excludes:
                if fnmatch.fnmatch(filepath, pattern):
                    should_exclude = True
                    break

            if should_exclude:
                # print(f'excluding: {filepath}')
                continue

            # Ignore those with the wrong extension
            ext = filepath.suffix[1:]
            if ext not in extensions:
                # print(f'excluding: {filepath}')
                continue
            # print(filepath)
            self.add_files.append(filepath)
        return self.add_files

    def processModifiedFiles(self, excludes: List[str] = None, extensions: List[str] = None):
        '''
            Collects the changed lines from the modified files
            :param excludes A list of patterns for the directories/files that should be ignored
            :param extensions A list of the accepted extensions for files that should be formatted
        '''
        if extensions is None:
            extensions = []

        if excludes is None:
            excludes = []

        lines_by_file = {}
        for dm in self.diff.iter_change_type('M'):
            filepath = Path(dm.a_path)

            # Ignore the excludes
            should_exclude = False
            for pattern in excludes:
                if fnmatch.fnmatch(filepath, pattern):
                    should_exclude = True
                    break

            if should_exclude:
                # print(f'excluding: {filepath}')
                continue

            # Ignore those with the wrong extension
            ext = filepath.suffix[1:]
            if ext not in extensions:
                # print(f'excluding: {filepath}')
                continue

            # print(filepath)

            # Get the different between the most recent commit and the current staged version of this file
            a_blobtext = dm.a_blob.data_stream.read().decode('utf-8').split('\n')
            b_blobtext = dm.b_blob.data_stream.read().decode('utf-8').split('\n')
            staged_diff = DiffUtils.makeDiff(filepath, a_blobtext, b_blobtext)

            filename = None
            filepath = str(filepath)
            for line in staged_diff:
                # Skip the part with the filename
                match = re.search(r'^\+\+\+\ (.*?/){%s}(\S*)' % 0, line)
                if match:
                    filename = match.group(2)
                if filename is None:
                    continue

                match = re.search(r'^@@.*\+(\d+)(,(\d+))?', line)
                if match:
                    start_line = int(match.group(1))
                    line_count = 1
                    if match.group(3):
                        line_count = int(match.group(3))
                    if line_count == 0:
                        continue
                    end_line = start_line + line_count - 1
                    lines_by_file[filepath] = []
                    lines_by_file[filepath].append(f'{start_line}:{end_line}')
        return lines_by_file
