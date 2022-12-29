import fnmatch
import multiprocessing
import os
import sys
import traceback
from functools import partial
from pathlib import Path
from typing import List

# Local
from .clang_format_base import ClangFormatBase
from .diff_utils import DiffUtils
from .utilities import ExitStatus, printTrouble, HookException


class WholeFilesFormat(ClangFormatBase):
    '''Executor for running clang-formatting and creating a diff for whole files'''

    def __init__(self, files: List[str]):
        '''
            :param files A list of the filepaths for files to run clang-format on
        '''
        super(WholeFilesFormat, self).__init__()
        self.files = files

    @staticmethod
    def createFromDirectories(root_directories: List[str], recursive: bool = False, extensions: List[str] = None,
                              excludes: List[str] = None) -> 'WholeFilesDiff':
        '''
            :param root_directories The list of directories to to run diffs on
            :param recursive Whether to recursively search through the directories
            :param extensions The list of extensions for files to run diff on
            :param excludes The list of directories/files to exclude
        '''
        if extensions is None:
            extensions = []

        if excludes is None:
            excludes = []

        files = []
        for root in root_directories:
            if recursive and Path(root).is_dir():
                for directory, dnames, fnames in os.walk(root):
                    directory = Path(directory)
                    fpaths = [directory.joinpath(fname) for fname in fnames]

                    # Check if directory or file is 'excluded'
                    for pattern in excludes:
                        # os.walk() supports trimming down the dnames list
                        # by modifying it in-place,
                        # to avoid unnecessary directory listings.
                        dnames[:] = [
                            x for x in dnames
                            if not fnmatch.fnmatch(directory.joinpath(x), pattern)
                        ]
                        fpaths = [
                            x for x in fpaths
                            if not fnmatch.fnmatch(x, pattern)
                        ]
                    # Ignore files with the wrong extensions
                    for f in fpaths:
                        ext = f.suffix[1:]
                        if ext in extensions:
                            files.append(str(f))
            else:
                files.append(root)
        return WholeFilesFormat(files)

    def runClangFormatDiffWrapper(self, file: str, exe: str, in_place: bool = False, style: str = None,
                                  dry_run: bool = False) -> (List[str], List[str]):
        '''
            :brief Runs clang-format on a file and created a unified diff for the formatting updates
            :param exe The executable
            :param file The path to the file
            :param in_place Whether to make the changes to the file in place
            :param style The style for clang-formatting
            :param dry_run Whether to just print the clang-format commands
        '''
        try:
            with open(file) as f:
                original = f.readlines()

            if in_place:
                invocation = [exe, '-i', file]
            else:
                invocation = [exe, file]

            if style:
                invocation.extend(['-style', style])

            if dry_run:
                print(" ".join(invocation))
                return [], []

            outputs, errors = self._runClangFormatDiff(invocation=invocation)
            if in_place:
                return [], errors

            return [line for line in DiffUtils.makeDiff(file, original, outputs)], errors

        except IOError as e:
            raise HookException(str(e))
        except HookException:
            raise
        except Exception as e:
            sys.stderr.write(traceback.format_exc())
            raise Exception(f'{file}: {e.__class__.__name__}: {e}')

    def run(self, njobs: int, exe: str, in_place: bool = False, style: str = None, dry_run: bool = False,
            use_color: bool = False) -> ExitStatus:
        '''
            :param njobs The number of parallel jobs to use for formatting
            :param exe The executable
            :param in_place Whether to make the changes to the file in place
            :param style The style for clang-formatting
            :param dry_run Whether to just print the clang-format commands
        '''
        if not self.files:
            return ExitStatus.SUCCESS
        njobs = min(len(self.files), njobs)

        # Execute one at a time
        if njobs == 1:
            it = (self.runClangFormatDiffWrapper(exe=exe, file=file, in_place=in_place, style=style, dry_run=dry_run)
                  for file in self.files)
            pool = None
        # Execute using a pool
        else:
            pool = multiprocessing.Pool(njobs)
            it = pool.imap_unordered(
                partial(self.runClangFormatDiffWrapper, exe=exe, in_place=in_place, style=style, dry_run=dry_run),
                self.files)
            pool.close()

        while True:
            try:
                outputs, errors = next(it)
            except StopIteration:
                break
            except HookException as e:
                printTrouble(__file__, str(e), use_color=use_color)
                if pool:
                    pool.terminate()
                return ExitStatus.TROUBLE
            except Exception as e:
                printTrouble(__file__, str(e), use_color=use_color)
                if pool:
                    pool.terminate()
                return ExitStatus.TROUBLE
            else:
                if outputs == []:
                    continue
                self.patch_lines.extend(outputs)
                # self.patch_lines.append('\n')
        if pool:
            pool.join()

        return ExitStatus.SUCCESS
