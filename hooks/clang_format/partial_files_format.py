import multiprocessing
import traceback
from functools import partial
from typing import List, Mapping, Tuple

# Local
from .clang_format_base import ClangFormatBase
from .diff_utils import DiffUtils
from .utilities import ExitStatus, printTrouble, HookException


class PartialFilesFormat(ClangFormatBase):
    def __init__(self, lines_by_file: Mapping[str, List[str]]):
        super(PartialFilesFormat, self).__init__()
        self.lines_by_file = lines_by_file

    def runClangFormatDiffWrapper(self, file_and_lines: Tuple[str, List[str]], exe: str, style: str = None) -> (
            List[str], List[str]):
        '''
            :brief Runs clang-format on a file and created a unified diff for the formatting updates
            :param exe The executable
            :param file The path to the file
            :param in_place Whether to make the changes to the file in place
            :param style The style for clang-formatting
            :param dry_run Whether to just print the clang-format commands
            :note This function being public is required for running in parallel, but it should never be run directly
        '''
        file, lines = file_and_lines

        try:
            with open(file) as f:
                original = f.readlines()

            invocation = [exe, file]
            for line in lines:
                invocation.extend(['-lines', line])

            if style:
                invocation.extend(['-style', style])

            outputs, errors = self._runClangFormatDiff(invocation=invocation)
            return [line for line in DiffUtils.makeDiff(file, original, outputs)], errors
        except IOError as error:
            raise HookException(str(error))
        except HookException:
            raise
        except Exception as e:
            sys.stderr.write(traceback.format_exc())
            raise Exception(f'{file}: {e.__class__.__name__}: {e}')

    def run(self, njobs: int, exe: str, style: str = None, use_color: bool = False) -> ExitStatus:
        '''
        '''
        if not self.lines_by_file:
            return
        njobs = min(len(self.lines_by_file), njobs)

        # Execute one at a time
        if njobs == 1:
            it = (self.runClangFormatDiffWrapper(exe=exe, file_and_lines=file_and_lines, style=style) for file_and_lines
                  in self.lines_by_file.items())
            pool = None
        # Execute using a pool
        else:
            pool = multiprocessing.Pool(njobs)
            it = pool.imap_unordered(partial(self.runClangFormatDiffWrapper, exe=exe, style=style),
                                     list(self.lines_by_file.items()))
            pool.close()

        while True:
            try:
                outputs, errors = next(it)
            except StopIteration:
                break
            except HookException as e:
                printTrouble(__file__, str(e), use_color=use_color)
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
