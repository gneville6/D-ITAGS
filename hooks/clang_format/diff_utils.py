import difflib
import sys
from typing import List


class DiffUtils:
    '''Utility functions for diffs'''

    @staticmethod
    def makeDiff(file, original: List[str], reformatted: List[str]) -> List[str]:
        '''
            :brief Makes a unified diff for two versions of the same file
            :param original The text from the original file
            :param reformatted The text from the reformatted file
        '''
        return difflib.unified_diff(
            original,
            reformatted,
            fromfile=f'{file}\t(original)',
            tofile=f'{file}\t(reformatted)',
            n=3
        )

    @staticmethod
    def printDiff(diff_lines: List[str], use_color: bool) -> None:
        '''
            :brief Prints the diff
            :param diff_lines A list of unified diff lines
            :param use_color Whether to print in color
        '''
        if use_color:
            def colorize(lines):
                def bold(s):
                    return '\x1b[1m' + s + '\x1b[0m'

                def cyan(s):
                    return '\x1b[36m' + s + '\x1b[0m'

                def green(s):
                    return '\x1b[32m' + s + '\x1b[0m'

                def red(s):
                    return '\x1b[31m' + s + '\x1b[0m'

                for line in lines:
                    if line[:4] in ['--- ', '+++ ']:
                        yield bold(line)
                    elif line.startswith('@@ '):
                        yield cyan(line)
                    elif line.startswith('+'):
                        yield green(line)
                    elif line.startswith('-'):
                        yield red(line)
                    else:
                        yield line

            diff_lines = colorize(diff_lines)
        sys.stdout.writelines(diff_lines)
