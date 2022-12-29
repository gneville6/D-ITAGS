import sys
from pathlib import Path
from typing import List


class HookException(Exception):
    '''Custom exception for when we detect an error'''

    def __init__(self, message):
        super(HookException, self).__init__(message)


# todo: is there a better python enum?
class ExitStatus:
    '''Effectively an enum for the exit status of the program'''
    SUCCESS = 0
    DIFF = 1
    TROUBLE = 2


def printTrouble(program: str, message: str, use_color: bool) -> None:
    '''
        :brief Print an error to stderr
        :param program The name of the program
        :param message The error message
        :param use_color Whether the error should be printed in color
    '''
    error_text = 'error:'
    if use_color:
        error_text = '\x1b[1m\x1b[31m' + error_text + '\x1b[0m'
    print(f'{program}: {error_text} {message}', file=sys.stderr)


def excludesFromFile(ignore_file: str) -> List[str]:
    '''
        :param ignore_file The file containing the patterns to ignore
    '''
    excludes = []
    if Path(ignore_file).exists():
        with open(ignore_file) as f:
            for line in f:
                # ignore comments
                if line.startswith('#'):
                    continue

                pattern = line.rstrip()

                # skip empty lines
                if not pattern:
                    continue
                excludes.append(pattern)
    return excludes
