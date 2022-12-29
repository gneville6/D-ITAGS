import subprocess
from typing import List

# Local
from .diff_utils import DiffUtils
from .utilities import HookException


class ClangFormatBase:
    def __init__(self):
        self.patch_lines = []

    def _runClangFormatDiff(self, invocation) -> (List[str], List[str]):
        '''
            :brief Runs the provided clang-format invocation and returns the results
        '''

        # todo: change to subprocess.run?
        try:
            proc = subprocess.Popen(
                invocation,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
        except OSError as error:
            raise HookException(f"Command '{subprocess.list2cmdline(invocation)}' failed to start: {error}")

        proc_stdout = proc.stdout
        proc_stderr = proc.stderr

        outputs = list(proc_stdout.readlines())
        errors = list(proc_stderr.readlines())
        proc.wait()
        if proc.returncode:
            print(errors)
            raise HookException(
                f"Command '{subprocess.list2cmdline(invocation)}' returned non-zero exit status: {proc.returncode}")

        return outputs, errors

    def print(self, use_color: bool) -> None:
        DiffUtils.printDiff(self.patch_lines, use_color=use_color)

    def canPatch(self) -> bool:
        return len(self.patch_lines) > 0

    def getPatchLines(self) -> List[str]:
        return self.patch_lines
