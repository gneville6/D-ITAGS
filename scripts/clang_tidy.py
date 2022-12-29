import fnmatch
import json
import multiprocessing
import os
import subprocess
import sys
from argparse import ArgumentParser
from functools import partial
from pathlib import Path
from typing import List

# CONSTANTS
DEFAULT_EXTENSIONS = 'c,h,C,H,cpp,hpp,cc,hh,c++,h++,cxx,hxx'
DEFAULT_CLANG_TIDY_IGNORE = '.clang-tidy-ignore'


def printError(program: str, message: str, use_color: bool) -> None:
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


def findCompilationDatabase(path):
    """Adjusts the directory until a compilation database is found."""
    result = './'
    while not os.path.isfile(os.path.join(result, path)):
        if os.path.realpath(result) == '/':
            printError(__file__, 'could not find compilation database.')
            sys.exit(1)
        result += '../'
    return os.path.realpath(result)


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


def getFiles(root_directories: List[str], recursive: bool = False, extensions: List[str] = None,
             excludes: List[str] = None) -> List[str]:
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
    return files


def runClangTidy(file: str,
                 exe: str,
                 build_path: str,
                 includes: List[str] = None,
                 extra_args_before: List[str] = None,
                 extra_args: List[str] = None,
                 use_color: bool = False):
    invocation = [exe, file]
    invocation.append(f'-p={build_path}')
    if extra_args_before:
        for arg in extra_args_before:
            invocation.append(f'-extra-arg-before={arg}')
    if extra_args:
        for arg in extra_args:
            invocation.append(f'-extra-arg={arg}')
    if includes:
        invocation.append('--')
        for include in includes:
            invocation.append(f'-I{include}')

    try:
        proc = subprocess.Popen(
            invocation,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
    except OSError as error:
        printError(__file__, f"Command '{subprocess.list2cmdline(invocation)}' failed to start: {error}",
                   use_color=use_color)
        sys.exit(1)

    proc_stdout = proc.stdout
    proc_stderr = proc.stderr

    outputs = list(proc_stdout.readlines())
    errors = list(proc_stderr.readlines())
    proc.wait()
    if proc.returncode:
        printError(__file__,
                   f"Command '{subprocess.list2cmdline(invocation)}' returned non-zero exit status: {proc.returncode}",
                   use_color=use_color)
        for error in errors:
            print(f'\x1b[1m\x1b[31m{error}\x1b[0m')
        for output in outputs:
            print(output)
        sys.exit(1)

    return outputs, errors


def main():
    parser = ArgumentParser(
        description='A wrapper script for clang-tidy to be used as a stand alone script or for continuous integration')
    parser.add_argument(
        '--exe',
        '--executable',
        help='path to the clang-tidy executable',
        default='clang-tidy')
    parser.add_argument('-p',
                        dest='build_path',
                        help='Path used to read a compile command database.')
    parser.add_argument('-i',
                        '--includes',
                        dest='include_path',
                        default=None,
                        help='Path to the includes folder')
    parser.add_argument(
        '--ext',
        '--extensions',
        metavar='EXT',
        help=f'comma separated list of file extensions (default: {DEFAULT_EXTENSIONS})',
        default=DEFAULT_EXTENSIONS)
    parser.add_argument(
        '--exclude',
        metavar='PATTERN',
        action='append',
        default=[],
        help='exclude paths matching the given glob-like pattern(s) from recursive search')
    parser.add_argument(
        '--col',
        '--color',
        default='auto',
        choices=['auto', 'always', 'never'],
        help='show colored diff (default: auto)')
    parser.add_argument(
        '-j',
        metavar='N',
        type=int,
        default=0,
        help='run N clang-format jobs in parallel (default: number of cpus)')
    parser.add_argument(
        '-r',
        '--recursive',
        action='store_true',
        help='run recursively over directories')
    parser.add_argument('--format',
                        action='store_true',
                        help='Reformat code after applying fixes')
    parser.add_argument(
        '--style',
        default='file',
        help='formatting style to apply (LLVM, Google, Chromium, Mozilla, WebKit, file)')
    # todo: update the help comment v
    parser.add_argument('--checks',
                        default=None,
                        help='checks filter, when not specified, use clang-tidy default')
    parser.add_argument('--fix',
                        action='store_true',
                        help='apply fix-its')
    parser.add_argument('files',
                        metavar='file',
                        nargs='+')
    parser.add_argument('--extra-arg',
                        dest='extra_args',
                        action='append',
                        default=[],
                        help='Additional argument to append to the compiler command line.')
    parser.add_argument('--extra-arg-before',
                        dest='extra_args_before',
                        action='append',
                        default=[],
                        help='Additional argument to prepend to the compiler command line.')

    args = parser.parse_args()

    colored_stdout = False
    colored_stderr = False
    if args.col == 'always':
        colored_stdout = True
        colored_stderr = True
    elif args.col == 'auto':
        colored_stdout = sys.stdout.isatty()
        colored_stderr = sys.stderr.isatty()

    invocation = [args.exe, str("-list-checks")]
    try:
        subprocess.check_call(invocation, stdout=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        printError(parser.prog, str(e), use_colors=colored_stderr)
        return ExitStatus.TROUBLE
    except OSError as e:
        printError(
            parser.prog,
            f"Command '{subprocess.list2cmdline(invocation)}' failed to start: {e}",
            use_colors=colored_stderr,
        )
        return 1

    # Collect excludes
    excludes = excludesFromFile(DEFAULT_CLANG_TIDY_IGNORE)
    excludes.extend(args.exclude)

    extensions = args.ext.split(',')

    if args.build_path is None:
        build_path = findCompilationDatabase('compile_commands.json')
    else:
        build_path = args.build_path

    with open(f'{build_path}/compile_commands.json') as f:
        compile_commands = json.load(f)

    includes = set()
    for entry in compile_commands:
        split_command = entry['command'].split(' ')
        for command_entry in split_command:
            if command_entry.startswith('-I'):
                includes.add(command_entry[2:])
    if args.include_path:
        includes.add(args.include_path)

    # Get max numbers of jobs
    njobs = args.j
    if njobs == 0:
        njobs = multiprocessing.cpu_count()

    files = getFiles(root_directories=args.files,
                     recursive=args.recursive,
                     excludes=excludes,
                     extensions=extensions)

    if not files:
        return 0

    njobs = min(len(files), njobs)
    if njobs == 1:
        it = (runClangTidy(exe=args.exe,
                           file=file,
                           build_path=build_path,
                           includes=includes,
                           extra_args_before=args.extra_args_before,
                           extra_args=args.extra_args,
                           use_color=colored_stderr) for file in files)
        pool = None
    else:
        pool = multiprocessing.Pool(njobs)
        it = pool.imap_unordered(partial(runClangTidy,
                                         exe=args.exe,
                                         build_path=build_path,
                                         includes=includes,
                                         extra_args_before=args.extra_args_before,
                                         extra_args=args.extra_args,
                                         use_color=colored_stderr), files)
        pool.close()

    while True:
        try:
            outputs, errors = next(it)
        except StopIteration:
            break
        except Exception as e:
            if pool:
                pool.terminate()
            raise
        else:
            if outputs == []:
                continue
            for output in outputs:
                if 'warning:' in output:
                    print(f'\x1b[1m\x1b[33m{output}\x1b[0m')
                elif 'error:' in output:
                    print(f'\x1b[1m\x1b[31m{output}\x1b[0m')
                else:
                    print(output)
    if pool:
        pool.join()

    return 0


if __name__ == '__main__':
    sys.exit(main())
