#!/usr/bin/env python3
import argparse
import multiprocessing
import subprocess
import sys

from .git_diff_format import GitDiffFormat
from .partial_files_format import PartialFilesFormat
from .patch_set import PatchSet
from .utilities import ExitStatus, HookException, printTrouble, excludesFromFile
from .whole_files_format import WholeFilesFormat

# CONSTANTS
DEFAULT_EXTENSIONS = 'c,h,C,H,cpp,hpp,cc,hh,c++,h++,cxx,hxx'
DEFAULT_CLANG_FORMAT_IGNORE = '.clang-format-ignore'


def main():
    # Top level parser
    parser = argparse.ArgumentParser(
        description='A wrapper script for clang-format to be used as a stand alone script, for continuous integration, or for a pre-commit hook')
    parser.add_argument(
        '--exe',
        '--executable',
        help='path to the clang-format executable',
        default='clang-format')
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
        '--style',
        help='formatting style to apply (LLVM, Google, Chromium, Mozilla, WebKit, file)')
    subparsers = parser.add_subparsers(dest='subparser')

    # Whole Files
    subsubparser_wholefiles = subparsers.add_parser('wholefiles', help='run on entire files')
    subsubparser_wholefiles.add_argument(
        '-r',
        '--recursive',
        action='store_true',
        help='run recursively over directories')
    subsubparser_wholefiles.add_argument(
        '-i',
        '--in-place',
        action='store_true',
        help='format file instead of printing differences')
    parser.add_argument(
        '-q',
        '--quiet',
        action='store_true',
        help="disable output, useful for the exit code")

    subsubparser_wholefiles.add_argument(
        '-d',
        '--dry-run',
        action='store_true',
        help='just print the list of files')
    subsubparser_wholefiles.add_argument('files', metavar='file', nargs='+')

    # Patch or Git Hook
    subsubparser_patch = subparsers.add_parser('patch', help='patch the staged changes in a git repo')
    subparser_hook = subparsers.add_parser('githook', help='run this as a git pre-commit hook')

    args = parser.parse_args()

    colored_stdout = False
    colored_stderr = False
    if args.col == 'always':
        colored_stdout = True
        colored_stderr = True
    elif args.col == 'auto':
        colored_stdout = sys.stdout.isatty()
        colored_stderr = sys.stderr.isatty()

    # TODO: maybe move to utilities?
    # Check if clang-format can be found
    version_invocation = [args.exe, str("--version")]
    try:
        subprocess.check_call(version_invocation, stdout=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        printTrouble(parser.prog, str(e), use_colors=colored_stderr)
        return ExitStatus.TROUBLE
    except OSError as e:
        printTrouble(
            parser.prog,
            f"Command '{subprocess.list2cmdline(version_invocation)}' failed to start: {e}",
            use_colors=colored_stderr,
        )
        return ExitStatus.TROUBLE

    # Collect excludes
    excludes = excludesFromFile(DEFAULT_CLANG_FORMAT_IGNORE)
    excludes.extend(args.exclude)

    extensions = args.ext.split(',')

    # Get max numbers of jobs
    njobs = args.j
    if njobs == 0:
        njobs = multiprocessing.cpu_count()

    if args.subparser == 'wholefiles':
        whole_files = WholeFilesFormat.createFromDirectories(
            root_directories=args.files,
            recursive=args.recursive,
            excludes=excludes,
            extensions=extensions)
        status = whole_files.run(njobs=njobs,
                                 exe=args.exe,
                                 in_place=args.in_place,
                                 style=args.style,
                                 dry_run=args.dry_run,
                                 use_color=colored_stderr)
        if status != ExitStatus.SUCCESS:
            return status

        if not args.quiet:
            whole_files.print(use_color=colored_stdout)

        return status


    # Patch or git hook
    else:
        git_diff = GitDiffFormat(staged_only=True if args.subparser == 'githook' else False)

        # Run clang-format on the entire file for new files
        add_files = git_diff.getNewFiles(excludes=excludes,
                                         extensions=extensions)
        add_files_diff = WholeFilesFormat(files=add_files)
        add_files_diff.run(njobs=njobs,
                           exe=args.exe,
                           in_place=False,
                           style=args.style,
                           dry_run=False,
                           use_color=colored_stderr)
        add_files_diff.print(use_color=colored_stdout)

        # Run clang-format on the new lines for modified files
        modified_lines_by_file = git_diff.processModifiedFiles(excludes=excludes,
                                                               extensions=extensions)
        partial_files_diff = PartialFilesFormat(lines_by_file=modified_lines_by_file)
        partial_files_diff.run(njobs=njobs,
                               exe=args.exe,
                               style=args.style,
                               use_color=colored_stderr)
        partial_files_diff.print(use_color=colored_stdout)

        if add_files_diff.canPatch() or partial_files_diff.canPatch():
            if args.subparser == 'githook':

                print('''
The staged content is not formatted correctly.
The patch shown above can be applied automatically to fix the formatting.
You can:
[a]: Apply the patch and continue the commit
[i]: Ignore the patch and commit anyway (NOT RECOMMENDED!)
[c]: Cancel the commit
What would you like to do? [a/i/c]
''')
                # Replace stdin with /dev/tty
                sys.stdin = open('/dev/tty')
                while True:
                    response = input()
                    response = response.lower()
                    if response not in ['a', 'i', 'c']:
                        print('''Unknown response. Options are [a/i/c]''')
                        continue
                    if response == 'a':
                        patch_set = PatchSet.parse(add_files_diff.getPatchLines())
                        patch_set.apply()
                        patch_set = PatchSet.parse(partial_files_diff.getPatchLines())
                        patch_set.apply()
                        git_diff.updateStage()
                        return ExitStatus.SUCCESS
                    elif response == 'i':
                        return ExitStatus.SUCCESS
                    else:  # c
                        return ExitStatus.TROUBLE


            else:  # 'patch'
                print('''
The modified content is not formatted correctly.
The patch shown above can be applied automatically to fix the formatting.
You can:
[a]: Apply the patch
[i]: Ignore the patch
What would you like to do? [a/i]
''')

                while True:
                    response = input()
                    response = response.lower()
                    if response not in ['a', 'i']:
                        print('''Unknown response. Options are [a/i]''')
                        continue

                    if response == 'a':
                        patch_set = PatchSet.parse(add_files_diff.getPatchLines())
                        patch_set.apply()
                        patch_set = PatchSet.parse(partial_files_diff.getPatchLines())
                        patch_set.apply()
                        return ExitStatus.SUCCESS
                    else:  # i
                        return ExitStatus.SUCCESS

        # with tempfile.NamedTemporaryFile()as patchfile:
        #    patchfile.writelines(patch_lines)
        #    repo.git.execute(['git', 'apply', '-p0', '--cached', patchfile.name])

    return ExitStatus.SUCCESS


if __name__ == '__main__':
    sys.exit(main())
