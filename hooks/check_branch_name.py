from git import Repo
from pathlib import Path


def main():
    repo = Repo(Path.cwd())
    branch = repo.active_branch

    if (branch.startswith('feature/') or
            branch.startswith('release/') or
            branch.startswith('hotfix/') or
            branch == 'master' or
            branch == 'dev'):
        return

    print("\033[91mCurrent branch name does not match the standard. Options are\n" +
          "\tmaster\tThe main stable branch (Should never be modified directly)\n" +
          "\tdev\tThe development stable branch (Should never be modified directly)\n"
          "\tfeature/*\tA branch for creating a new feature\n" +
          "\trelease/*\tA branch for changes for a release (read paper)\n" +
          "\thotfix/*\tA branch for directly fixing an issue in either master or dev\033[0m")
    exit(1)


if __name__ == '__main__':
    main()
