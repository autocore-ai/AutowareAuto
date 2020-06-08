Git Subtree Tips and Tricks {#git-subtrees}
=============================

# Adding a new subtree with `git subtree`

From the base of the repository, run:

```
git subtree add --prefix path/to/subtree/dir https://github.com/user/repo branch1 --squash -m "[1234] My commit message"
```

- Replace `path/to/subtree/dir` with the subdirectory to where you want the files imported.
- Replace `https://github.com/user/repo` with the repository that you want to import.
- Replace `branch1` with the branch, tag, or commit SHA from the subtree repository that you want to pull.
- Replace `1234` with the issue number that was created to track the import of this repository.

This step generates two commits: a squashed history of the repository, and a merge commit.
- Note that the `-m` option is required to add the correct issue id to the commit message (`[1234]`)

For documentation, add the repository and commit information for the imported repository to `.subtree/subtrees.yaml`.

If you have just added a new subtree or pulled an existing one, **DO NOT USE THE GITLAB REBASE BUTTON** on the merge request.
If you need to rebase the merge request in which you added the subtree, run the following locally:

```
git fetch
git rebase -i origin/master --rebase-merges
```

The rebase to-do file dialogue will be displayed: e.g.
```
label onto

# Branch [1234]-Merge-commit-'211d368547cfe7d48b087cabe4b59545632172e4'-as-'src/external/mpc'
reset [new root]
pick 211d36854 Squashed 'src/external/mpc/' content from commit f35f7423c
label [1234]-Merge-commit-'211d368547cfe7d48b087cabe4b59545632172e4'-as-'src/external/mpc'

reset onto
merge -C fe7cee43a [1234]-Merge-commit-'211d368547cfe7d48b087cabe4b59545632172e4'-as-'src/external/mpc' # [1234] Merge commit '211d368547cfe7d48b087cabe4b59545632172e4' as 'src/external/mpc'
pick 01d97845b [1234] Ignore doxygen
pick 95601e63a [1234] Update subtrees.yaml with mpc 
pick 90d9456dd [1234] Update .subtree/README
```

To rebase successfully, there are a few things that must be taken into account:

1. The second `label` needs to be changed to not have any special characters (such as `[`, `]`, etc.) since the special characters can't be handled by git.
   Replace the long string with something shorter:
```diff
label onto

# Branch [1234]-Merge-commit-'211d368547cfe7d48b087cabe4b59545632172e4'-as-'src/external/mpc'
reset [new root]
pick 211d36854 Squashed 'src/external/mpc/' content from commit f35f7423c
- label [1234]-Merge-commit-'211d368547cfe7d48b087cabe4b59545632172e4'-as-'src/external/mpc'
+ label myshortlabel

reset onto
- merge -C fe7cee43a [1234]-Merge-commit-'211d368547cfe7d48b087cabe4b59545632172e4'-as-'src/external/mpc' # [1234] Merge commit '211d368547cfe7d48b087cabe4b59545632172e4' as 'src/external/mpc'
+ merge -C fe7cee43a myshortlabel
pick 01d97845b [1234] Ignore doxygen on imported joystick_drivers repository
pick 95601e63a [1234] Update subtrees.yaml with joystick_drivers
pick 90d9456dd [1234] Update THIRD_PARTY_SOFTWARE.md
```

2. As in a regular rebase, you can do operations on regular commits (e.g, If you want to reword a commit (to add the issue ID), change `pick` with `reword`):
```diff
label onto

# Branch [1234]-Merge-commit-'211d368547cfe7d48b087cabe4b59545632172e4'-as-'src/external/mpc'
reset [new root]
pick 211d36854 Squashed 'src/external/mpc/' content from commit f35f7423c
label myshortlabel

reset onto
merge -C fe7cee43a myshortlabel
pick 01d97845b [1234] Ignore doxygen on imported joystick_drivers repository
- pick 95601e63a [1234] Update subtrees.yaml with joystick_drivers
+ reword 95601e63a [1234] Update subtrees.yaml with joystick_drivers
pick 90d9456dd [1234] Update THIRD_PARTY_SOFTWARE.md
```

3. To edit the commit message of the merge commit, use the `merge -c` option instead of `merge -C`:
   e.g., to add the issue ID to the commit message:
```diff
label onto

# Branch [1234]-Merge-commit-'211d368547cfe7d48b087cabe4b59545632172e4'-as-'src/external/mpc'
reset [new root]
pick 211d36854 Squashed 'src/external/mpc/' content from commit f35f7423c
label myshortlabel

reset onto
- merge -C fe7cee43a myshortlabel
+ merge -c fe7cee43a myshortlabel
pick 01d97845b [1234] Ignore doxygen on imported joystick_drivers repository
reword 95601e63a [1234] Update subtrees.yaml with joystick_drivers
pick 90d9456dd [1234] Update THIRD_PARTY_SOFTWARE.md
```

Once you have finished preparing the to-do file, save and exit. You will be prompted
for any commit messages that need to change, etc.

# Updating a package that has been `git subtree add`ed before

From the root of the repository, run:

```
git subtree pull --prefix path/to/subtree/dir https://github.com/user/repo 0.9.0 --squash -m "[1234] Updating repo to 0.9.0"
```

- Replace `path/to/subtree/dir` with the subdirectory where the subtree currently exists.
- Replace `https://github.com/user/repo` with the URL of the upstream repository.
- Replace `0.9.0` with the branch, tag, or commit SHA from the subtree repository that you want to pull.
- Replace `1234` with the issue number that was created to track the update of this repository.

This step generates two commits: a squashed history of the repository, and a merge commit.
- Note that the `-m` option is required to add the correct issue ID to the commit message (`[1234]`)

For documentation, make sure to update `.subtree/subtrees.yaml` with the new commit SHA.
