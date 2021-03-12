What is this project ?
----------------------

This project is **NOT** the official VTK repository.

It is a fork of VTK sources hosted at https://github.com/Kitware/VTK.

The official upstream repository is https://gitlab.kitware.com/vtk/vtk.

It is used as staging area to maintain and test patches that will be contributed back to the
official repository.


What is the branch naming convention ?
--------------------------------------

Each branch is named following the pattern `slicer-vY.Y.Z-YYYY-MM-DD-SHA{7}`

where:

* `vX.Y.Z` is the version of the forked project
* `YYYY-MM-DD` is the date of the last official commit associated with the branch.
* `SHA{9}` are the first seven characters of the last official commit associated with the branch.

For more details, see https://www.slicer.org/wiki/Documentation/Nightly/Developers/ProjectForks


How to backport changes to a specific branch ?
----------------------------------------------

1. Fork `Slicer/VTK`

2. Checkout the relevant `slicer-vX.Y.Z-YYYY-MM-DD-SHA{9}` branch. See [SuperBuild/External_VTK.cmake](https://github.com/Slicer/Slicer/blob/master/SuperBuild/External_VTK.cmake) to identify the name of the branch.

3. Add the remote from which to cherry-pick commit from

4. Cherry-pick the commit(s)

5. Amend the commit updating title and message to include `[backport MR-1234]` and `Cherry-picked commit ...` information.

    ```
    [Backport MR-7480] Simplify coincident topology resolution
        
    See https://gitlab.kitware.com/vtk/vtk/-/merge_requests/7480/diffs?commit_id=03e2ac53aba58a0a060d77e12d4893230c146e1b

    ...
    ```

    Or if the change has already been integrated into the upstream repository:

    ```
    [backport] Fix infinite loop in vtkContourTriangulator 

    Cherry-picked commit a93fff6 from main VTK repository.

    ...
    ```

6. Finally, create a pull request against the relevant `slicer-vX.Y.Z-YYYY-MM-DD-SHA{9}` branch.


_Note: If you have push access to the `Slicer/VTK` fork, you could directly push commit to the relevant branch_


How to update the version of VTK ?
----------------------------------

1. Clone this repository and add a remote to the official project

```
git clone git://github.com/Slicer/VTK
cd VTK
git remote add upstream git://github.com/kitware/VTK
git fetch upstream
```

2. Create a new branch following the convention

```
# Extract version from https://github.com/Kitware/VTK/blob/master/CMake/vtkVersion.cmake
XYZ=$(cat CMake/vtkVersion.cmake | grep VERSION | sed -re "s/set\(VTK_[A-Z]+_VERSION (.+)\)/\\1/" | perl -pe 'chomp if eof' | tr '\n' '.')
echo "XYZ [${XYZ}]"

DATE=$(git show -s --format=%ci upstream/master | cut -d" " -f1)
echo "DATE [${DATE}]"

vtkSHA=$(git show -s --format=%h upstream/master)
echo "vtkSHA: [${SHA}]"

BRANCH_NAME=slicer-v${XYZ}-${DATE}-${vtkSHA}
echo "BRANCH_NAME [${BRANCH_NAME}]"

git checkout -b ${BRANCH_NAME} ${vtkSHA}
```

3. Cherry-pick the Slicer specific commits from last branch. Resolve conflict as needed.

   - the commit hash included in each branch name is the VTK *upstream* hash, so
     Slicer specific commits may be viewed with the git range (`..`) operation:
```          
          git log --pretty=format:"%h" {vtkSHA}..slicer-xyz-{vtkSHA}
```

4. To **test the changes**, locally rebuild VTK, CTK and Slicer.

5. Publish the branch. (directly in this repo if you have push rights, or on a fork)

6. Update Slicer VTK external project and submit a pull request.


How to be granted push rights ?
-------------------------------

Ask on https://discourse.slicer.org/


Questions
---------

If you have questions, see https://discourse.slicer.org/
