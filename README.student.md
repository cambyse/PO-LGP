MLR Student Repository
========================

This document explains how students get access to the MLR code base.

There are two repositories: one for the MLR group (`mlr_repo`); one for the
students (`student_repo`).


For Students
------------

You should make yourselves familiar with git before you proceed!

This is how you get access to the `mlr_repo`. Your supervisor will give you
access to the gitlab server which host the code.
Login here: https://sully.informatik.uni-stuttgart.de/gitlab/
Add your ssh-key in you profile.

Then clone the repository and create a branch for your work::
```
mkdir git/
cd git
git clone git@sully.informatik.uni-stuttgart.de:mlr_students/mlr.git
cd mlr
git checkout -b your_project_name_or_your_name
```

Note that in the last step you created your own branch for all your work. You
are free to add, edit, commit, and push to that branch as you like.

Don't change anything in the master branch.


For the MLR Group
-----------------

To update files in the `student_repo` you have to have the `student_repo` as
remote:
```
git remote add student_repo git@sully.informatik.uni-stuttgart.de:mlr_students/mlr.git
```

You then can push to the master of the `student_repo`:
```
git checkout master  # switch to the master branch.
git push student_repo master
```

Note: try to only push master to make possible merges easier!
