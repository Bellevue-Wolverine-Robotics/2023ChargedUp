
# How to work in your own branch and push changes for review

## Working on your local machine
- Create your own branch using `git checkout -b <your_branch_name>`
- Working in your branch
     - Switch to your branch using `git checkout <your_branch_name>`
     - Edit the code
     - Make sure you have unit tests and have tested the code
     - Check all changes you made using `git status`; Changes are shown in red
     - Stage all your changes using `git add .` for all files in the current directory or `git add <file_name>` to add one file
     - Check that all changes are staged using `git status`; Staged files are shown in green
     - Commit your changes using `git commit -m "explain what you did here"`
- Push your branch to remote/origin
     - Run `git push`. If it displays a command to push new branch, copy the command and run it.


## Check your branch and create a pull request
- Go to Github and review the changes before submitting a pull request to merge your branch
- After submitting your pull request, ask a teammate to review the changes for you. 
- After you receive approval, merge.


# Reference
FIRST Robotics Competition Control System [WPIlib](https://docs.wpilib.org/en/stable/index.html)
