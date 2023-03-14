


# How to work in your own branch and push changes for review

## Working in your local machine
- create you own branch  `git checkout -b <your_branch_name>`
- working in your own branch
     - switch to your branch `git checkout <your_branch_name>
     - add your changes
     - check all changes you made by `git status`; they are shown in red
     - stage all your changes:  `git add .` for all files under current directory or  `git add <file_name>`
     - check that all changes are staged: they are green
     - commit your changes: `git commit -m "explain what you did here"`
     - make sure you have unit test and  tested code
- push your branch to remote/origin
     - run `git push` and it might display a command to push new branch. Copy the command and run it.

  
## Check your branch and prepare for pull request
- Got to github and review the changes before submit pull requst
- After submit your pull request, asking a teammate to review the changes for you. 
- After you receive an approval,  merge. 



# Reference
FIRST Robotics Competition Control System [WPIlib](https://docs.wpilib.org/en/stable/index.html)
