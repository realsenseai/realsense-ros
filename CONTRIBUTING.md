# How to Contribute

This project welcomes third-party code via GitHub pull requests.

You are welcome to propose and discuss enhancements using project [issues](https://github.com/IntelRealSense/realsense-ros/issues).

> **Branching Policy**:
> The `ros2-master` branch is considered stable, at all times.
> If you plan to propose a patch, please commit into the `ros2-development` branch, or its own feature branch.

In addition, please run `pr_check.sh` under `scripts` directory. This scripts verify compliance with project's standards:

1. Every example / source file must refer to [LICENSE](https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/LICENSE)
2. Every example / source file must include correct copyright notice
3. For indentation we are using spaces and not tabs
4. Line-endings must be Unix and not DOS style

Most common issues can be automatically resolved by running `./pr_check.sh --fix`

Please familirize yourself with the [Apache License 2.0](https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/LICENSE) before contributing.

## Step-by-Step

1. Make sure you have `git` and `cmake` installed on your system. On Windows we recommend using [Git Extensions](https://github.com/gitextensions/gitextensions/releases) for git bash.
2. Run `git clone https://github.com/IntelRealSense/realsense-ros.git` and `cd realsense-ros`
3. To align with latest status of the ros2-development branch, run:
```
git fetch origin
git checkout ros2-development
git reset --hard origin/ros2-development
```
4. `git checkout -b name_of_your_contribution` to create a dedicated branch
5. Make your changes to the local repository
6. Make sure your local git user is updated, or run `git config --global user.email "email@example.com"` and `git config --global user.user "user"` to set it up. This is the user & email that will appear in GitHub history.
7. `git add -p` to select the changes you wish to add
8. `git commit -m "Description of the change"`
9. Make sure you have a GitHub user and [fork realsense-ros](https://github.com/IntelRealSense/realsense-ros#fork-destination-box)
10. `git remote add fork https://github.com/username/realsense-ros.git` with your GitHub `username`
11. `git fetch fork`
12. `git push fork` to push `name_of_your_contribution` branch to your fork
13. Go to your fork on GitHub at `https://github.com/username/realsense-ros`
14. Click the `New pull request` button
15. For `base` combo-box select `ros2-development`, since you want to submit a PR to that branch
16. For `compare` combo-box select `name_of_your_contribution` with your commit
17. Review your changes and click `Create pull request`
18. Wait for all automated checks to pass
19. The PR will be approved / rejected after review from the team and the community

To continue to new change, goto step 3.
To return to your PR (in order to make more changes):
1. `git stash`
2. `git checkout name_of_your_contribution`
3. Repeat items 5-8 from the previous list
4. `git push fork`
The pull request will be automatically updated

