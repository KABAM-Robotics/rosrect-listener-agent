# Branching Model

We will follow a simple branching model as shown [here](https://nvie.com/posts/a-successful-git-branching-model/#the-main-branches). Based on this, we will maintain 2 main branches:
- `master`
- `develop`

`master` always refers to the current production-ready state. `develop` refers to the latest development changes that will be part of the next release. This is the main integration branch. Contributors working on bug fixes and feature developments should create their own mini-branches for their development and then issue a pull request to merge their changes to the develop branch. Branch naming convention: 
- If working on a bug fix, use the prefix, `fix/*`, e.g. `fix/listener-cpu-usage`
- If working on a feature, use the prefix, `feature/*`, e.g. `feature/gundam-support`