@ECHO OFF
ECHO Syncing Upstream...
git fetch upstream --quiet
git push origin "refs/remotes/upstream/*:refs/heads/*" --force

REM Get all tags
for /f "delims=" %%a in ('git tag -l') do (
    REM Delete the tag
    git tag -d %%a
)

git fetch upstream --tags --quiet
git push origin --tags --force

PAUSE