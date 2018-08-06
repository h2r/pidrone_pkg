#!/bin/bash

# credit goes to @pete-otaqui for initial gist:
# https://gist.github.com/pete-otaqui/4188238

# original version modified for pidrone_pkg releases

# works with a file called VERSION in the current directory,
# the contents of which should be a semantic version number
# such as "1.2.3"

# this script will display the current version, automatically
# suggest a "minor" version update, and ask for input to use
# the suggestion, or a newly entered value.

# once the new version number is determined, the script will
# pull a list of changes from git history, prepend this to
# a file called CHANGES (under the title of the new version
# number) and create a GIT tag.

if [ -f VERSION ]; then
    BASE_STRING=`cat VERSION`
    BASE_LIST=(`echo $BASE_STRING | tr '.' ' '`)
    V_MAJOR=${BASE_LIST[0]}
    V_MINOR=${BASE_LIST[1]}
    echo "Current version : $BASE_STRING"
    if [ $V_MINOR -eq 5 ]; then
        V_MINOR=0
    fi
    V_MAJOR=$((V_MAJOR + 1))
    SUGGESTED_VERSION="$V_MAJOR.$V_MINOR"
    read -p "Enter a version number [$SUGGESTED_VERSION]: " INPUT_STRING
    if [ "$INPUT_STRING" = "" ]; then
        INPUT_STRING=$SUGGESTED_VERSION
    fi
    echo "Will set new version to be $INPUT_STRING"
    echo $INPUT_STRING > VERSION
    git add VERSION
    git commit -m "Version bump to $INPUT_STRING"
    git tag -a -m "Tagging version $INPUT_STRING" "v$INPUT_STRING" 
    git push
    git push origin master:develop
    git push origin --tags
else
    echo "Could not find a VERSION file"
fi
