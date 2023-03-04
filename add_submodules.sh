#!/bin/bash
find . -name ".git" | sed 's/.git$//' | while read repo_path; 
do
  # Navigate to the parent directory of the Git repository
  cd $repo_path
  # Get the URL of the Git repository
  repo_url=$(git config --get remote.origin.url)
  # Add the Git repository as a submodule
  git submodule add $repo_url $repo_path
  # Return to the original directory
  cd -
    #echo $repo_path
done