# Guide to Using This GitHUB 

## Setting Up Git (Should already be done before)

1. Create a git account
    - [https://github.com/login](https://github.com/login)
2. Open up your terminal
    - On Mac, you can click Spotlight (the magnifying glass in the top right) and search for Terminal
    - On Windows, you can right click the background and click on Powershell
3. Set up your git variables (make sure to change username and your_email@example.com to your actual ones)
    ```sh
    git config --global user.name <user-name>
    git config --global user.email <your-email@example.com>
    ```
    - Make sure to use quotations.
    - For each command, press enter before proceeding to next line
4. Create SSH keygen
    - This will authentic your account and allow git to trust that it is you making the changes
    ```sh
    ssh-keygen -t rsa -C <your-email@example.com>
    ```
    - If it asks for a password, you can either click enter again to have no password, or you can type a password, but make sure you remember it.
    ```sh
    cat ~/.ssh/id_rsa.pub
    ```
    - Copy starting from "ssh-rsa" to <your-email@example.com>, making sure to include both endpoints
5. Connecting SSH keygen to your github account
    - Go log in to GitHub
    - Click on your profile picture in top right
    - Go to settings >>> SSH and GPG kets
        * Add a title (Personal Laptop, Personal Computer, etc.)
        * Keep it as Authentication Key
        * Paste from step 4 into Key textbox area
    - Then add the SSH key
    ```sh
    ssh -T git@github.com
    ```
    - You should have received the following message:
    > Hi "username"! You've been successfully authenticated, but GitHub does not provide shell access.

## Connecting to Micromouse Repository

1. Create a folder dedicated to the micromouse repository
    ```sh
    ssh -T git@github.com
    ```
2. Make sure to find where the folder is
    - It should be called "Micromouse-2026"
3. When you want to work in that folder, you need to change the directory within the terminal to it.
    ```sh
    cd <file-path>
    ```

## Making Changes to Bot's Remote Repository

### (Part 1: Before making edits to the files)

1. Switch to correct micromouse branch
    ```sh
    git checkout <bot-name>
    ```
    If this has an error (invalid path), then you must do this before redoing this step:
    ```sh
    git config --global core.protectNTFS false
    ```
2. Create a branch to work with
    ```sh
    git branch user/<user-name>/<bot-name>/<problem>
    ```
3. Switch to the new branch
    ```sh
    git checkout user/<user-name>/<bot-name>/<problem>
    ```

### (Part 2: Sending Changes from Working Directory to Remote Repository)

1. Sending changes from Working Directory to Staging Area
    ```sh
    git add .
    ```
    - This adds all files
2. Sending changes from Staging Area to Local Directory
    ```sh
    git commit
    ```
    - This commits everything ins taging area with a message. Messages should be detailed. If it is one sentence message, you can use `-m "message"`.
    - WARNING: To actually get this accepted by the Micromouse Repository, a log number (ex. #45) needs to be included. This log number should reference an [issue](https://github.com/RutgersMicromouse/Micromouse-2026/issues).
3. Commit VIM
    * A VIM would pop up and is different from the regular terminal commands. Anything in hashtags is not part of the VIM. Use the empty space at the top to write the message that you want. Hit enter if you want to write on a new line (recommended to make the commit more detailed). Look at the VIM guide at the bottom for ways to exit VIM.
4. Sending changes from Local Directory to Remote Repository
    ```sh
    git push
    ```
    - This pushes all commits to the remote repository
    - WARNING: To actually get this accepted by the Micromouse Repository, only one commit can be done to the branch. There are ways to combine multiple commits into one.

### (Part 3: Merge the Branch Back Into The Bot)

1. Go To Pull Requests in the Micromouse Repository

2. Click New Pull Request

3. Check to make sure the merge is setup correctly, then create pull request
    - Base should be the bot branch
    - Compare should be the issue branch that you made edits to it

4. Now you must get two people to look over it

### (Part 4: Approve The Merge)

1. Two people need to look at the changes
    - Go to the Pull Request >>> Files Changed
    - Make sure everything is correct
    - Click "Submit Review"
    - Add a comment if needed, and if it is good, then check "Approve" and hit "Submit Review"

2.  There's two main errors you may receive
    - check-commit-message
        * This means that the commit message doesn't include the log number. The commit message should change to include it.
    - check-single-commit
        * This means there is multiple commits. This should be reduced to one.

3. To help resolve the commit errors, you can run this command to open up VIM (Vi Improved)
    ```sh
    git rebase -i HEAD~2
    ```
    * Head~2: This squashes the commit to the parent commit. Effectively making it one commit.
    * -i: Opens up the VIM which can allow you to edit commit message and make it multiline and more clear.
    ```sh
    git push -f
    ```
    * This forces the remote repository to accept these changes and rewrite any history in that branch.

## Useful Git Commands

1. To view commit history
    ```sh
    git log
    ```
2. To view what branch you are on
    ```sh
    git branch
    ```

## Useful VIM Information

1. To enter insert mode to edit the text, press i and start pressing. Anything in hashtags is not considered. 
2. If you are in insert mode, you have to press the escape key to then run the colon commands.
3. To exit (make sure you hit enter after these commands)
    ```sh
    :q
    ```
    This quits (q) the VIM.
    ```sh
    :qa!
    ```
    This quits (!) all (a) of it and forces (!) it even if there are unsaved changes.
    ```sh
    :wq
    ```
    This writes (w) the changes aka saving it and then it quits (q)


