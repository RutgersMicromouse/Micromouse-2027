# Micromouse-2027

## introduction

This is the official repository for micromouse bots. 

- They're all on separate branches. Branches are named as `release/<bot name>`
    - for example, `release/cheesieee`

## Branching Conventions

For whatever issue you're working on, you'll create a branch off of the bot you're working. For example, if you're working on ratatouieee, you'll create a branch off `release/ratatouieee`.

The naming convention will be as follows:

```sh
user/<user-name>/<bot-name>/problem

# example, afe123x is starting the implementation of pid rotation 
user/arfelix/ratatouieee/pid-rotation-implementation
```


## Making a branch

lets say I'm working on ratatouieee

```sh
git checkout ratatouieee # checkout the ratatouieee branch
git branch user/arfelix/ratotouieee/pid-rotation-implementation # create a branch off ratatouieee named user/arfelix/ratotouieee/pid-rotation-implementation
git checkout user/arfelix/ratotouieee/pid-rotation-implementation
```

now you can start working on the code.

## Making a PR

- Once you create your changes and tested it, you can make a **pull request**. you can see the instructions [here](https://github.com/RutgersMicromouse/git-workshop)
