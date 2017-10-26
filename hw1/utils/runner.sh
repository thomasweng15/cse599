#!/bin/sh

WATCH_DIRS=( \
    ~/Documents/CSE599/hw1 \
)

inotifywait -m ${WATCH_DIRS[@]} -e close_write |
    while read path action file; do
        if [[ "$file" =~ .*py$ ]]; then
            if [[ $file == *"DiscreteEnvironment.py"* ]]; then
                echo "$file modified, running script"
                python ../DiscreteEnvironment.py
            fi
        fi
    done