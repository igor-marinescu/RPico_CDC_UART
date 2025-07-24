#!/usr/bin/bash

start=$(date +%s)
echo "Start time:$start"

# run stuff, capture the return code
cmake ../src/
make

end=$(date +%s)
echo "End time:$end"
echo "Delta time:" $(( $(date +%s) - $start ))