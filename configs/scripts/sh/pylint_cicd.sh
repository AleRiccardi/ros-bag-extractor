#!/bin/bash

PYLINT_MIN=8
rm -rf .pylint && mkdir -p .pylint/

# -----------------------------------------------------------------------------
# Analysis

SOURCE_PATH=tomeye/
PYLINT_REPORT_FILE=.pylint/report.txt
touch $PYLINT_REPORT_FILE

echo "Computing Pylint score ..."
PYLINT_OUTPUT=$(poetry run pylint --exit-zero --output $PYLINT_REPORT_FILE $(find $SOURCE_PATH -type f -name "*.py"))
cat $PYLINT_REPORT_FILE

PYLINT_SCORE=$(cat $PYLINT_REPORT_FILE | sed -n 's/^Your code has been rated at \([-0-9.]*\)\/.*/\1/p')

# -----------------------------------------------------------------------------
# Anybadge

poetry run anybadge --label=Pylint --file=.pylint/pylint.svg --value=$PYLINT_SCORE 2=red 4=orange 8=yellow 10=green

# -----------------------------------------------------------------------------
# Score check

echo "Pylint score is $PYLINT_SCORE (min. $PYLINT_MIN)"

if [[ $(echo "$PYLINT_SCORE>$PYLINT_MIN" | bc -l) == 1 ]]; then
    exit 0
else
    echo "Error: low score"
    exit 1
fi
