#!/bin/sh

if test $# -ne 1; then
    echo "Usage: $0 boost_path"
    exit 1
fi

BOOST_PATH=$1
for f in `cat boost.files`; do echo cp "$BOOST_PATH/boost/$f boost/$f; done
# FIXME: use while read to handle spaces.
