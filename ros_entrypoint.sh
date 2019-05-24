#!/bin/bash
set -e

# setup ros environment
source "$RICOH_WORKSPACE/devel/setup.bash"
exec "$@"

