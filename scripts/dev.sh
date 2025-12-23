#!/usr/bin/env bash
# Enter the Icarus development environment or run a command within it
if [ $# -gt 0 ]; then
    nix develop --command "$@"
else
    nix develop
fi
