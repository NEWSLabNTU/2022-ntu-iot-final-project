#!/usr/bin/env bash
set -e

if ! command -v parallel >/dev/null 2>/dev/null
then
    echo "'parallel' command is not installed" >&2
    echo "If you're using ubuntu, try this:" >&2
    echo "sudo apt install parallel" >&2
    exit 1
fi

if [[ !  -v DISPLAY ]]
then
    echo "DISPLAY environment variable is not set. Find 'N' and try this:" >&2
    echo "export DISPLAY=:N" >&2
    exit 1
fi

port="$1"
shift || {
    echo "Usage: ./$0 PORT" >&2
    exit 1
}


source install/setup.sh
parallel --lb --jobs 0 --halt now,fail=1 <<EOF
ros2 launch launch/final.launch.xml carla_port:=${port}
cd judge && poetry run main --port ${port}
EOF
