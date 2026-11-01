#!/usr/bin/env bash
# Stop the Fleetra warehouse demo from another terminal.
for NAME in fleetra amazon_warehouse; do
  if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
    docker exec "$NAME" bash -c 'pkill -INT -f multi-robot.py; pkill -INT -f "ros2 launch"' 2>/dev/null
    sleep 2
    docker exec "$NAME" bash -c 'pkill -9 -f "gzserver|gzclient|rviz2|ros2|/opt/"' 2>/dev/null
    docker stop -t 5 "$NAME" >/dev/null 2>&1 && echo "Container $NAME stopped."
  fi
done
xhost -local:docker >/dev/null 2>&1
echo "Fleetra demo stopped."
