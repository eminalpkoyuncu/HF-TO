#!/usr/bin/env bash
# Heron simulator helper.
#   ./gz.sh build        build the image
#   ./gz.sh up           start the headless sim in the background
#   ./gz.sh gui          start the sim with the Gazebo GUI (X11)
#   ./gz.sh test         drive the boat from inside the container (smoke test)
#   ./gz.sh logs         follow the sim output
#   ./gz.sh shell        bash in the running container (ROS sourced)
#   ./gz.sh restart      restart the sim (resets the boat to the origin)
#   ./gz.sh down         stop and remove the container
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
DC="docker compose"

wait_ready() {
  echo ">> waiting for /odometry/base ..."
  for _ in $(seq 120); do
    if docker exec heron_sim /entrypoint.sh timeout 3 rostopic echo -n 1 /odometry/base >/dev/null 2>&1; then
      echo ">> sim ready (ROS master http://localhost:11311)"
      return 0
    fi
    sleep 1
  done
  echo ">> sim not ready after 120 s; see ./gz.sh logs" >&2
  return 1
}

case "${1:-}" in
  build)   $DC build ;;
  up)      $DC up -d && wait_ready ;;
  gui)
    xhost +local:root >/dev/null
    $DC run -d --rm --name heron_sim -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw heron_sim \
      roslaunch ocean_gazebo heron_headless.launch gui:=true
    wait_ready ;;
  test)    docker exec -it heron_sim /entrypoint.sh rosrun ocean_gazebo smoke_test.py ;;
  logs)    $DC logs -f ;;
  shell)   docker exec -it heron_sim /entrypoint.sh bash ;;
  restart) $DC restart && wait_ready ;;
  down)    $DC down; docker rm -f heron_sim >/dev/null 2>&1 || true ;;
  *)       sed -n '2,10p' "$0"; exit 1 ;;
esac
