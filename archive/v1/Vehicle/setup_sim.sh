trap "exit" INT TERM ERR
trap 'kill -SIGINT 0' EXIT

exec python carla_vehicle_control.py &
exec python carla_video_capture.py &
exec python carla_vehicle_transform.py &

wait