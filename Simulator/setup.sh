trap "exit" INT TERM ERR
trap 'kill -SIGINT 0' EXIT

exec python spawn_autonomous_car.py &
exec python spawn_npc.py -n 80 --safe &

wait