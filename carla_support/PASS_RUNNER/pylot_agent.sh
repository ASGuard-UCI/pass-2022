export PYLOT_HOME=/home/zhisheng/new_disk/Documents/pylot
export CARLA_HOME=/home/zhisheng/new_disk/Documents/carla
export LEADERBOARD_HOME=/home/zhisheng/new_disk/Documents/xlab_projects/PASS_RUNNER/leaderboard/
export SCENARIO_RUNNER_ROOT=$(pwd)
export PYTHONPATH=$SCENARIO_RUNNER_ROOT:$PYLOT_HOME:$PYLOT_HOME/dependencies:$CARLA_HOME:$CARLA_HOME/PythonAPI/carla:$CARLA_HOME/PythonAPI/carla/dist/carla-0.9.13-py3.6-linux-x86_64.egg:$LEADERBOARD_HOME

python3 my_leaderboard_control.py --scenarios=OfflineChallenge_1 --agent=leaderboard/leaderboard/autoagents/pylot_agents/ERDOSAgent.py --routes=srunner/data/my_routes_test.xml --agent-config=leaderboard/leaderboard/autoagents/pylot_agents/test.conf
