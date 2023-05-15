# ubuntu
#export CARLA_HOME=/home/zhisheng/new_disk/Documents/carla
#export LEADERBOARD_HOME=/home/zhisheng/new_disk/Documents/xlab_projects/PASS_RUNNER/leaderboard/
#export SCENARIO_RUNNER_ROOT=$(pwd)
#export PYTHONPATH=$SCENARIO_RUNNER_ROOT:$CARLA_HOME:$CARLA_HOME/PythonAPI/carla:$CARLA_HOME/PythonAPI/carla/dist/carla-0.9.13-py3.6-linux-x86_64.egg:$LEADERBOARD_HOME
#
#python3 my_leaderboard_control.py --scenarios=OfflineChallenge_1 --agent=leaderboard/leaderboard/autoagents/detour_agents/my_detour_agent.py --routes=srunner/data/my_routes_test.xml


# windows

CARLA_HOME=/d/hzsxi/Documents/2022/CARLA_0.9.13/WindowsNoEditor
LEADERBOARD_HOME=/d/hzsxi/Documents/2022/PASS_RUNNER/leaderboard/
SCENARIO_RUNNER_ROOT=$(pwd)

echo "Starting the game"
start $CARLA_HOME/CarlaUE4.exe


PYTHONPATH=$SCENARIO_RUNNER_ROOT:$CARLA_HOME:$CARLA_HOME/PythonAPI/carla/:$CARLA_HOME/PythonAPI/carla/dist/carla-0.9.13-py3.7-win-amd64.egg:$LEADERBOARD_HOME python my_leaderboard_multi_agents.py --scenarios=OfflineChallenge_1 --agent=leaderboard/leaderboard/autoagents/detour_agents/my_detour_agent.py --routes=srunner/data/my_routes_test.xml

taskkill //IM "CarlaUE4*" //F

echo "Press ESC key to quit"
while read -r -n1 key ; do
read -t 3 -n 1
if [[ $key == $'\e' ]] ; then
break ;
fi
userinput+=$key
done
printf "\nYou have typed : $userinput\n"

#D:\hzsxi\Documents\2022\CARLA_0.9.13\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.13-py3.7-win-amd64.egg;D:\hzsxi\Documents\2022\CARLA_0.9.13\WindowsNoEditor\PythonAPI\carla\;D:\hzsxi\Documents\2022\PASS_RUNNER;D:\hzsxi\Documents\2022\PASS_RUNNER\leaderboard\