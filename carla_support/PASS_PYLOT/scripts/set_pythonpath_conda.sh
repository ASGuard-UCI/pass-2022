#!/bin/bash

if [ -z "$PYLOT_HOME" ]; then
    echo "setting \$PYLOT_HOME..."
    export PYLOT_HOME=$(pwd)
fi
if [ -z "$CARLA_HOME" ]; then
    echo "setting \$CARLA_HOME..."
    export CARLA_HOME=$PYLOT_HOME/dependencies/CARLA_0.9.10.1/
fi

CARLA_EGG=$(ls $CARLA_HOME/PythonAPI/carla/dist/carla*py3*egg)
export PYTHONPATH=$PYTHONPATH:$PYLOT_HOME:/$PYLOT_HOME/dependencies/:$CARLA_EGG:$CARLA_HOME/PythonAPI/carla/:$PYLOT_HOME/dependencies/lanenet/
