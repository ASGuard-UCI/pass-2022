#!/bin/bash

script_dir=$(dirname $(readlink -f $0))
repo_dir=$(realpath ${script_dir}/..)
export ACHILLES_ROOT=${repo_dir}

# Note: replace ACHILLES_LOGDIR with your own paths
export ACHILLES_LOGDIR=${ACHILLES_ROOT}/log

# Make sure this log directory exists
mkdir -p ${ACHILLES_ROOT}/log

# export SCENARIO=${ACHILLES_ROOT}/achilles/data/scenario_stop_sign.json
# export SCENARIO=${ACHILLES_ROOT}/achilles/data/scenario_stop_sign_shorter.json
export SCENARIO=${ACHILLES_ROOT}/achilles/data/scenario_stop_sign_shortest.json
export ROUTE=${ACHILLES_ROOT}/achilles/data/route_stop_sign.json
export AGENT=${ACHILLES_ROOT}/achilles/autoagents/modular_agent.py
# export AGENT_CONFIG=${ACHILLES_ROOT}/achilles/data/ego_stop_sign_hdmap3d.yaml
# export AGENT_CONFIG=${ACHILLES_ROOT}/achilles/data/ego_stop_sign_pinhole3d.yaml
export AGENT_CONFIG=${ACHILLES_ROOT}/achilles/data/ego_stop_sign_fusion.yaml

MODELLIB=${ACHILLES_ROOT}/attacks/robust-physical-attack/models/research
export PYTHONPATH=${ACHILLES_ROOT}:${ACHILLES_ROOT}/PythonAPI:${MODELLIB}:${PYTHONPATH}

# Using CPU version
export CUDA_VISIBLE_DEVICES=-1

python3 ${ACHILLES_ROOT}/achilles/achilles_evaluator.py \
  --scenario=${SCENARIO} \
  --route=${ROUTE} \
  --agent=${AGENT} \
  --agent-config=${AGENT_CONFIG}
