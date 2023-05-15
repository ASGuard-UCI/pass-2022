# PASS: Platform for Autonomous Driving Safety and Security

PASS is a simulation-centric Autonomous Driving security & safety evaluation platform.

Please follow the next steps to setup and run the platform.

## First-time setup (take the ShapeShifter for example)

1. Download the map bundle from [here](https://drive.google.com/file/d/1faczNvMEsuJ9vHCpfAd-3vhZH7a86OY7/view?usp=sharing).
2. Download the customized SVL simulator from [here](https://drive.google.com/file/d/12bd3pFms3tSy7NHNLwuQjUKscDPl5XM0/view?usp=sharing).
  - This customized SVL builds upon SVL v2021.3 with additional dynamic stop sign patching capability.
  - Unzip the downloaded package and enter the extracted folder, which is `SimBuild_StopSign` by default.
  - Check if a file `stop_sign_sf_1.png` under `simulator_Data/Resources` exists. This is the adversarial patch generated from ShapeShifter attack. To run simulation with a benign stop sign, please replace this file with the one named `stop_sign_sl_1.png`.
3. Start the `simulator` executable, select `Go Online`, and click `LINK TO CLOUD` or `Open Browser` on the simulator window.
  - Register a SVL account through the web interface if it is the first time SVL execution.
  - Sign in the web system, create a cluster, and assign a name to the cluster, e.g., `achilles`.
    - Under `Library/Maps`, click `Add New` and upload the map bundle.
      - Change the map name to `SanFrancisco_Achilles`, click `Next`.
      - Click `Publish`. (Note: clicking `Publish and Add to Store` will make the map bundle public to everyone!)
    - Under `Simulations`, click `Add New` to create a new simulation.
      - Assign a name, e.g, `API Mode`, to this new simulation instance.
      - Under `Select Cluster`, select the cluster name you just created, i.e., `achilles`, and Click `Next`.
      - Under `Runtime Template`, select `API Only`. Click `Next`.
      - Click `Next` and click `Publish`.
<!--  - Close simulator window and web interface tab. !-->
4. Clone this repository, run a terminal, and navigate to the cloned repository, e.g., `ade2e`, in the terminal.
  - Build the docker image for ShapeShifter: `bash ./docker/build.sh`, select `1`
  - Start the docker container: `bash ./docker/start.sh`, select `1`
  - Enter the docker container: `bash ./docker/enter.sh`, select `1`
  - Navigate to the `ade2e` folder to setup the SVL environment: `bash ./setup_env.sh`
  - Exit the docker container: `exit`

## Run system-level evaluation on the stop sign attack

1. Navigate to your `ade2e` folder in the terminal.
2. Enter the docker container: `bash ./docker/enter.sh`, select `1`
3. Navigate to the same `ade2e` folder in the container.
4. Check the `./scripts/run_evaluation.sh` file to set the correct environment variables.
5. Start the already created `API Mode` simulation instance in the simulator web system.
6. Run the evaluation: `bash ./scripts/run_evaluation.sh`
  - If you encounter case when SVL suddenly closes itself (likely due to high runtime overhead), please try switch to offline simulation mode.
  - Simulation log will be stored in the log directory at the end of simulation.

## Acknowledgements

The PASS platform makes use of the following open source projects:

 - [CARLA leaderboard](https://github.com/carla-simulator/leaderboard)
 - [ScenarioRunner for CARLA](https://github.com/carla-simulator/scenario_runner)
