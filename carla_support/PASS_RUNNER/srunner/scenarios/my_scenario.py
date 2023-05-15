
import py_trees
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle

class MyScenario(BasicScenario):
    """
    Some documentation on NewScenario
    :param world is the CARLA world
    :param ego_vehicles is a list of ego vehicles for this scenario
    :param config is the scenario configuration (ScenarioConfiguration)
    :param randomize can be used to select parameters randomly (optional, default=False)
    :param debug_mode can be used to provide more comprehensive console output (optional, default=False)
    :param criteria_enable can be used to disable/enable scenario evaluation based on test criteria (optional, default=True)
    :param timeout is the overall scenario timeout (optional, default=60 seconds)
    """

    # some ego vehicle parameters
    # some parameters for the other vehicles

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=300):
        """
        Initialize all parameters required for NewScenario
        """
        self.config = config
        self.route = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        # Call constructor of BasicScenario
        super(MyScenario, self).__init__(
          "MyScenario",
          ego_vehicles,
          config,
          world,
          debug_mode,
          criteria_enable=criteria_enable)


    def _create_behavior(self):
        """
        Setup the behavior for NewScenario
        """
        # Build behavior tree
        sequence = py_trees.composites.Sequence("MasterScenario")
        idle_behavior = Idle()
        sequence.add_child(idle_behavior)

        return sequence

    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for NewScenario
        """

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()