import copy
import unittest
from evaluation.seed_runner.simulator_preflight import check_rule_settings


class SimulatorRuleTests(unittest.TestCase):
    def test_rule_overrides_and_missing_values_are_rejected(self):
        game={'IgnoreAllHitCollision':False,'IgnoreOverTime':False}
        settings={'RpcEnabled':False,'Vehicles':{'drone_1':{'EnableCollisions':True,
                                                           'EnableCollisionPassthrogh':False}}}
        check_rule_settings(game,settings)
        for key in game:
            altered=dict(game);altered[key]=True
            with self.assertRaises(ValueError):check_rule_settings(altered,settings)
        with self.assertRaises(ValueError):check_rule_settings({},settings)
        for key,value in (('EnableCollisions',False),('EnableCollisionPassthrogh',True)):
            altered=copy.deepcopy(settings);altered['Vehicles']['drone_1'][key]=value
            with self.assertRaises(ValueError):check_rule_settings(game,altered)
        altered=copy.deepcopy(settings);altered['RpcEnabled']=True
        with self.assertRaises(ValueError):check_rule_settings(game,altered)


if __name__ == '__main__':
    unittest.main()
