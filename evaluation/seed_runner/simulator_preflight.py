"""Verify strict local rule settings and immutable simulator provenance."""
import json
from evaluation.seed_runner.manifest import file_hash


def check_rule_settings(game,settings):
    if (game.get('IgnoreAllHitCollision') is not False
            or game.get('IgnoreOverTime') is not False):
        raise ValueError('collision and timeout rules must both be enforced')
    if settings.get('RpcEnabled') is not False:
        raise ValueError('official baseline requires RpcEnabled=false')
    vehicle=settings.get('Vehicles',{}).get('drone_1',{})
    if (vehicle.get('EnableCollisions') is not True
            or vehicle.get('EnableCollisionPassthrogh',False)
            or vehicle.get('EnableCollisionPassthrough',False)):
        raise ValueError('vehicle collision passthrough is forbidden')


def verify_local_simulator(root):
    simulator=root/'simulator/simulator_12.0.0.5'
    game_path=simulator/'Build/LinuxNoEditor/RMUA/Content/Configs/GameConfig.json'
    settings_path=root/'configs/simulator_settings.json'
    game=json.loads(game_path.read_text())
    settings=json.loads(settings_path.read_text())
    check_rule_settings(game,settings)
    baseline=json.loads((root/'configs/simulator_baseline.json').read_text())
    binary='Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping'
    binary_hash=file_hash(simulator/binary)
    if binary_hash != baseline['sha256'][binary]:
        raise ValueError('simulator binary changed from the frozen baseline')
    plugin='Build/LinuxNoEditor/RMUA/Plugins/AirSimROS/Binaries/Linux/libairsimros.so'
    plugin_hash=file_hash(simulator/plugin)
    if plugin_hash != baseline['sha256'][plugin]:
        raise ValueError('simulator ROS plugin changed from the frozen baseline')
    settings_hash=file_hash(settings_path)
    if settings_hash != baseline['runtime_settings']['sha256']:
        raise ValueError('simulator settings changed from the frozen baseline')
    return {'strict_rules_verified':True,'game_config':game,'game_config_sha256':file_hash(game_path),
            'binary_sha256':binary_hash,'ros_plugin_sha256':plugin_hash,'settings_sha256':settings_hash}
