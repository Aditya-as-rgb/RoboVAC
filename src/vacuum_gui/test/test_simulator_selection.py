from PySide6.QtWidgets import QApplication

from vacuum_gui import preflight
from vacuum_gui.commands import StackConfig, restart_fields
from vacuum_gui.launch_panel import LaunchPanel


def test_simulator_selector_carries_backend_and_camera_into_command():
    app = QApplication.instance() or QApplication([])
    panel = LaunchPanel()
    old_config = StackConfig.from_dict({'dirt': 5})
    panel.set_config(old_config)
    assert panel.sim.currentText() == 'Gazebo'
    assert 'sim:=gazebo' in panel.config().stack_command()

    panel.sim.setCurrentIndex(panel.sim.findData('mujoco'))
    panel.cameras.setChecked(True)
    cfg = StackConfig.from_dict(panel.config().to_dict())
    assert cfg.sim == 'mujoco'
    assert 'sim:=mujoco' in cfg.stack_command()
    assert 'cameras:=true' in cfg.stack_command()
    assert restart_fields(cfg.to_dict(), old_config.to_dict()) == ['sim', 'cameras']
    panel.set_running(True, cfg)
    assert not panel.sim.isEnabled()
    panel.set_running(False)
    assert panel.sim.isEnabled()
    panel.close()
    app.processEvents()


def test_mujoco_preflight_checks_optional_install_and_world(tmp_path, monkeypatch):
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(tmp_path))
    assert not preflight.check_mujoco().ok

    index = tmp_path / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    for package in ('vacuum_mujoco', 'mujoco_ros2_control', 'diff_drive_controller',
                    'joint_state_broadcaster'):
        (index / package).touch()
    assert not preflight.check_mujoco().ok

    lib = tmp_path / 'lib'
    lib.mkdir()
    (lib / 'libvacuum_mujoco_sensors.so').touch()
    assert preflight.check_mujoco().ok
    assert preflight.check_world(StackConfig(sim='mujoco')).ok
    assert not preflight.check_world(StackConfig(sim='mujoco', world='house.world')).ok
