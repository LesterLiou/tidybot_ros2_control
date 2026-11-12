from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest

launch = pytest.importorskip('launch')
pytest.importorskip('launch_ros')


@pytest.mark.parametrize('mode', ['manual', 'navigation'])
def test_gamepad_mux_is_single_output_path(monkeypatch, mode):
    path = Path(__file__).resolve().parents[1] / 'launch' / 'tidybot_control.launch.py'
    spec = spec_from_file_location('tidybot_control_launch', path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: '/fake/share')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = launch.LaunchContext()
    context.launch_configurations['control_mode'] = mode

    nodes = {node['executable']: node for node in module.control_nodes(context)}
    assert set(nodes) == {'joy_node', 'gamepad_teleop', 'control_mux', 'base_node'}
    assert nodes['base_node']['parameters'][-1]['require_enable']
    assert nodes['gamepad_teleop']['parameters'][-1]['require_both_deadman']
    assert not nodes['gamepad_teleop']['parameters'][-1]['shutdown_on_guide']
    assert dict(nodes['gamepad_teleop']['remappings'])['/tidybot/enable'] == '/teleop/enable'
    mux_outputs = dict(nodes['control_mux']['remappings'])
    if mode == 'navigation':
        assert mux_outputs == {
            '/cmd_vel': '/manual_control/cmd_vel',
            '/tidybot/enable': '/manual_control/tidybot/enable',
            '/tidybot/control_frame': '/manual_control/tidybot/control_frame',
        }
    else:
        assert not mux_outputs
