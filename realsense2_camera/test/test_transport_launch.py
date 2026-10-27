import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.utilities import evaluate_parameters


@pytest.mark.parametrize("suffix,intra_process", [("", "true"), ("1", "false")])
def test_camera_container_and_intra_process(monkeypatch, suffix, intra_process):
    path = Path(__file__).resolve().parents[1] / "launch" / "rs_launch.py"
    spec = importlib.util.spec_from_file_location("realsense_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    params = {}
    for parameter in module.configurable_parameters:
        name = parameter["name"] + suffix
        context.launch_configurations[name] = parameter["default"]
        params[parameter["name"]] = module.LaunchConfiguration(name)
    assert context.launch_configurations["intra_process_comms" + suffix] == "true"
    context.launch_configurations["intra_process_comms" + suffix] = intra_process
    monkeypatch.setattr(module, "yaml_to_dict", lambda path: {"use_lifecycle_node": False})
    captured = {}

    def container(**kwargs):
        captured.update(kwargs)
        return ComposableNodeContainer(**kwargs)

    monkeypatch.setattr(module.launch_ros.actions, "ComposableNodeContainer", container)
    actions = module.launch_setup(context, params, suffix)
    assert len(actions) == 1
    assert isinstance(actions[0], ComposableNodeContainer)
    assert captured["executable"] == "component_container"
    components = captured["composable_node_descriptions"]
    assert len(components) == 1
    camera = components[0]
    assert perform_substitutions(context, camera.node_plugin) == "realsense2_camera::RealSenseNodeFactory"
    assert perform_substitutions(context, camera.node_name) == "camera"
    assert perform_substitutions(context, camera.node_namespace) == "camera"
    extra = evaluate_parameters(context, camera.extra_arguments)
    assert extra == ({"use_intra_process_comms": intra_process == "true"},)
