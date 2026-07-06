import ast
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
PLANNER_DIR = REPO_ROOT / "src" / "robot_planner"


ADAPTER_SOURCES = {
    "mecanum_competition": PLANNER_DIR
    / "local_planner"
    / "mecanum_competition_planner_node.py",
    "mecanum_omni": PLANNER_DIR / "local_planner" / "mecanum_omni_planner_node.py",
}

ADAPTER_CONFIGS = {
    "mecanum_competition": PLANNER_DIR / "config" / "robot1_mecanum_competition.yaml",
    "mecanum_omni": PLANNER_DIR / "config" / "robot1_mecanum_omni.yaml",
}


def _existing_sources():
    sources = {mode: path for mode, path in ADAPTER_SOURCES.items() if path.exists()}
    assert sources, "No expected mecanum planner adapter source exists"
    return sources


def _existing_configs():
    configs = {mode: path for mode, path in ADAPTER_CONFIGS.items() if path.exists()}
    assert configs, "No expected mecanum planner config exists"
    return configs


def _string_constant(node, named_constants=None):
    named_constants = named_constants or {}
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name):
        return named_constants.get(node.id)
    return None


def _declare_topic_default(node, named_constants):
    if not isinstance(node, ast.Call):
        return None
    if not isinstance(node.func, ast.Attribute):
        return None
    if node.func.attr not in {
        "_declare_str",
        "_declare_robot1_topic",
        "_declare_fixed_output_topic",
    }:
        return None
    if len(node.args) < 2:
        return None
    return _string_constant(node.args[1], named_constants)


def _self_attr_name(node):
    if not isinstance(node, ast.Attribute):
        return None
    if isinstance(node.value, ast.Name) and node.value.id == "self":
        return node.attr
    return None


def _publisher_topics(source: str):
    tree = ast.parse(source)
    named_constants = {}
    declared_strings = {}
    publishers = []

    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        value = _string_constant(node.value)
        if value is None:
            continue
        for target in node.targets:
            if isinstance(target, ast.Name):
                named_constants[target.id] = value

    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            default = _declare_topic_default(node.value, named_constants)
            if default is None:
                continue
            for target in node.targets:
                attr = _self_attr_name(target)
                if attr is not None:
                    declared_strings[attr] = default

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != "create_publisher" or len(node.args) < 2:
            continue

        topic_arg = node.args[1]
        topic = _string_constant(topic_arg, named_constants)
        if topic is None:
            attr = _self_attr_name(topic_arg)
            topic = declared_strings.get(attr, f"<dynamic:{attr}>")
        publishers.append(topic)

    return publishers


def _planner_params(config_path: Path):
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    assert data

    first_node = next(iter(data.values()))
    assert isinstance(first_node, dict)
    params = first_node.get("ros__parameters")
    assert isinstance(params, dict)
    return params


def _mode_strings(source: str):
    tree = ast.parse(source)
    known_modes = {
        "mecanum_competition",
        "mecanum_omni",
        "external_dwa",
        "simple_tracker",
    }
    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and node.value in known_modes
    }


def _method_source(source: str, method_name: str):
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == method_name:
            return ast.get_source_segment(source, node)
    raise AssertionError(f"Method {method_name} was not found")


def test_adapter_publishers_stay_on_raw_velocity_safety_seam():
    for source_path in _existing_sources().values():
        source = source_path.read_text(encoding="utf-8")
        publisher_topics = _publisher_topics(source)

        assert "/robot1/local_path" in publisher_topics
        assert "/robot1/cmd_vel_raw" in publisher_topics
        assert "/robot1/planner_state" in publisher_topics
        assert "/robot1/cmd_vel" not in publisher_topics
        assert "cmd_vel" not in publisher_topics


def test_adapter_config_exposes_robot1_outputs_without_final_cmd_vel():
    for config_path in _existing_configs().values():
        params = _planner_params(config_path)

        assert params["local_path_topic"] == "/robot1/local_path"
        assert params["cmd_vel_raw_topic"] == "/robot1/cmd_vel_raw"
        assert params["planner_state_topic"] == "/robot1/planner_state"

        final_cmd_keys = {
            key
            for key in params
            if key in {"cmd_vel_topic", "output_cmd_vel_topic", "final_cmd_vel_topic"}
        }
        assert not final_cmd_keys
        assert all(value != "/robot1/cmd_vel" for value in params.values())
        assert all(value != "/cmd_vel" for value in params.values())


def test_adapter_supports_primary_and_fallback_modes_without_ros_graph():
    source_modes = {
        mode: _mode_strings(path.read_text(encoding="utf-8"))
        for mode, path in _existing_sources().items()
    }

    for primary_mode, modes in source_modes.items():
        assert primary_mode in modes
        assert "simple_tracker" in modes

    for config_mode, config_path in _existing_configs().items():
        params = _planner_params(config_path)
        assert params["planner_mode"] == config_mode
        assert params["fallback_planner_mode"] == "simple_tracker"

        if config_mode in source_modes:
            assert params["planner_mode"] in source_modes[config_mode]
            assert params["fallback_planner_mode"] in source_modes[config_mode]


def test_competition_adapter_uses_hard_stop_before_core_not_front_stop():
    source_path = ADAPTER_SOURCES["mecanum_competition"]
    config_path = ADAPTER_CONFIGS["mecanum_competition"]
    assert source_path.exists()
    assert config_path.exists()

    source = source_path.read_text(encoding="utf-8")
    params = _planner_params(config_path)
    front_guard = _method_source(source, "_front_obstacle_too_close")

    assert params["front_stop_distance"] == 0.32
    assert params["hard_stop_distance_m"] == 0.10
    assert "hard_stop_distance_m" in source
    assert "self.hard_stop_distance_m" in front_guard
    assert "self.front_stop_distance" not in front_guard
