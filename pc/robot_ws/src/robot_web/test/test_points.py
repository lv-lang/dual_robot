from robot_web.points import find_default_task_points_file


def test_find_default_task_points_file_accepts_config_directory(tmp_path):
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    task_points = config_dir / "task_points.yaml"
    task_points.write_text("points: {}\n", encoding="utf-8")

    assert find_default_task_points_file(config_dir) == task_points
