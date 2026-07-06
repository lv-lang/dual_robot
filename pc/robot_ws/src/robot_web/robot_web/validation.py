from robot_web.exceptions import GatewayError


ALLOWED_TASK_TYPES = {"DELIVERY", "INSPECTION"}
ALLOWED_ROBOT_PREFERENCES = {"auto", "mecanum", "ackermann"}


def _required_string(payload, field):
    value = payload.get(field)
    if not isinstance(value, str) or not value.strip():
        raise GatewayError(400, "invalid_template", f"{field} is required")
    return value.strip()


def validate_template_payload(payload, point_catalog):
    normalized_payload = dict(payload)
    if "display_name" not in normalized_payload and "name" in normalized_payload:
        normalized_payload["display_name"] = normalized_payload["name"]
    if "target_point_ids" not in normalized_payload and "target_points" in normalized_payload:
        normalized_payload["target_point_ids"] = normalized_payload["target_points"]

    display_name = _required_string(normalized_payload, "display_name")
    task_type = _required_string(normalized_payload, "task_type").upper()
    if task_type not in ALLOWED_TASK_TYPES:
        raise GatewayError(
            400,
            "invalid_task_type",
            "task_type must be DELIVERY or INSPECTION",
            {"task_type": task_type},
        )

    robot_preference = str(normalized_payload.get("robot_preference") or "auto")
    if robot_preference not in ALLOWED_ROBOT_PREFERENCES:
        raise GatewayError(
            400,
            "invalid_robot_preference",
            "robot_preference must be auto, mecanum, or ackermann",
            {"robot_preference": robot_preference},
        )

    raw_points = normalized_payload.get("target_point_ids")
    if not isinstance(raw_points, list) or not raw_points:
        raise GatewayError(
            400,
            "invalid_target_points",
            "target_point_ids must be a non-empty list",
        )
    target_point_ids = []
    for raw_point in raw_points:
        if not isinstance(raw_point, str) or not raw_point.strip():
            raise GatewayError(
                400,
                "invalid_target_points",
                "target_point_ids must contain non-empty point ids",
            )
        target_point_ids.append(raw_point.strip())

    unknown = [point_id for point_id in target_point_ids if point_id not in point_catalog]
    if unknown:
        raise GatewayError(
            400,
            "unknown_task_points",
            "template references unknown task points",
            {"unknown_point_ids": unknown},
        )

    if task_type == "DELIVERY":
        if len(target_point_ids) != 2:
            raise GatewayError(
                400,
                "invalid_delivery_template",
                "DELIVERY templates require exactly one pickup point and one delivery point",
            )
        first = point_catalog[target_point_ids[0]].kind
        second = point_catalog[target_point_ids[1]].kind
        if first != "PICKUP" or second != "DELIVERY":
            raise GatewayError(
                400,
                "invalid_delivery_template",
                "DELIVERY templates must target pickup then delivery points",
                {"point_kinds": [first, second]},
            )

    if task_type == "INSPECTION":
        bad = [
            point_id for point_id in target_point_ids
            if point_catalog[point_id].kind != "INSPECTION"
        ]
        if bad:
            raise GatewayError(
                400,
                "invalid_inspection_template",
                "INSPECTION templates may contain inspection points only",
                {"invalid_point_ids": bad},
            )

    try:
        sort_order = int(normalized_payload.get("sort_order", 100))
    except (TypeError, ValueError):
        raise GatewayError(400, "invalid_sort_order", "sort_order must be an integer")

    return {
        "display_name": display_name,
        "task_type": task_type,
        "target_point_ids": target_point_ids,
        "robot_preference": robot_preference,
        "sort_order": sort_order,
    }
