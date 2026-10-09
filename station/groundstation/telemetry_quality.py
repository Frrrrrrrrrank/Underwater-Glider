"""共享遥测字段质量规则：数值可用性与接收新鲜度分别表达。"""

from __future__ import annotations

from copy import deepcopy
from math import isfinite
from typing import Any, Dict, Mapping, Tuple


# 这些字段是 EKF 估计输出；计数器、状态位、原始深度和协方差诊断仍可单独查看。
EKF_ESTIMATE_FIELDS = (
    "roll_deg", "pitch_deg", "yaw_deg", "pN", "pE", "depth",
    "vN", "vE", "vD", "bgx", "bgy", "bgz", "bax", "bay", "baz",
)
WORM_TRAVEL_FIELDS = ("stroke_percent", "mm_from_front")
WORM_POSITION_FIELDS = ("pulse",) + WORM_TRAVEL_FIELDS


def evaluate_telemetry_quality(
    topic: str, data: Mapping[str, Any]
) -> Tuple[Dict[str, Any], Dict[str, str], Dict[str, Any]]:
    """返回独立的 JSON 数据、无效字段原因和质量摘要。

    非有限浮点数必须转为 null，否则 REST 序列化或浏览器 JSON.parse 会失败。
    板端有效位不抹去有限的原始数值，便于诊断；界面必须同时读取 invalid_fields。
    valid 仅表示通过本模块已实现的字段检查，不代表设备健康或具备控制权限。
    """

    invalid_fields: Dict[str, str] = {}

    def clean(value: Any, path: str) -> Any:
        if isinstance(value, float) and not isfinite(value):
            invalid_fields[path] = "non_finite"
            return None
        if isinstance(value, Mapping):
            return {
                key: clean(item, "{}.{}".format(path, key) if path else str(key))
                for key, item in value.items()
            }
        if isinstance(value, (list, tuple)):
            return [clean(item, "{}[{}]".format(path, index)) for index, item in enumerate(value)]
        return deepcopy(value)

    cleaned = clean(dict(data), "")
    unknown_reasons = []

    def mark(fields: Tuple[str, ...], reason: str) -> None:
        for name in fields:
            if name in cleaned:
                # 非有限数优先保留直接原因；同字段有多个有效位时保留首个失败原因。
                invalid_fields.setdefault(name, reason)

    if topic == "ugekf":
        flags = cleaned.get("fl")
        if isinstance(flags, int):
            if not flags & (1 << 1):
                mark(EKF_ESTIMATE_FIELDS, "ekf_not_valid")
        else:
            unknown_reasons.append("ekf_validity_not_reported")
    elif topic == "ugwrm":
        # data_fresh 判的是板端驱动器反馈，不等于主机刚收到这条遥测。
        if cleaned.get("data_fresh") == 0:
            mark(WORM_POSITION_FIELDS, "worm_feedback_stale")
        elif cleaned.get("data_fresh") != 1:
            unknown_reasons.append("worm_feedback_validity_not_reported")
        if cleaned.get("span_valid") == 0:
            mark(WORM_TRAVEL_FIELDS, "worm_span_invalid")
        elif cleaned.get("span_valid") != 1:
            unknown_reasons.append("worm_span_validity_not_reported")

    # UGCTL 的 armed/output_valid/fault 是独立运行状态，本阶段不据此否定整行诊断数据。
    reasons = list(dict.fromkeys(invalid_fields.values())) + unknown_reasons
    status = "invalid" if invalid_fields else ("unknown" if unknown_reasons else "valid")
    return cleaned, invalid_fields, {"status": status, "reasons": reasons}
