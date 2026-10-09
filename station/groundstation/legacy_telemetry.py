"""把现有 contcp_t 遥测解析器接入 URGC 状态模型。

这是迁移期适配层：解析规则仍只有 contcp_t.py 一份，先保证线上字节兼容；
后续再把这些纯解析函数整体搬入 protocol 包，并让旧 CLI 反向复用。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

try:
    from ..contcp_t import (
        EKF_CSV_HEADER,
        EKF_FIELDS,
        EKF_INT_FIELDS,
        UGCTL_CSV_HEADER,
        UGCTL_FIELDS,
        UGCTL_INT_FIELDS,
        UGWRM_CSV_HEADER,
        UGWRM_FIELDS,
        UGWRM_INT_FIELDS,
        ekf_checksum_ok,
        ekf_csv_row,
        ekf_parse,
        ugctl_csv_row,
        ugctl_parse,
        ugwrm_csv_row,
        ugwrm_parse,
    )
except ImportError:
    from contcp_t import (
        EKF_CSV_HEADER,
        EKF_FIELDS,
        EKF_INT_FIELDS,
        UGCTL_CSV_HEADER,
        UGCTL_FIELDS,
        UGCTL_INT_FIELDS,
        UGWRM_CSV_HEADER,
        UGWRM_FIELDS,
        UGWRM_INT_FIELDS,
        ekf_checksum_ok,
        ekf_csv_row,
        ekf_parse,
        ugctl_csv_row,
        ugctl_parse,
        ugwrm_csv_row,
        ugwrm_parse,
    )


@dataclass(frozen=True)
class DecodedTelemetry:
    """一行板端文本解析后的主题与新鲜度约束。"""

    topic: str
    data: Dict[str, Any]
    device_tick_ms: Optional[int]
    stale_after_ms: int


def decode_telemetry_line(line: str) -> Optional[DecodedTelemetry]:
    """解析 URGC 首批接入的三类遥测；坏帧和其他文本返回 None。"""

    line = line.strip()
    if not line or not ekf_checksum_ok(line):
        return None

    if line.startswith("$UGEKF"):
        row = ekf_parse(line)
        topic = "ugekf"
    elif line.startswith("$UGCTL"):
        row = ugctl_parse(line)
        topic = "ugctl"
    elif line.startswith("$UGWRM"):
        row = ugwrm_parse(line)
        topic = "ugwrm"
    else:
        return None

    if row is None:
        return None
    return DecodedTelemetry(
        topic=topic,
        data=dict(row),
        device_tick_ms=row.get("t_ms"),
        stale_after_ms=500,
    )
