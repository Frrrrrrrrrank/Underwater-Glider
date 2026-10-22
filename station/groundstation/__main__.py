"""允许在 station 目录执行 python -m groundstation。"""

import uvicorn

from .app import app


def main() -> None:
    # 只绑定回环地址，首版不会把尚未鉴权的接口暴露到局域网。
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="info")


if __name__ == "__main__":
    main()
