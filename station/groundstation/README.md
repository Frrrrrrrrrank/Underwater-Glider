# UnderwaterRobotGroundControl Core

简称：URGC。

这是常驻地面站核心的只读遥测阶段。默认监听板端 TCP 客户端使用的
`192.168.57.100:8800`，复用 `contcp_t.py` 的解析规则接收 `$UGEKF`、
`$UGCTL`、`$UGWRM`，再把最新状态发布给网页。

建立连接后只发送一次 `MSG_STREAM_CONTROL`，掩码为 `EKF | CTL | WRM`
（`0x0D`），用于请求板端输出遥测；当前没有执行器、回零、PID、参数写入或任务控制接口。
URGC 是设备 socket 的唯一所有者，因此运行它时不要同时启动 `contcp_t.py`，否则两个程序会
竞争同一个 8800 端口。旧脚本仍保留，可在不启动 URGC 时单独使用。

## 启动

在 station 目录执行：

    python -m pip install -r groundstation/requirements.txt
    python -m groundstation

设备网卡地址或端口不同时，可在 PowerShell 中先设置：

    $env:URGC_DEVICE_HOST = "192.168.57.100"
    $env:URGC_DEVICE_PORT = "8800"
    python -m groundstation

只查看网页骨架、不监听设备端口时：

    $env:URGC_DEVICE_LINK_ENABLED = "0"
    python -m groundstation

服务默认只监听 127.0.0.1:8000：

- GET /api/v1/health：后端进程健康状态；
- GET /api/v1/state：完整只读状态快照；
- GET /api/v1/recording：本次设备会话的记录目录、文件、行数及写入错误；
- WS /api/v1/ws/state：状态变更与 1 秒新鲜度心跳；
- /docs：FastAPI 自动接口文档。

frontend/dist 存在时会由核心托管；开发阶段可单独运行 Vite。不要使用多 worker：设备 TCP
socket 必须只有一个所有者，多 worker 会产生多个互相竞争的核心实例。

## 会话记录

默认自动记录到 station/logs/sessions 下的独立会话目录。每次设备 TCP 连接建立新目录：

- raw.log：主机收包时间和收到的完整文本行，包含未通过解析的行；不是逐字节抓包。
- events.jsonl：会话建立、断开原因。
- ekf_日期_时间.csv、ctl_日期_时间.csv、worm_日期_时间.csv：复用旧 CLI 的列定义，
  相应主题首次到达时创建。host_iso 使用实际接收时的 UTC 时间。

CSV 可以交给原绘图工具，输入目录需指定到对应会话目录；旧默认目录不会自动递归查找 sessions。
记录由有界队列和单独磁盘线程完成。写入失败或队列溢出会报告 error/dropped_records，
不会伪装为完整记录。当前按行 flush，不保证操作系统或机器突然掉电时最后数据已物理落盘。
原始数据/CSV保留故障 nan，网页快照将非有限值转换成 null 并附无效原因。

可用环境变量 URGC_LOG_DIR 指定目录；URGC_RECORDING_ENABLED=0 可禁用记录。
禁用设备链路时也不会创建日志。网页尚未提供历史会话浏览、下载和回放入口。

## 数据质量

state.required_topics / missing_topics 说明本阶段期望的三个主题是否齐备。
fresh 仅表示接收时间满足阈值；quality 和 invalid_fields 表示字段有效性，
不能把任何一个标志单独当作允许控制的依据。数值有限但板端判为无效时，原数保留供诊断，
正常监视面板应显示不可用。网页另用本地时钟检测推送中断，不依赖下一条后台心跳才变灰。

后续实施和验收安排统一记录在仓库根目录的本地 `project_status.md`（不随 Git 分发）。

## 本地验证

在仓库根目录，使用已装 requirements 的 station/.venv：

    station\.venv\Scripts\python.exe -B -m unittest station.tools.test_groundstation_state station.tools.test_groundstation_quality station.tools.test_groundstation_link station.tools.test_groundstation_recording -v
    station\.venv\Scripts\python.exe -X utf8 -B station/tools/verify_groundstation_workbench.py

后一项需要先构建前端，默认使用 Windows 已安装的 Edge，可用 --edge 指定浏览器路径。
它只运行明确标记的本地测试夹具，验证三个视口的四个页面、纯黑/渐变/导航样式、
关键表格完整可见、长记录路径、缺失有效位、非有限数、遥测停流、恢复和核心断开；
另验证主页操作分组不发送HTTP写请求。截图输出到 station/plots/urgc-workbench-check。
不会启动设备 8800 监听，也不代表真实板端或控制功能验收通过。
