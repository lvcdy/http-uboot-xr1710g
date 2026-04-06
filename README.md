# U-Boot XR1710G 项目说明

本文档聚焦本仓库中 XR1710G 平台的实现细节，重点说明四件事：

1. U-Boot 如何为 XR1710G 构建
2. 板级功能如何实现（MAC、factory、恢复按键等）
3. HTTP 恢复界面如何与 U-Boot 主流程联动
4. 驱动、设备树、分区和环境变量之间的关系

## 1. 平台与代码入口

### 1.1 目标平台

- SoC 平台：Airoha AN7581（ARM64）
- 机型：XR1710G
- 代码中常见兼容串：`econet,xr1710g`、`gemtek,xr1710g`

说明：仓库中未出现 `brightspeed` 字符串。代码层面以 XR1710G 硬件平台识别，运营商品牌通常不直接写入源码。

### 1.2 关键代码路径

- 板级主逻辑：`board/airoha/an7581/an7581_rfb.c`
- 板级默认环境：`board/airoha/an7581/xr1710g.env`
- defconfig：`configs/xr1710g_defconfig`
- HTTP 恢复命令入口：`cmd/http_recovery.c`
- HTTP 恢复核心实现：`net/lwip/httpd_recovery.c`
- lwIP 网络收发桥接：`net/lwip/net-lwip.c`
- U-Boot 覆盖设备树：`arch/arm/dts/xr1710g-u-boot.dtsi`
- 上游设备树基底：`dts/upstream/src/arm64/airoha/xr1710g.dts`

## 2. 构建与配置（XR1710G）

### 2.1 配置

```bash
make xr1710g_defconfig
```

### 2.2 编译

```bash
make -j$(nproc)
```

### 2.3 交叉编译

本目标是 ARM64，建议使用 AArch64 工具链前缀，例如：

```bash
export CROSS_COMPILE=aarch64-linux-gnu-
make xr1710g_defconfig
make -j$(nproc)
```

`configs/xr1710g_defconfig` 的关键项：

- `CONFIG_ARM64=y`
- `CONFIG_TARGET_AN7581=y`
- `CONFIG_DEFAULT_DEVICE_TREE="airoha/xr1710g"`
- `CONFIG_NET_LWIP=y`
- `CONFIG_HTTPD_RECOVERY=y`
- `CONFIG_MTD_SPI_NAND=y`
- `CONFIG_MTD_UBI=y`
- `CONFIG_AIROHA_ETH=y`
- `CONFIG_AIROHA_SNFI_SPI=y`

### 2.4 通过 TTL 刷入这份 U-Boot

本节说明如何通过 TTL 串口进入 U-Boot 控制台并刷写本仓库编译出的 `u-boot.bin`。

推荐流程是 TTL 负责控制台操作，镜像通过 TFTP 下载到内存后再写入 SPI NAND。

#### 2.4.1 准备工作

1. 硬件连接

- USB 转 TTL（3.3V 电平）连接开发板 UART（TX/RX/GND）
- 串口参数通常为 `115200 8N1`

2. 产物准备

- 在主机编译得到 `u-boot.bin`

3. 网络准备（用于推荐方式）

- 主机启动 TFTP 服务，并把 `u-boot.bin` 放到 TFTP 根目录
- 开发板网口与主机可互通

#### 2.4.2 进入 U-Boot 命令行

1. 上电并在串口终端中打断自动启动
2. 进入 `U-Boot>` 提示符

建议先查看目标变量：

```bash
printenv recovery_dev_uboot uboot_ofs recovery_size_uboot loadaddr
```

本项目默认 U-Boot 写入目标为：

- 设备：`spi-nand0`
- 偏移：`0x600000`
- 大小上限：`0x100000`

#### 2.4.3 方式 A：TTL + TFTP 刷写（推荐）

1. 设置网络参数（示例）：

```bash
setenv ipaddr 192.168.1.2
setenv serverip 192.168.1.10
ping ${serverip}
```

2. 下载镜像到内存：

```bash
tftpboot ${loadaddr} u-boot.bin
```

3. 检查大小，确保不超过 `recovery_size_uboot`（默认 `0x100000`）

```bash
echo ${filesize}
```

4. 擦除并写入：

```bash
mtd erase ${recovery_dev_uboot} ${uboot_ofs} ${recovery_size_uboot}
mtd write ${recovery_dev_uboot} ${loadaddr} ${uboot_ofs} ${filesize}
```

5. 回读校验（建议）：

```bash
setexpr verify_addr ${loadaddr} + 0x200000
mtd read ${recovery_dev_uboot} ${verify_addr} ${uboot_ofs} ${filesize}
cmp.b ${loadaddr} ${verify_addr} ${filesize}
```

6. 重启验证：

```bash
reset
```

#### 2.4.4 方式 B：纯 TTL 串口传输刷写（可选）

如果当前环境不方便走 TFTP，可用串口传输命令（例如 `loadx`/`loady`，取决于当前镜像是否启用）。

示例（若 `loady` 可用）：

```bash
loady ${loadaddr}
```

随后在串口工具中发送 `u-boot.bin`（YMODEM），传输完成后重复方式 A 的第 3~6 步执行擦写和校验。

#### 2.4.5 风险与注意事项

1. 必须确认电平是 3.3V，禁止 5V TTL。
2. 擦写前确认目标设备、偏移和长度，避免误擦 `vendor/ubi` 区域。
3. `filesize` 不应大于 `recovery_size_uboot`。
4. 刷写过程中严禁断电。
5. 建议保留一种应急恢复路径（例如恢复按键 + HTTP 恢复）。

## 3. 启动与恢复总流程

### 3.1 默认启动路径

`board/airoha/an7581/xr1710g.env` 默认：

- `bootcmd=run boot_ubi || http_recovery`

含义：

1. 先尝试从 UBI 的 `fit` 卷引导
2. 若失败，自动进入 HTTP 恢复

### 3.2 按键触发恢复

在 `board_late_init()` 中会检测 `recovery-gpios`：

1. 同步运行时 MAC
2. 同步 factory 数据
3. 如果恢复按键按下：设置 `ipaddr/netmask/gatewayip`
4. 直接调用 `run_http_recovery()`

即使 `bootcmd` 还未执行，按键也可强制进入 Web 恢复。

## 4. HTTP 恢复与 U-Boot 的耦合点

### 4.1 编译耦合

- `net/lwip/Kconfig` 提供 `CONFIG_HTTPD_RECOVERY`
- `cmd/Makefile` 在该开关下编译 `http_recovery.o`
- `net/lwip/Makefile` 在该开关下编译 `httpd_recovery.o`

### 4.2 命令入口

`cmd/http_recovery.c` 的命令 `http_recovery` 会：

1. 设置静态网络参数（`192.168.255.1/24`）
2. 调用 `run_http_recovery()`

### 4.3 运行时主循环

`run_http_recovery()` 主要步骤：

1. 初始化恢复状态（进度、标志位、LED）
2. 启动以太网与 lwIP netif
3. 启动内置 DHCP server（向连接设备发地址）
4. 启动 lwIP httpd
5. 轮询收包与超时事件
6. 收到上传后执行刷写
7. 成功后延时重启，失败则保持服务运行

### 4.4 页面与接口

动态接口通过 `fs_open_custom()` 提供：

- `/status`：返回 JSON 进度（擦除/写入/总量/阶段）
- `/about`：返回 U-Boot 版本信息

上传处理：

- `POST /upload/firmware`
- `POST /upload/uboot`

上传数据先放到 RAM，再触发刷写，避免在 HTTP 回包期间阻塞网络。

## 5. 刷写策略（MTD/UBI）

### 5.1 目标选择优先级

恢复代码会根据环境变量解析目标：

- 目标名：`recovery_mtd` / `recovery_mtd_uboot`
- 原始设备：`recovery_dev` / `recovery_dev_uboot`
- 大小上限：`recovery_size` / `recovery_size_uboot`
- 偏移：`recovery_ofs` / `uboot_ofs`
- UBI 分区：`recovery_ubi_part` / `recovery_ubi_part_uboot`

流程上优先尝试可用的 MTD/UBI 目标，失败再做回退。

### 5.2 MTD 写入路径

1. 按擦除块大小对齐并整区擦除
2. 分块写入（循环写）
3. 更新进度与状态

### 5.3 UBI 写入路径

1. 选择目标 volume（默认 firmware 对应 `fit`，uboot 对应 `uboot`）
2. 必要时创建或扩容 volume
3. 处理保留卷逻辑（环境卷、factory 等）
4. `ubi_volume_write` 写入

## 6. 板级功能实现细节

### 6.1 MAC 地址与 FDT 修补

`an7581_rfb.c` 中 XR1710G 专有逻辑会：

1. 从 vendor 区域读取 DSD 数据
2. 提取 `lan_mac` / `wan_mac`
3. 写入环境变量 `ethaddr` / `eth1addr`
4. 在 `ft_board_setup()` 中修补 FDT 网卡节点的 `mac-address`

### 6.2 factory 数据同步

`xr1710g_sync_factory()` 会把 DSD EEPROM + MAC 信息同步到 UBI `factory` 卷，确保运行时和恢复流程使用一致的校准/地址数据。

### 6.3 恢复按键与 LED

- 恢复按键来自设备树 `recovery-gpios`
- 恢复期间可使用状态 LED（优先）或链路 LED（回退）显示活动

## 7. 设备树与分区关系

`dts/upstream/src/arm64/airoha/xr1710g.dts` 定义了 SPI NAND 分区与 UBI 卷：

- 固定分区：`vendor`、`chainloader`、`ubi`、`reserved_bmt`
- UBI 卷：`ubootenv`、`ubootenv2`、`uboot`、`fit`、`factory`

这正对应了：

- 引导路径从 `fit` 卷读取系统镜像
- 环境变量存放在 `ubootenv/ubootenv2`
- HTTP 恢复可写 `fit`（固件）与 `uboot`（引导器）

## 8. 常用验证与调试建议

### 8.1 构建后基础检查

1. 确认 `u-boot.bin` 已生成
2. 启动串口确认版本和板型匹配
3. `printenv` 检查 `bootcmd` 与 `recovery_*` 变量

### 8.2 恢复功能联调

1. 手动执行 `http_recovery`
2. 主机接入后访问 `http://192.168.255.1/`
3. 轮询 `/status` 观察擦写阶段变化
4. 上传 `firmware` 或 `uboot` 镜像验证流程

### 8.3 失败场景定位

- 无网络：先查 `CONFIG_AIROHA_ETH`、MDIO/PHY DTS、网线链路
- 上传失败：检查 `recovery_max`、目标分区大小和 RAM 缓冲地址
- 写入失败：重点看 MTD 擦除/写入返回码或 UBI 卷可用空间

## 9. 总结

本仓库的 XR1710G 方案不是“独立网页升级程序”，而是把 Web 恢复完整嵌入 U-Boot：

1. 配置层启用 lwIP + HTTPD_RECOVERY
2. 板级层提供恢复按键、MAC/factory 同步
3. 启动层提供启动失败回退到 Web 恢复
4. 恢复层实现上传、状态、刷写和自动重启闭环

这使 XR1710G 在量产与现场维护中可通过统一的 U-Boot 恢复路径完成固件和引导器修复。
