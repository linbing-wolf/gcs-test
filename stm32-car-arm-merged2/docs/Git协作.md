# Git协作

## 获取当前程序

在准备放项目的目录打开终端：

```powershell
git clone https://github.com/linbing-wolf/gcs-test.git
cd gcs-test
git status
```

先读 `stm32-car-test/docs/队友交接.md`，再打开 `stm32-car-test/keil/TableNavigation.uvprojx`。不要用旧压缩包覆盖克隆目录。当前开发机器也直接在这个仓库工作，无须再建第二份程序副本。

## 每次开发

未提交改动先检查并保存；如果 `pull --ff-only` 因本地修改或分叉拒绝，先看差异，与队友协调，不强推、不重置丢弃文件。

```powershell
git switch main
git pull --ff-only origin main
git switch -c fix/具体问题
```

只改本次问题，按交接说明验证。提交前查看状态和差异，逐个添加确定要提交的文件，例如：

```powershell
git status --short
git diff -- stm32-car-test/src/car_config.h
git add -- stm32-car-test/src/car_config.h
git diff --cached --stat
git diff --cached
git commit -m "调整横移控制参数并记录实测结果"
git push -u origin fix/具体问题
```

在GitHub打开这个分支，发起到main的Pull Request，由队友核对改动和验证结果后合并。上面的命令是协作示例，提交内容必须与实际修改一致；不要执行无差别 `git add -A` 把其他项目一起带入。

## 跟踪范围和固件

- 跟踪当前 `stm32-car-test/` 的源码、Keil工程、随包驱动、查表、文档、记录和验证证据；保留原仓库 `文档/`、`附件/` 及提交历史。
- 根目录其他本地研究、硬件、串口屏、旧交接包和工具目录仅留本机，尚未纳入本轮Git管理。后续需要协作这些内容时，另行确定范围并调整 `.gitignore`。
- Keil OBJ只保留交付用 `table_navigation.axf/.hex/.map` 三个产物，其他编译中间文件、临时测试可执行文件和Keil界面个人设置忽略。现有 `.uvoptx` 是工程配置，修改前检查是否影响队友的编译/下载设置。
- `.gitattributes`对程序目录禁用Git行尾转换，保留原始字节，保证清单SHA256跨电脑一致；不是要求全项目改行尾。
- `.env`、`auth.json`、私钥等凭据按文件名忽略；不要将凭据加入仓库或写入文档。GitHub认证用自己的Git凭据管理器或官方登录流程，不在聊天里发令牌。

程序修改后，应在交付说明中记录验证结果，重新构建相应AXF/HEX/MAP并核对一致性；验证完再更新 `package-manifest.json` 中相应文件哈希。清单本身不记录自己的哈希。新增需要随包交接的文件也应加入清单；测试临时输出不加入。不要仅刷新所有哈希把未验证修改包装成已验证版本。

## 查版本和报告问题

```powershell
git rev-parse --short HEAD
git log -5 --oneline
```

反馈问题请附提交号、实际烧录版本、起点A/C、发出的命令、从启动到故障的完整串口日志，以及实际碰撞/偏差发生的位置。实测记录放 `records/`，注明地面、负载、速度和测量方法。

本次接入以原仓库main为父提交，只追加当前程序和交接资料，未重写远程历史。
