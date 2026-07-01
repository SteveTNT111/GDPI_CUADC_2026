# 队员 GitHub + VS Code 操作指南

> 所有人今天完成。不会的在群里问。

---

## 第一步：浏览器装油猴 + GitHub 中文化（5 分钟）

### 1.1 安装 Tampermonkey（油猴）

用 Edge 或 Chrome 浏览器打开：https://www.tampermonkey.net/ → 点"下载" → 安装扩展。

### 1.2 安装两个 GitHub 翻译脚本

打开下面两个链接，各点一次"安装此脚本"：

- https://greasyfork.org/zh-CN/scripts/435208-github-%E4%B8%AD%E6%96%87%E5%8C%96%E6%8F%92%E4%BB%B6
- https://greasyfork.org/zh-CN/scripts/407485-github-internationalization

两个都要装，装完后 GitHub 才是全中文。

### 1.3 注册 GitHub 账号

打开 https://github.com → 右上角"注册" → 填邮箱、密码、用户名 → 验证邮箱。

**然后把你的 GitHub 用户名发给伍尚京。**

---

## 第二步：VS Code 克隆仓库（2 分钟）

### 2.1 打开 VS Code

按 `Ctrl+Shift+P` → 输入 `git clone` → 回车 → 粘贴仓库地址：

```
https://github.com/SteveTNT111/GDPI_CUADC_2026.git
```

→ 选择保存位置（放 D 盘根目录就行）→ 克隆完成后右下角点"打开"。

---

## 第三步：日常工作流程（每天 30 秒）

### 每天开工

按 `Ctrl+Shift+P` → 输入 `git pull` → 回车。拉取队友的最新改动。

### 每天收工

1. 把你的代码和说明文件放进 `代码/你的名字/` 下面
2. `Ctrl+Shift+G` 打开源代码管理面板
3. **只点你自己文件的 `+` 号**（不要点别人的文件，不要点 `全部暂存`）
4. 在上方消息框写一行说明，比如 `陈智勇：桶检测脚本更新`
5. 点消息框上面的 `✓` 提交
6. 点左下角 `...` → `推送`

---

## 注意

- **cuadc_src 文件夹不要碰**——那是主程序，伍尚京在管
- **不要点 `全部暂存`**——只提交你自己文件夹里的东西
- **代码文件必须配同名 .md 说明**——至少写清楚这个脚本干什么、怎么跑
- **在自己电脑上测通了再上传**

---

> 以上全部操作不需要命令行，全程鼠标点击即可。
>
> GitHub 是中文的，VS Code 有快捷键提示，放心操作。
