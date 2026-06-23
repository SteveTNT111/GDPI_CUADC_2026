# Claude 提示词与 n8n 流程可视化

## 当前真正生效的提示词/规则文件

### 1. 主动提醒系统提示词

- 路径：
  - `D:\文档\14_OBSIDIAN智能数据综合管理系统\02_知识库\零1-2N智能协作决策中枢\system\feishu_reminder_skill.md`
- 用途：
  - 给 Claude 规定“主动提醒怎么写”
  - 规定语气、长度、是否允许模板化、是否要区分拍板/委派/拉资源

### 2. 对话回复系统提示词

- 路径：
  - `D:\文档\14_OBSIDIAN智能数据综合管理系统\02_知识库\零1-2N智能协作决策中枢\system\feishu_reply_skill.md`
- 用途：
  - 给 Claude 规定“收到飞书消息后怎么回复”

### 3. 当前焦点覆盖

- 路径：
  - `D:\文档\14_OBSIDIAN智能数据综合管理系统\02_知识库\零1-2N智能协作决策中枢\system\07_focus_override.md`
- 用途：
  - 这是最高优先级的“现在到底该盯什么”
  - 用来强制提醒不要跑偏

### 4. 团队协作规则

- 路径：
  - `D:\文档\14_OBSIDIAN智能数据综合管理系统\02_知识库\零1-2N智能协作决策中枢\system\08_team_delegation_rules.md`
- 用途：
  - 规定遇到团队问题时，Claude 必须按“拍板 / 委派 / 拉资源”来写

## 定时提醒时的真实执行流程

1. `n8n` 的定时工作流触发
2. 工作流执行脚本：
   - `D:\n8n\workflows\scripts\zero12n_reminder.js`
3. 这个脚本调用本地桥接服务：
   - `D:\文档\14_OBSIDIAN智能数据综合管理系统\02_知识库\零1-2N智能协作决策中枢\system\claude_bridge_server.js`
4. 桥接服务开始组装“任务入口”
5. 读取系统提示词与规则文件
6. 生成：
   - `live_signals`
   - `web_signals`
   - `priority_files`
   - `search_roots`
   - `search_hints`
7. 桥接层把这些入口信息交给 Claude，但不再预读大段本地文件正文
8. Claude 在拥有本地目录读取权限的前提下，自行读取相关本地文件
9. Claude 输出提醒正文
10. `n8n` 再把正文发到飞书

## 目前本地提醒系统主要会读哪些内容

说明：

- 现在的设计目标是“桥接服务少做、Claude 多读”
- 也就是说，桥接负责告诉 Claude 去哪里找、重点看什么
- 真正的大段本地资料读取，尽量交给 Claude 自己完成

### 系统规则

- `feishu_reminder_skill.md`
- `feishu_reply_skill.md`
- `07_focus_override.md`
- `08_team_delegation_rules.md`

### 零1-2N 中枢主文件

- `00_inbox.md`
- `01_events.md`
- `02_today.md`
- `03_projects.md`
- `04_habits.md`
- `05_profile.md`
- `logs\`
- `summaries\`

### 你的本地项目资料

- 当前已接入：
  - `D:\文档\14_OBSIDIAN智能数据综合管理系统\02_知识库\项目资料\项目资料_CUADC_2026\`

### 你的个人语料

- 自述日志
- QQ 聊天记录导出
- 元宝对话
- ChatGPT 对话
- 看番清单
- 小米笔记同步内容

## 这份文件的作用

- 你以后想改提醒口径，就优先看这里列出来的四份规则文件
- 你以后想让 Claude 更懂项目进度，就往 `项目资料_CUADC_2026` 里补
- 你以后想让提醒更像你自己，就继续往本地日志和聊天记录里沉淀内容
