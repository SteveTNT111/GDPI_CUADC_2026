把 YOLOv8 权重文件 best.pt 放到这个目录下。

运行时 launch 文件会自动解析为包内路径 $(find cuadc_vision)/models/best.pt，
无需再手动指定绝对路径。

⚠️ best.pt 不纳入 Git 版本管理（.gitignore），防止其他队伍直接克隆跑通。
需要在 NUC 上手动放入此文件。
