# 上传到 Gitee 步骤

## 1. 准备本地配置

在 `firmware/Final/` 目录下：

```bash
copy secrets.h.example secrets.h
```

编辑 `secrets.h`，填入你的 Wi-Fi 名称和密码。

## 2. 初始化 Git 仓库

在项目根目录 `esp32-pet-feeder/` 执行：

```bash
git init
git add .
git status
git commit -m "Initial commit: ESP32-S3 smart pet feeder"
```

确认 `git status` 中**没有**出现 `secrets.h`。

## 3. 在 Gitee 创建仓库

1. 登录 https://gitee.com
2. 点击「新建仓库」
3. 仓库名建议：`esp32-pet-feeder`
4. 不要勾选「使用 Readme 文件初始化」（避免冲突）

## 4. 推送代码

```bash
git remote add origin https://gitee.com/你的用户名/esp32-pet-feeder.git
git branch -M main
git push -u origin main
```

## 5. 以后更新

```bash
git add .
git commit -m "描述你的修改"
git push
```

## 注意事项

- **不要提交** `secrets.h`（已在 `.gitignore` 中）
- **不要提交** Edge Impulse 模型库（体积大，在 README 中说明如何导出）
- 若以后要同步到 GitHub，可再加一个 remote：

```bash
git remote add github https://github.com/你的用户名/esp32-pet-feeder.git
git push github main
```
