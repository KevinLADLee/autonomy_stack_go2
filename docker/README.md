# Docker 部署文件

本文件夹包含所有Docker相关的配置文件。

## 文件说明

- **Dockerfile** - 生产环境Dockerfile，在构建镜像时编译整个工作空间
- **Dockerfile.dev** - 开发环境Dockerfile，在容器内编译（适合快速迭代）
- **docker-compose.yml** - Docker Compose配置文件
- **docker-entrypoint.sh** - 容器入口脚本，自动设置ROS2环境
- **DOCKER_README.md** - 详细的部署文档（中文）

## 快速开始

从项目根目录运行：

```bash
# 准备环境
./docker-setup.sh

# 构建并启动
./docker-run.sh rebuild

# 进入容器
./docker-run.sh shell
```

更多信息请参考 [DOCKER_README.md](./DOCKER_README.md)

