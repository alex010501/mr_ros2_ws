## Курс по предмету Автономные мобильные роботы в МГТУ им Н.Э.Баумана

Здесь расположены условия задач и их решения на ROS2 лабораторных работ по предмету "Автономные мобильные роботы"

Сборка пакета:
```bash
# Клонируем репозиторий с подмодулями
git clone --recurse-submodules https://github.com/alex010501/mr_ros2_ws.git
# Заходим в репозиторий
cd mr_ros2_ws
# Устанавливаем зависимости
sudo bash ./install_dependencies.sh
```

Задачи:
1. [Контроллер управления движением по траектории](./src/simple_controller/readme.md)
2. [Контроллер управления скоростью](./src/velocity_controller/readme.md)
3. [MPC контроллер](./src/mpc_controller/readme.md)
4. [Планирование траектории](./src/trajectory_planner/readme.md)
5. [Построение карты](./src/mapping/readme.md)
6. [Локализация по дальномеру](./src/localization/readme.md)
7. [EKF SLAM](./src/ekf_slam/readme.md)



## Autonomous mobile robots course in BMSTU

Here are the conditions of tasks and their solutions for ROS2 of laboratory works on the subject "Autonomous mobile robots"

Building a package:
```bash
# Cloning the repository with submodules
git clone --recurse-submodules https://github.com/alex010501/mr_ros2_ws.git
# Entering the repository
cd mr_ros2_ws
# Installing dependencies
sudo bash ./install_dependencies.sh
```

Tasks:
1. [Trajectory Motion Control Controller](./src/simple_controller/readme.md)
2. [Speed Control controller](./src/velocity_controller/readme.md)
3. [MPC controller](./src/mpc_controller/readme.md)
4. [Trajectory Planning](./src/trajectory_planner/readme.md)
5. [Building a map](./src/mapping/readme.md)
6. [Localization by rangefinder](./src/localization/readme.md)
7. [EKF SLAM](./src/ekf_slam/readme.md)