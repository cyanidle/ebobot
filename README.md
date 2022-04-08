# ebobot

Чрезвычайно продвинутые технологии БФУ.
Все работает с первого раза, как и должно
:100:


# Настройка
Все настройки содержатся в папке **config**. Перед запуском экспортируйте нужный маршрут в EBOBOT_ROUTE
(**export EBOBOT_ROUTE1/EBOBOT_TEST_ROUTE1 my_route(без расширения .yaml)**), по умолчанию test_route.yaml.
Для прошивки ардуино следует скинуть библиотеки из папки **firmware** в корень библиотек ArduinoIDE
Установите ros-noetic-rosserial-arduino

# Папки:
# temp
Файлопомойка

# config
Содержит настройки для маршрута, изображения карты и рвиза, параметры для скриптов и нод

# src
Библиотеки питона и исполняемые файлы Си

# scripts
Исполняемые скрипты, отрабатывающие один раз

# nodes
Папка для нод на питоне (постоянно исполняемый скрипт)

# msg/srv/action
Сообщения/Сервисы/Экшоны соответственно

# launch
**Скрипты системы Roslaunch**
Запуск навигации roslaunch ebobot move_base.launch
Запустить rviz с оптимальными настройками можно через
*roslaunch ebobot rviz.launch*


# firmware
Хранит скетч и библиотеки, а также **install.sh**, который нужно выполнить через *bash*
При выполнении скопирует все библиотеки и зальет прошивку на указанный порт (по умолчанию /dev/ttyACM0)

