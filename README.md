# naiveslam
simple slam implementetion for rostselmash job interview
With my gratitude to geohot and old new-terra team 
###############################################################################
INSTALL: 

requirements: ubuntu 18.04 or similar, python3

sudo apt update -y
sudo apt upgrade -y
sudo apt install build-essential git cmake cmake-qt-gui libeigen3-dev python3-dev python3-pip -y

############################################################################### 
git clone and follow instructions: 
https://github.com/stevenlovegrove/Pangolin 
https://github.com/RainerKuemmerle/g2o

pip3 install opencv-python pip3 install vidgear pip3 install pygame

...

usage:

-F 500 ./slam.py videos/[videoname]
-F 500 ./liveslam.py

###############################################################################

Чтобы выбрать камеру - в VideoCapture(index) меняем индекс от 0

-F фокусное расстояние камеры, можно указать в переменных окружения вместо использования в команде

Классы
Frame -- Изображения с ORB фичами
Point -- 3-D точки на карте и их соответствия на кадре
Map -- Коллекция из Point и Frame
Display2D -- Для отображения текущего кадра, возможно придется поставить SDL2
Display3D -- Pangolin Для отображния 3d облака фич
