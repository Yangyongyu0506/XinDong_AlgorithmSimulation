# XinDong_AlgorithmSimulation
## 使用方法
首先在asset/track.c中定义循迹算法函数，函数原型为：

    float track(uint8_t img[CAM_IMAGE_HEIGHT][CAM_IMAGE_WIDTH])

也即接收的入参是一个CAM_IMAGE_HEIGHT * CAM_IMAGE_WIDTH大小的uint8_t型数组，返回值是一个属于[-1, 1]的浮点数（归一化后的舵机输出值）。然后将track.c编译为Windows动态库文件（.dll）放在asset目录下。最后在XinDong_AlgorithmSimulation目录下运行WayFinder.py文件即可。
  
运行后会显示出小车在赛道俯视图上的实时动画。小车是红色矩形，小车所看到的地面范围用一个绿色梯形框了出来。左上角贴出了小车看到的视野经过透视变换后的矩形画面，有助于结合分析实际场景和摄像头画面的关系。按q键即可退出程序，如果小车走出了画面范围程序也会终止。
## 文件说明
  WayFinder.py：运行仿真的主要脚本文件；
  
  asset/track.c：在其中定义循迹函数float track(uint8_t img[CAM_IMAGE_HEIGHT][CAM_IMAGE_WIDTH])。这里用C语言定义循迹算法而非Python是为了能够直接向单片机上移植；
  
  asset/track.dll：由asset/track.c编译所得的Windows动态库文件。WayFinder.py调用的就是这里面的函数；
  
  asset/map.png：这是赛道的俯视图（我自己手画的）。
  
## 改进方向
  这个仿真没有暂停看动态画面的功能。
## 编译为动态库方法
法1：在XinDong_AlgorithmSimulation目录下打开cmd，输入：

    gcc -shared -o asset/track.dll -fPIC asset/track.c
  
法2：在XinDong_AlgorithmSimulation目录下打开powershell，输入：

    gcc -shared -o 'asset/track.dll' 'asset/track.c'
