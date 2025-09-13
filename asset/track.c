#include <stdint.h>

// 摄像头画面分辨率宏定义，根据实际情况调整
#define CAM_IMAGE_WIDTH 188
#define CAM_IMAGE_HEIGHT 120

volatile float srv_out = 0.f; // 定义舵机输出值
volatile int prev_wdiff = 0; // 定义前一次左右白点插值

// 循迹算法函数
// 这是我们使用的算法，仅作为示例
float track(uint8_t img[CAM_IMAGE_HEIGHT][CAM_IMAGE_WIDTH]) { // 函数名，形参和返回值不要改，否则WayFinder.py识别不了算法函数
    /*
    img[CAM_IMAGE_HEIGHT][CAM_IMAGE_WIDTH]是摄像头读取的YUV编码图像的Y（亮度）通道，约等于灰度图
        */
    int wcountl = 0; // 左侧白值
    int wcountr = 0; // 右侧白值
    int IsWhiteThreshold = 100; // 判定为白色像素的阈值
    float Kp = 1.31; // 调控舵机输出的P系数
    float Kd = 0.38; // D系数。两个参数需要自己试
    for(int i = 60;i < CAM_IMAGE_HEIGHT;++i){ // 我们只考察图像下半部分
        for(int j = 40;j < CAM_IMAGE_WIDTH - 40; ++j){ // 我们只考察中间108列
            uint8_t gray = (uint8_t)img[i][j]; // 这里对img[i][j]进行了强制类型转换，这是因为比赛时图像是uint16_t型的数组，非常抽象
            if(gray > IsWhiteThreshold){ // 这里为了加快处理速度没有单独做二值化
                if(j < CAM_IMAGE_WIDTH/2){ // 判断白色像素的位置
                    wcountl += i * i; // 这里不是简单的加1，而是根据所处的行数乘以一定权重在加1.这里选取的权重为i * i，是为了加强离车身近的白色部分的重要性
                }else{
                    wcountr += i * i;
                }
            }
        }
    }
    /*
    这里令左右白值差分为我们要保持为0的变量，对它进行PD控制。求和后除以左右白值之和+1进行归一化。+1是防止除0错误
        */
    srv_out = ((Kp * (float)(wcountl - wcountr) + Kd * (wcountl - wcountr - prev_wdiff)) / (wcountl + wcountr + 1)); // 舵机的PD控制算法
    prev_wdiff = (wcountl - wcountr); // 更新上一次的左右白值差分
    return srv_out;
}
