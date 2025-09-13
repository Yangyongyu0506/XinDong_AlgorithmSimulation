#include <stdint.h>
#include <math.h>
#include <stdio.h>

#define CAM_IMAGE_WIDTH 188
#define CAM_IMAGE_HEIGHT 120

volatile float srv_out = 0.f;
volatile int prev_wdiff = 0;

int max(int *arr, int n) {
    int m = arr[0];
    for(int i = 1;i < n;++i){
        if(arr[i] > m) m = arr[i];
    }
    return m;
}

int min(int *arr, int n) {
    int m = arr[0];
    for(int i = 1;i < n;++i){
        if(arr[i] < m) m = arr[i];
    }
    return m;
}

int avg(int *arr, int n) {
    int sum = 0;
    for(int i = 0;i < n;++i){
        sum += arr[i];
    }
    return sum / n;
}
// 判断是否为左端环岛
int Is_lra(uint8_t img[CAM_IMAGE_HEIGHT][CAM_IMAGE_WIDTH]) {
    int Ys[][3] ={
        {100, 97, 103}, 
        {60, 57, 63}, 
        {20, 17, 23}
    };
    int countw[3][3] = {0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int x = CAM_IMAGE_WIDTH / 2; x >= 0; --x) {
                if (img[Ys[i][j]][x] > 100) {
                    ++countw[i][j];
                }
                if (img[Ys[i][j]][x] > 100) {
                    break;
                }
            }
        }
    }
    int wu, wm, wd;
    wu = avg(countw[2], 3);
    wm = avg(countw[1], 3);
    wd = avg(countw[0], 3);

    if ((wd - wm > 0) && (wu - wm > 0) && (wm > 65)) {
        printf("Is_lra: %d, %d, %d\n", wd, wm, wu);
        return 1;
    }
    return 0;
}

// 主要循迹算法
float track(uint8_t img[CAM_IMAGE_HEIGHT][CAM_IMAGE_WIDTH]) {
    if (Is_lra(img)) {
        return 1.0f;
    }
    int wcountl = 0;
    int wcountr = 0;
    for(int i = 60;i < CAM_IMAGE_HEIGHT;++i){
        for(int j = 40;j < CAM_IMAGE_WIDTH - 40; ++j){
            uint8_t gray = (uint8_t)img[i][j];
            if(gray > 100){
                if(j < CAM_IMAGE_WIDTH/2){
                    wcountl += i * i;
                }else{
                    wcountr += i * i;
                }
            }
        }
    }
    srv_out = ((1.31 * (float)(wcountl - wcountr) + 0.38 * (wcountl - wcountr - prev_wdiff)) / (wcountl + wcountr + 1));
    prev_wdiff = (wcountl - wcountr);
    return srv_out;
}


//=================== 环岛V7完整识别代码 ===================
// 包含OLED显示头文件
#include "Display.h"

//V7
int Test = 0;
int flagg = 0; // 环岛状态标志
if (flagg == 0)
{
    int count_left[60] = { 0 };
    int number_left = 0;
    
    // 左侧边界复杂度检测
    for (int j = 0; j < IMAGEW * 0.3; j++)
    {
        count_left[j] = 0;
        for (int i = 5; i < IMAGEH * 0.6; i++)
        {
            if (binary[i][j] != binary[i - 1][j])
            {
                int wrong = 0;
                int flag_color = binary[i - 1][j];
                for (int k = 1; k < 6; k++)
                {
                    if (binary[i - k][j] != flag_color && binary[i + k][j] == flag_color)
                    {
                        wrong = 1;
                        break;
                    }
                }
                if (wrong == 0) count_left[j]++;
            }
        }
        if (count_left[j] >= 3) number_left++;
    }

    // 右线判断
    int count_right;
    int number_right = 0;

    for (int j = IMAGEW - 1; j > IMAGEW * 0.55 - 1; j--)
    {
        count_right = 0;
        for (int i = 5; i < IMAGEH * 0.6; i++)
        {
            if (binary[i][j] != binary[i - 1][j])
            {
                int wrong = 0;
                int flag_color = binary[i - 1][j];
                for (int k = 1; k < 6; k++)
                {
                    if (binary[i - k][j] != flag_color && binary[i + k][j] == flag_color)
                    {
                        wrong = 1;
                        break;
                    }
                }
                if (wrong == 0) count_right++;
            }
        }
        if (count_right == 1) number_right++;
    }

    // OLED显示环岛检测数据
    displayRoundaboutData(number_left, number_right);

    // 环岛入口判断
    if (number_left > 15 && number_right > 45) // 参数待调
    {
        flagg++; 
        Test += 1;
        
        // 显示环岛检测成功信息
        displayRingDetected();
        
        // 可选的立即响应动作（当前被注释）
        // myduty=0;
        // motor_duty(myduty);
        // delay_ms(4000);
        // myangle=-100;
        // steer_angle(myangle);
        // myduty=12;
        // motor_duty(myduty);
        // delay_ms(1000);
    }
}

// 出环岛检测
int blacknumber = 0;
for (int i = 0; i < 20; i++)
{
    for (int j = 0; j <= IMAGEW; j++)
    {
        if (binary[i][j] == 0) blacknumber++;
    }
}
if (blacknumber > 3500)
{
    // 显示准备出环岛信息
    displayExitRing();
    
    // if(前期判断为左环岛){
    //     steer_angle(-80);motor_duty(12);delay_ms(1000);
    // }
    // else {steer_angle(80);motor_duty(12);delay_ms(1000);}
    flagg++; 
    Test += 3;
}

// ================= OLED显示相关函数 =================

// 主要的OLED显示函数 - 显示环岛检测数据
void displayRoundaboutData(int number_left, int number_right) {
    // 清除OLED屏幕指定区域（避免重影）
    OLED_ClearArea(0, 0, 128, 64);
    
    // 第1行：显示标题
    OLED_ShowString(0, 0, "Round Detect:", OLED_8X16);
    
    // 第2行：显示左侧检测数据
    OLED_ShowString(0, 16, "Left:", OLED_8X16);
    OLED_ShowNum(40, 16, number_left, 3, OLED_8X16);
    
    // 第3行：显示右侧检测数据
    OLED_ShowString(0, 32, "Right:", OLED_8X16);
    OLED_ShowNum(48, 32, number_right, 3, OLED_8X16);
    
    // 第4行：显示环岛检测状态
    if (number_left > 15 && number_right > 45) {
        OLED_ShowString(0, 48, "Status: RING!", OLED_8X16);
    } else {
        OLED_ShowString(0, 48, "Status: Normal", OLED_8X16);
    }
    
    // 更新OLED显示缓冲区到屏幕
    OLED_Update();
}

// 显示详细调试信息的函数
void displayDebugInfo(int number_left, int number_right, int flagg_state, int test_value) {
    // 清屏
    OLED_Clear();
    
    // 使用Printf函数显示格式化信息（更紧凑）
    OLED_Printf(0, 0, OLED_6X8, "L:%d R:%d", number_left, number_right);
    OLED_Printf(0, 10, OLED_6X8, "Flag:%d Test:%d", flagg_state, test_value);
    
    // 显示阈值信息
    OLED_ShowString(0, 20, "Threshold: L>15", OLED_6X8);
    OLED_ShowString(0, 30, "           R>45", OLED_6X8);
    
    // 显示当前状态
    if (flagg_state > 0) {
        OLED_ShowString(0, 40, "ROUNDABOUT MODE", OLED_6X8);
    } else {
        OLED_ShowString(0, 40, "NORMAL MODE", OLED_6X8);
    }
    
    // 显示实时数据条形图（可选）
    OLED_ShowString(0, 50, "L:", OLED_6X8);
    for (int i = 0; i < number_left && i < 20; i++) {
        OLED_DrawPoint(12 + i, 52);
        OLED_DrawPoint(12 + i, 53);
    }
    
    OLED_ShowString(0, 56, "R:", OLED_6X8);
    for (int i = 0; i < number_right && i < 20; i++) {
        OLED_DrawPoint(12 + i, 58);
        OLED_DrawPoint(12 + i, 59);
    }
    
    OLED_Update();
}

// 初始化OLED显示的函数
void initRoundaboutDisplay() {
    // 初始化OLED硬件
    OLED_Init();
    
    // 清除整个屏幕
    OLED_Clear();
    
    // 显示启动信息（中英文混合）
    OLED_ShowChinese(16, 0, "环岛");
    OLED_ShowString(48, 0, " V7", OLED_8X16);
    OLED_ShowChinese(16, 16, "检测器");
    OLED_ShowChinese(16, 32, "准备");
    OLED_ShowChinese(48, 32, "完毕");
    
    // 更新显示
    OLED_Update();
    
    // 显示启动信息2秒
    delay_ms(2000);
}

// 显示环岛检测成功的动画效果
void displayRingDetected() {
    // 清屏
    OLED_Clear();
    
    // 显示检测成功信息
    OLED_ShowChinese(32, 8, "检测");
    OLED_ShowChinese(64, 8, "到");
    OLED_ShowChinese(32, 24, "环岛");
    
    // 添加一个简单的边框效果
    OLED_DrawRectangle(8, 4, 112, 40, OLED_UNFILLED);
    OLED_DrawRectangle(6, 2, 116, 44, OLED_UNFILLED);
    
    OLED_Update();
    
    // 显示1秒
    delay_ms(1000);
}

// 显示准备出环岛的信息
void displayExitRing() {
    OLED_Clear();
    OLED_ShowChinese(24, 16, "准备");
    OLED_ShowChinese(56, 16, "出");
    OLED_ShowChinese(24, 32, "环岛");
    
    // 添加箭头指示
    OLED_ShowString(88, 24, "->", OLED_8X16);
    
    OLED_Update();
    delay_ms(800);
}