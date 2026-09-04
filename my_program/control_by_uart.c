#include "pinctrl.h"
#include "uart.h"
#include "osal_debug.h"
#include "soc_osal.h"
#include "app_init.h"
#include "string.h"
#include "cmsis_os2.h"
#include "tcxo.h"

#include "led.h"        // LED控制头文件
#include "motor.h"      // 电机控制头文件
#include "tcrt5000.h"   // 红外传感器头文件

// ============ 宏定义区 ============
// 电机控制引脚定义
#define LEFT_MOTOR_PWM 12
#define LEFT_MOTOR_CHANNEL 4
#define RIGHT_MOTOR_PWM 11
#define RIGHT_MOTOR_CHANNEL 3
#define PWM_MODE 1
#define LEFT_MOTOR_MOVE_OUT1 9
#define LEFT_MOTOR_MOVE_OUT2 10
#define RIGHT_MOTOR_MOVE_OUT1 7
#define RIGHT_MOTOR_MOVE_OUT2 8

#define PID_KP  100      // 比例系数
#define PID_KI  0.4     // 积分系数
#define PID_KD  50      // 微分系数
#define BASE_SPEED 400  // 基础速度

// PID 控制器变量
static float pid_integral = 0;     // 积分累计值
static float pid_last_error = 0;   // 上一次的偏差

// 红外巡线
#define TCRT5000_RIGHT_PIN   1   // GPIO1
#define TCRT5000_LEFT_PIN   2   // GPIO2
#define GPIO_MODE 0             // GPIO模式

// 串口通信
#define DELAY_TIME_MS 5           // 延时时间（毫秒）
#define UART_RECV_SIZE 32         // 串口接收缓冲区大小（增大一点）
#define UART_TASK_STACK_SIZE     0x1000 // 串口任务堆栈大小
#define MOTOR_TASK_STACK_SIZE    0x1000 // 电机任务堆栈大小
#define UART_TASK_PRIO           24    // 串口任务优先级

#define MOTOR_TASK_PRIO          23    // 电机任务优先级（略高于UART）

// LED灯（可改为蜂鸣器）
#define BLINKY_PIN 6              // LED连接的GPIO引脚

// 命令类型枚举
typedef enum {
    CMD_NONE = 0,
    CMD_SPACE_MOVE,       // 安全模式（巡线）1
    CMD_UNSAFE,     // 不安全模式（停止）2
    CMD_ON,         // 开灯 3 
    CMD_OFF,        // 关灯 4
    CMD_UPSTAIR_MOVE,    //上楼梯 5
    CMD_DOWMSTAIR_MOVE,  //下楼梯  6
    CMD_UNKNOWN     // 未知命令
} cmd_type_t;

// 消息队列结构体
typedef struct {
    cmd_type_t cmd;             // 命令类型
    uint8_t data[16];           // 附加数据（备用）
} motor_msg_t;

// ============ 全局变量区 ============
uint8_t uart_recv[UART_RECV_SIZE] = {0};  // 串口接收数据缓冲区
osMessageQueueId_t motor_msg_queue = NULL; // 消息队列句柄
volatile cmd_type_t current_mode = CMD_NONE;

#define UART_INT_MODE 1           // 定义串口工作模式：1=中断模式，0=轮询模式
#if (UART_INT_MODE)
static uint8_t uart_rx_flag = 0;  // 串口接收完成标志位
#endif

// 串口缓冲区配置结构体
uart_buffer_config_t g_app_uart_buffer_config = {.rx_buffer = uart_recv, .rx_buffer_size = UART_RECV_SIZE};

// ============ 辅助函数 ============
/**
 * @brief 将字符串命令转换为枚举类型
 */

cmd_type_t parse_command(uint8_t *buf, uint16_t len)//读取串口调试助手发送的信息
{
    // 去除末尾的换行符
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\r' || buf[i] == '\n') {
            buf[i] = '\0';
            break;
        }
    }
    
    if (strcmp((char *)buf, "space_move") == 0)   return CMD_SPACE_MOVE;
    if (strcmp((char *)buf, "up_stair") == 0)   return CMD_UPSTAIR_MOVE;
    if (strcmp((char *)buf, "dowm_stair") == 0)   return CMD_DOWMSTAIR_MOVE;
    if (strcmp((char *)buf, "unsafe") == 0) return CMD_UNSAFE;
    if (strcmp((char *)buf, "on") == 0)     return CMD_ON;
    if (strcmp((char *)buf, "off") == 0)    return CMD_OFF;
    
    return CMD_UNKNOWN;
}

// ============ UART 模块 ============
/**
 * @brief 初始化UART的GPIO引脚
 */
void uart_gpio_init(void)
{
    uapi_pin_set_mode(GPIO_17, PIN_MODE_1);  // 设置GPIO_17为UART TX功能
}

/**
 * @brief 初始化UART配置
 */
void uart_init_config(void)
{
    uart_attr_t attr = {
        .baud_rate = 115200,//波特率115200
        .data_bits = UART_DATA_BIT_8, // 8位数据位
        .stop_bits = UART_STOP_BIT_1,  // 1位停止位
        .parity = UART_PARITY_NONE,  // 无校验
    };

    // 配置UART引脚
    uart_pin_config_t pin_config = {
        .tx_pin = GPIO_17,    // 发送引脚GPIO_17
        .rx_pin = GPIO_18,    // 接收引脚GPIO_18
        .cts_pin = PIN_NONE,  // 不使用硬件流控CTS
        .rts_pin = PIN_NONE,  // 不使用硬件流控RTS
    };
    
    uapi_uart_deinit(UART_BUS_0);  // 先反初始化串口0
    // 初始化串口0，参数：串口号、引脚配置、属性配置、发送回调、接收缓冲区配置
    int ret = uapi_uart_init(UART_BUS_0, &pin_config, &attr, NULL, &g_app_uart_buffer_config);
    if (ret != 0) {
        osal_printk("uart init failed ret = %02x\n", ret);  // 初始化失败打印错误码
    }
}

#if (UART_INT_MODE)
/**
 * @brief 串口接收中断回调函数
 */
void uart_read_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    osal_printk("请选择：space_move,up_stair,dowm_stair,unsafe,on,off\n");
    
    if (buffer == NULL || length == 0) {
        osal_printk("接收数据无效\n");
        return;
    }
    
    // 将接收到的数据复制到uart_recv缓冲区
    if (memcpy_s(uart_recv, length, buffer, length) != EOK) {
        osal_printk("数据复制失败\n");
        return;
    }
    
    if (length < UART_RECV_SIZE) {
        uart_recv[length] = '\0';
    } else {
        uart_recv[UART_RECV_SIZE - 1] = '\0';
    }
    
    osal_printk("接收数据成功，长度 = %d\n", length);
    uart_rx_flag = 1;  // 设置接收完成标志
}
#endif

/**
 * @brief UART任务主函数 - 只负责接收命令并发送到消息队列
 */
void *uart_task(const char *arg)
{
    unused(arg);
    
    // 初始化UART
    uart_gpio_init();
    uart_init_config();

#if (UART_INT_MODE)
    // 注册串口接收回调函数
    if (uapi_uart_register_rx_callback(0, UART_RX_CONDITION_MASK_IDLE, 1, uart_read_handler) == ERRCODE_SUCC) {
        osal_printk("UART回调注册成功\n");
    }
#endif

    osal_printk("UART任务已启动，等待命令...\n");

    while (1) {
#if (UART_INT_MODE)
        // 等待接收完成标志
        while (!uart_rx_flag) {
            osDelay(DELAY_TIME_MS);
        }
        uart_rx_flag = 0;
        
        // 解析命令
        cmd_type_t cmd = parse_command(uart_recv, strlen((char *)uart_recv));
        
        // 构造消息并发送到电机线程
        motor_msg_t msg;
        msg.cmd = cmd;
        
        if (motor_msg_queue != NULL) {
            osStatus_t status = osMessageQueuePut(motor_msg_queue, &msg, 0, 0);
            if (status != osOK) {
                osal_printk("消息队列发送失败: %d\n", status);
            } else {
                osal_printk("已发送命令到电机线程: %d\n", cmd);
            }
        }
        
        // 清空接收缓冲区
        memset(uart_recv, 0, UART_RECV_SIZE);
#else
        // 轮询模式
        if (uapi_uart_read(UART_BUS_0, uart_recv, UART_RECV_SIZE, 0)) {
            cmd_type_t cmd = parse_command(uart_recv, strlen((char *)uart_recv));
            
            motor_msg_t msg;
            msg.cmd = cmd;
            
            if (motor_msg_queue != NULL) {
                osMessageQueuePut(motor_msg_queue, &msg, 0, 0);
            }
        }
#endif
    }
    return NULL;
}

// ============ 电机控制模块 ============
/**
 * @brief 初始化所有电机相关的硬件
 */
void motor_hardware_init(void)
{
    tcrt_config(TCRT5000_LEFT_PIN, GPIO_MODE);
    tcrt_config(TCRT5000_RIGHT_PIN, GPIO_MODE);
    motor_move_config(LEFT_MOTOR_MOVE_OUT1, GPIO_MODE);//电机驱动引脚初始化和配置
    motor_move_config(RIGHT_MOTOR_MOVE_OUT1, GPIO_MODE);//电机驱动引脚初始化和配置
    motor_move_config(LEFT_MOTOR_MOVE_OUT2, GPIO_MODE);//电机驱动引脚初始化和配置
    motor_move_config(RIGHT_MOTOR_MOVE_OUT2, GPIO_MODE);//电机驱动引脚初始化和配置
    uapi_pwm_init();
    uapi_pin_init();
    led_config(BLINKY_PIN, GPIO_MODE);     // 初始化引脚控制
}

/**
 * @brief 执行巡线前进
 */
int pid_calculate(float error)
{

    if (error == 0) {
        pid_integral *= 0.95;  
    }
    
    // 积分项（带限幅）
    pid_integral += ( error*20 );
    if (pid_integral > 250) pid_integral = 250;
    if (pid_integral < -250) pid_integral = -250;
    float p_out = PID_KP * error * 1.5;
    float i_out = PID_KI * pid_integral;
    
    // 微分项
    float d_out = PID_KD * (error - pid_last_error);
    
    // 保存本次偏差
    pid_last_error = error;
    
    // 总输出
    float output = p_out + i_out + d_out;
    
    // 输出限幅
    if (output > 300) output = 300;
    if (output < -300) output = -300;
    osal_printk("output:%d\tpid_integral:%d\tpid_last_error:%d\terror:%d\n",(int)output,(int)pid_integral,(int)pid_last_error,(int)error);
    return (int)output;
}

/**
 * @brief PID 巡线函数
 * 使用增量式 PID 控制，让巡线更平滑
 */
void line_following(void)
{
    int check_left = tcrt_check(TCRT5000_LEFT_PIN);
    int check_right = tcrt_check(TCRT5000_RIGHT_PIN);
    osal_printk("check_left=%d, check_right=%d\n",check_left,check_right);
    static int lost_count = 0;
    
    // 脱线检测保持不变
    if (check_left == 1 && check_right == 1) {
        lost_count++;
        if (lost_count > 50) {
            motor_stop(LEFT_MOTOR_MOVE_OUT1, LEFT_MOTOR_MOVE_OUT2,
                       RIGHT_MOTOR_MOVE_OUT1, RIGHT_MOTOR_MOVE_OUT2);
            osal_printk("脱线太久，停止\n");
            lost_count = 0;
            current_mode = CMD_NONE;
            return;
        }
    } else {
        lost_count = 0;
    }
    
    // 计算偏差
    float error = (float)(check_left - check_right);
    
    // 脱线时沿用上次方向
    if (check_left == 1 && check_right == 1) {
        error = pid_last_error;
        if (error == 0) error = 1;
    }
    
    // PID计算
    int pid_output = pid_calculate(error);
    osal_printk("pid_output=%d\n",pid_output);
    // 根据PID输出决定方向和速度
    if (pid_output > 200) {
        // 大幅左转：左轮反转，右轮正转
        motor_left_backward(LEFT_MOTOR_MOVE_OUT1, LEFT_MOTOR_MOVE_OUT2);
        motor_right_forward(RIGHT_MOTOR_MOVE_OUT1, RIGHT_MOTOR_MOVE_OUT2);
        pwm_control(LEFT_MOTOR_PWM, PWM_MODE, 300 - abs(pid_output),  abs(pid_output), LEFT_MOTOR_CHANNEL);
        pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, 300 - abs(pid_output), abs(pid_output), RIGHT_MOTOR_CHANNEL);
        osal_printk("急左转: out=%d\n", pid_output);
    }
    else if (pid_output < -200) {
        // 大幅右转：左轮正转，右轮反转
        motor_left_forward(LEFT_MOTOR_MOVE_OUT1, LEFT_MOTOR_MOVE_OUT2);
        motor_right_backward(RIGHT_MOTOR_MOVE_OUT1, RIGHT_MOTOR_MOVE_OUT2);
        pwm_control(LEFT_MOTOR_PWM, PWM_MODE, 300 - abs(pid_output), abs(pid_output), LEFT_MOTOR_CHANNEL);
        pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, 300 - abs(pid_output),  abs(pid_output), RIGHT_MOTOR_CHANNEL);
        osal_printk("急右转: out=%d\n", pid_output);
    }
    else {
        // 小偏差：差速前进
        int left_speed = BASE_SPEED - pid_output;
        int right_speed = BASE_SPEED + pid_output;
        
        // 限幅
        if (left_speed < 0) left_speed = 0;
        if (left_speed > 500) left_speed = 500;
        if (right_speed < 0) right_speed = 0;
        if (right_speed > 500) right_speed = 500;
        
        motor_left_forward(LEFT_MOTOR_MOVE_OUT1, LEFT_MOTOR_MOVE_OUT2);
        motor_right_forward(RIGHT_MOTOR_MOVE_OUT1, RIGHT_MOTOR_MOVE_OUT2);
        pwm_control(LEFT_MOTOR_PWM, PWM_MODE, 500 - left_speed, left_speed, LEFT_MOTOR_CHANNEL);
        pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, 500 - right_speed, right_speed, RIGHT_MOTOR_CHANNEL);
        
        osal_printk("差速: L=%d R=%d\n", left_speed, right_speed);
    }
    osDelay(200);
}

uint8_t up_stair(uint32_t start_time){
    uint8_t flag=0;
    uint16_t car_speed=250;
    uint32_t end_time=uapi_tcxo_get_us();
    uint32_t duration=(end_time-start_time)/1000000;
    motor_left_forward(LEFT_MOTOR_MOVE_OUT1,LEFT_MOTOR_MOVE_OUT2);
    motor_right_forward(RIGHT_MOTOR_MOVE_OUT1,RIGHT_MOTOR_MOVE_OUT2);
    if(duration<10){
        pwm_control(LEFT_MOTOR_PWM, PWM_MODE, car_speed, car_speed, LEFT_MOTOR_CHANNEL);
        pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, car_speed, car_speed, RIGHT_MOTOR_CHANNEL);
        flag=1;
        
        osal_printk("return flag1,消耗的时间为：%d",(int)duration);
        osDelay(100);
        return flag;
        
    }
    else if(duration>10&&duration<15){
        osal_printk("需要调整速度");
        pwm_control(LEFT_MOTOR_PWM, PWM_MODE, car_speed+100, car_speed-100, LEFT_MOTOR_CHANNEL);
        pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, car_speed+100, car_speed-100, RIGHT_MOTOR_CHANNEL);
        // motor_left_forward(LEFT_MOTOR_MOVE_OUT1,LEFT_MOTOR_MOVE_OUT2);
        // motor_right_forward(RIGHT_MOTOR_MOVE_OUT1,RIGHT_MOTOR_MOVE_OUT2);
        flag=2;
        
        osal_printk("return flag2,消耗的时间为:%d",(int)duration);
        osDelay(100);
        return flag;
        
    }
    else{
        osal_printk("无法上楼梯,开始减速，准备倒退，不继续上楼梯");
        for(car_speed=150;car_speed>0;car_speed-=20){
            pwm_control(LEFT_MOTOR_PWM, PWM_MODE, car_speed+100, car_speed-100, LEFT_MOTOR_CHANNEL);
            pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, car_speed+100, car_speed-100, RIGHT_MOTOR_CHANNEL);
            osDelay(50);
        }
        motor_left_backward(LEFT_MOTOR_MOVE_OUT1,LEFT_MOTOR_MOVE_OUT2);
        motor_right_backward(RIGHT_MOTOR_MOVE_OUT1,RIGHT_MOTOR_MOVE_OUT2);
        for(car_speed=0;car_speed<150;car_speed+=10){
            pwm_control(LEFT_MOTOR_PWM, PWM_MODE, 500-car_speed, car_speed, LEFT_MOTOR_CHANNEL);
            pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, 500-car_speed, car_speed, RIGHT_MOTOR_CHANNEL);
            osDelay(50);
        }
        osDelay(2000);
        osal_printk("已经倒退致平地,return flag3");

        flag=3;
        return flag;
    }
    return flag;

}

void dowm_stair(void){
    osal_printk("准备下楼梯");
    pwm_control(LEFT_MOTOR_PWM, PWM_MODE, 250, 250, LEFT_MOTOR_CHANNEL);
    pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, 250, 250, RIGHT_MOTOR_CHANNEL);
    motor_left_forward(LEFT_MOTOR_MOVE_OUT1,LEFT_MOTOR_MOVE_OUT2);
    motor_right_forward(RIGHT_MOTOR_MOVE_OUT1,RIGHT_MOTOR_MOVE_OUT2);
}

/**
 * @brief 重置 PID（切换模式时调用）
 */
void pid_reset(void)
{
    pid_integral = 0;
    pid_last_error = 0;
}

/**
 * @brief 执行停止动作
 */
void stop_motors(void)
{
    pwm_control(LEFT_MOTOR_PWM, PWM_MODE, 500, 0, LEFT_MOTOR_CHANNEL);
    pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, 500, 0, RIGHT_MOTOR_CHANNEL);
    motor_stop(LEFT_MOTOR_MOVE_OUT1, LEFT_MOTOR_MOVE_OUT2, 
               RIGHT_MOTOR_MOVE_OUT1, RIGHT_MOTOR_MOVE_OUT2);
    osal_printk("危险，停车");
    osDelay(10);
}

/**
 * @brief 电机控制任务主函数 - 从消息队列接收命令并执行
 */
void *motor_task(const char *arg)
{
    unused(arg);
    
    // 初始化所有硬件
    motor_hardware_init();
    
    osal_printk("电机任务已启动，等待命令...\n");

    while (1) {
        motor_msg_t msg;
        uint32_t start_time;//记录初始时间
        uint8_t mode_flag=0;//标记上楼梯的状态
        // 从消息队列接收命令（阻塞等待，超时时间为100ms）
        osStatus_t status = osMessageQueueGet(motor_msg_queue, &msg, NULL, 10);
        
        if (status == osOK) {
            // 收到新命令，更新当前模式
            current_mode = msg.cmd;
            osal_printk("切换到新模式: %d\n", current_mode);
        }
            // 收到有效命令
            switch (current_mode) {
                case CMD_SPACE_MOVE://平地巡线模式
                    pid_reset();
                    osal_printk("执行: 平地模式 - 巡线前进\n");
                    while(current_mode==CMD_SPACE_MOVE){
                        line_following();
                        //在while循环里面接收信息
                        osStatus_t status = osMessageQueueGet(motor_msg_queue, &msg, NULL, 10);
        
                        if (status == osOK) {
                        // 收到新命令，更新当前模式
                        current_mode = msg.cmd;
                        osal_printk("切换到新模式: %d\n", current_mode);
                        }
                    }
                    break;

                case CMD_UPSTAIR_MOVE:
                    osal_printk("开始上楼梯\n");
                    start_time=uapi_tcxo_get_us();
                    while(current_mode==CMD_UPSTAIR_MOVE){
                        mode_flag=up_stair(start_time);
                        if(mode_flag==3){
                            osal_printk("无法上楼梯,掉头，继续在该楼层巡线\n");
                            pwm_control(LEFT_MOTOR_PWM, PWM_MODE, 0, 500, LEFT_MOTOR_CHANNEL);
                            pwm_control(RIGHT_MOTOR_PWM, PWM_MODE, 0, 500, RIGHT_MOTOR_CHANNEL);
                            motor_right_backward(RIGHT_MOTOR_MOVE_OUT1,RIGHT_MOTOR_MOVE_OUT2);
                            motor_left_forward(LEFT_MOTOR_MOVE_OUT1,LEFT_MOTOR_MOVE_OUT2);
                            osDelay(1000);
                            break;
                        }
                        osStatus_t status = osMessageQueueGet(motor_msg_queue, &msg, NULL, 10);
        
                        if (status == osOK) {
                        // 收到新命令，更新当前模式
                        current_mode = msg.cmd;
                        osal_printk("切换到新模式: %d\n", current_mode);
                         }
                        
                    }
                    osal_printk("flag=%d\t", mode_flag);
                    current_mode=CMD_SPACE_MOVE;
                    break;

                case CMD_DOWMSTAIR_MOVE:
                    osal_printk("开始下楼梯");
                    while(current_mode==CMD_DOWMSTAIR_MOVE){
                        dowm_stair();
                    }
                    osal_printk("下楼梯结束");
                    current_mode=CMD_SPACE_MOVE;
                    break;
                    
                case CMD_UNSAFE:
                    osal_printk("执行: 不安全模式 - 停止\n");
                    stop_motors();
                    current_mode=CMD_NONE;
                    break;
                    
                case CMD_ON:
                    osal_printk("执行: 开灯\n");
                    blinky_open(BLINKY_PIN, GPIO_LEVEL_HIGH);
                    current_mode=CMD_SPACE_MOVE;
                    break;
                    
                case CMD_OFF:
                    osal_printk("执行: 关灯\n");
                    blinky_close(BLINKY_PIN, GPIO_LEVEL_LOW);
                    current_mode=CMD_SPACE_MOVE;
                    break;

                case CMD_NONE:
                    break;
                    
                case CMD_UNKNOWN:
                default:
                    osal_printk("未知命令\n");
                    current_mode = CMD_NONE;
                    osDelay(10);
                    break;
            }
    }
    return NULL;
}

// ============ 系统入口 ============
/**
 * @brief 创建消息队列和所有任务
 */
static void app_entry(void)
{
    osal_printk("=== 双线程控制系统启动 ===\n");
    
    // 1. 创建消息队列（最多存储5条消息）
    motor_msg_queue = osMessageQueueNew(5, sizeof(motor_msg_t), NULL);
    if (motor_msg_queue == NULL) {
        osal_printk("消息队列创建失败！\n");
        return;
    }
    osal_printk("消息队列创建成功\n");
    
    // 2. 创建UART任务
    osal_task *uart_task_handle = NULL;
    osal_kthread_lock();
    uart_task_handle = osal_kthread_create((osal_kthread_handler)uart_task, 0, 
                                          "UartTask", UART_TASK_STACK_SIZE);
    if (uart_task_handle != NULL) {
        osal_kthread_set_priority(uart_task_handle, UART_TASK_PRIO);
    }
    osal_kthread_unlock();
    
    if (uart_task_handle != NULL) {
        osal_printk("UART任务创建成功, ID = %p\n", uart_task_handle);
    }
    
    // 3. 创建电机控制任务
    osal_task *motor_task_handle = NULL;
    osal_kthread_lock();
    motor_task_handle = osal_kthread_create((osal_kthread_handler)motor_task, 0, 
                                           "MotorTask", MOTOR_TASK_STACK_SIZE);
    if (motor_task_handle != NULL) {
        osal_kthread_set_priority(motor_task_handle, MOTOR_TASK_PRIO);
    }
    osal_kthread_unlock();
    
    if (motor_task_handle != NULL) {
        osal_printk("电机任务创建成功, ID = %p\n", motor_task_handle);
    }
    
    osal_printk("系统启动完成，等待串口命令...\n");
}

/* 应用入口 */
app_run(app_entry);