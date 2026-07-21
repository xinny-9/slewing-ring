#include "cargo_door.h"

// 实例化两扇门对象
CargoDoor_t door_top;
CargoDoor_t door_lower;

/* 状态名称字符串映射 */
static const char* Get_State_Name(DoorState_t state)
{
    switch (state)
    {
        case DOOR_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case DOOR_STATE_CLOSED:        return "CLOSED";
        case DOOR_STATE_OPENING:       return "OPENING";
        case DOOR_STATE_OPENED:        return "OPENED";
        case DOOR_STATE_CLOSING:       return "CLOSING";
        case DOOR_STATE_STOPPED:       return "STOPPED";
        default:                       return "UNKNOWN";
    }
}

/* 内部辅助：读取接近开关状态 (1:触发导通, 0:未触发) */
static uint8_t Read_Limit_State(GPIO_TypeDef* port, uint16_t pin)
{
    if (HAL_GPIO_ReadPin(port, pin) == LIMIT_TRIGGER_LEVEL) {
        return 1;
    }
    return 0;
}

/**
  * @brief  仓门组件全引脚初始化绑定
  */
void CargoDoor_Init(CargoDoor_t* door, const char* name, TB6612_MotorTypeDef* motor,
                    GPIO_TypeDef* open_port, uint16_t open_pin,
                    GPIO_TypeDef* close_port, uint16_t close_pin)
{
    door->Name             = name;
    door->Motor            = motor;
    door->Limit_Open_Port  = open_port;
    door->Limit_Open_Pin   = open_pin;
    door->Limit_Close_Port = close_port;
    door->Limit_Close_Pin  = close_pin;
    
    door->State            = DOOR_STATE_UNINITIALIZED;
    door->timer_start      = 0;

    printf("[%s] Initialized. Initial State: UNINITIALIZED\r\n", door->Name);
}

/**
  * @brief  状态机轮询更新函数，需在主循环中被高频调用
  */
void CargoDoor_Update(CargoDoor_t* door)
{
    uint8_t limit_open_active  = Read_Limit_State(door->Limit_Open_Port,  door->Limit_Open_Pin);
    uint8_t limit_close_active = Read_Limit_State(door->Limit_Close_Port, door->Limit_Close_Pin);

    // 状态转移核心逻辑
    switch (door->State)
    {
        case DOOR_STATE_UNINITIALIZED:
            TB6612_SetMotor(door->Motor, MOTOR_BRAKE, 0);
            printf("[%s] Init completed. State -> STOPPED\r\n", door->Name);
            door->State = DOOR_STATE_STOPPED;
            break;

        case DOOR_STATE_CLOSED:
            break;

        case DOOR_STATE_OPENING:
            // 启动前一段时间内屏蔽限位防抖
            if (HAL_GetTick() - door->timer_start > LIMIT_DEBOUNCE_MASK_MS)
            {
                if (limit_open_active)
                {
                    TB6612_SetMotor(door->Motor, MOTOR_BRAKE, 0);
                    printf("[%s] >>> LIMIT OPEN TRIGGERED! State -> OPENED. Stopping motor.\r\n", door->Name);
                    door->State = DOOR_STATE_OPENED;
                }
            }
            break;

        case DOOR_STATE_OPENED:
            break;

        case DOOR_STATE_CLOSING:
            // 启动前一段时间内屏蔽限位防抖
            if (HAL_GetTick() - door->timer_start > LIMIT_DEBOUNCE_MASK_MS)
            {
                if (limit_close_active)
                {
                    TB6612_SetMotor(door->Motor, MOTOR_BRAKE, 0);
                    printf("[%s] >>> LIMIT CLOSE TRIGGERED! State -> CLOSED. Stopping motor.\r\n", door->Name);
                    door->State = DOOR_STATE_CLOSED;
                }
            }
            break;

        case DOOR_STATE_STOPPED:
            break;

        default:
            door->State = DOOR_STATE_STOPPED;
            break;
    }
}

/**
  * @brief  对外公开API：执行【开仓】动作指令
  */
void CargoDoor_Open(CargoDoor_t* door)
{
    if (door->State != DOOR_STATE_OPENING && door->State != DOOR_STATE_OPENED)
    {
        printf("[%s] Activating: OPEN command received.\r\n", door->Name);
        printf("[%s] State transition: %s -> OPENING (Motor FWD, Speed=%d)\r\n", 
               door->Name, Get_State_Name(door->State), MOTOR_DEFAULT_SPEED);
        
        door->State = DOOR_STATE_OPENING;
        door->timer_start = HAL_GetTick();
        
        TB6612_SetMotor(door->Motor, MOTOR_FWD, MOTOR_DEFAULT_SPEED);
    }
    else
    {
        printf("[%s] Activating IGNORED: Already in %s state.\r\n", door->Name, Get_State_Name(door->State));
    }
}

/**
  * @brief  对外公开API：执行【关仓】动作指令
  */
void CargoDoor_Close(CargoDoor_t* door)
{
    if (door->State != DOOR_STATE_CLOSING && door->State != DOOR_STATE_CLOSED)
    {
        printf("[%s] Activating: CLOSE command received.\r\n", door->Name);
        printf("[%s] State transition: %s -> CLOSING (Motor REV, Speed=%d)\r\n", 
               door->Name, Get_State_Name(door->State), MOTOR_DEFAULT_SPEED);
        
        door->State = DOOR_STATE_CLOSING;
        door->timer_start = HAL_GetTick();
        
        TB6612_SetMotor(door->Motor, MOTOR_REV, MOTOR_DEFAULT_SPEED);
    }
    else
    {
        printf("[%s] Activating IGNORED: Already in %s state.\r\n", door->Name, Get_State_Name(door->State));
    }
}

/**
  * @brief  对外公开API：执行【暂停/停止】动作指令
  */
void CargoDoor_Stop(CargoDoor_t* door)
{
    printf("[%s] Activating: STOP command received.\r\n", door->Name);
    printf("[%s] State transition: %s -> STOPPED (Motor BRAKE)\r\n", door->Name, Get_State_Name(door->State));
    door->State = DOOR_STATE_STOPPED;
    TB6612_SetMotor(door->Motor, MOTOR_BRAKE, 0);
}

void CargoDoor_PrintStatus(CargoDoor_t* door)
{
    GPIO_PinState pin_open = HAL_GPIO_ReadPin(door->Limit_Open_Port, door->Limit_Open_Pin);
    GPIO_PinState pin_close = HAL_GPIO_ReadPin(door->Limit_Close_Port, door->Limit_Close_Pin);
    
    uint8_t lim_open  = (pin_open  == LIMIT_TRIGGER_LEVEL) ? 1 : 0;
    uint8_t lim_close = (pin_close == LIMIT_TRIGGER_LEVEL) ? 1 : 0;
    
    printf("  [%s]\r\n", door->Name);
    printf("    State             : %s\r\n", Get_State_Name(door->State));
    printf("    Limit OPEN Pin    : %s  (Electrical level: %s)\r\n", 
           lim_open ? "TRIGGERED" : "FREE", (pin_open == GPIO_PIN_SET) ? "HIGH (3.3V)" : "LOW (GND)");
    printf("    Limit CLOSE Pin   : %s  (Electrical level: %s)\r\n", 
           lim_close ? "TRIGGERED" : "FREE", (pin_close == GPIO_PIN_SET) ? "HIGH (3.3V)" : "LOW (GND)");
}

