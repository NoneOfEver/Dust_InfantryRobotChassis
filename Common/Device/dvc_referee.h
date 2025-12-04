
/**
 * Command_ID分全局, 己方, 指定机器人
 * 0x00xx, 全局是Game
 * 0x01xx, 己方是Event_Self
 * 0x02xx, 指定机器人是Robot_x, 没有x则是任意己方机器人
 * 0x03xx的指令帧直接具体指明收发方
 */

#ifndef DVC_REFEREE_H
#define DVC_REFEREE_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_usart.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裁判系统状态
 *
 */
enum RefereeStatus
{
    Referee_Status_DISABLE = 0,
    Referee_Status_ENABLE,
};

/**
 * @brief 各种标签, 场地, 相关设施激活与存活状态
 *
 */
enum RefereeDataStatus : uint8_t
{
    Referee_Data_Status_DISABLE = 0,
    Referee_Data_Status_ENABLE,
};

/**
 * @brief 裁判系统命令码类型
 *
 */
enum RefereeCommandID : uint16_t
{
    Referee_Command_ID_GAME_STATUS = 0x0001,
    Referee_Command_ID_GAME_RESULT = 0x0002,
    Referee_Command_ID_GAME_ROBOT_HP = 0x0003,
    Referee_Command_ID_EVENT_SELF_DATA = 0x0101,
    Referee_Command_ID_EVENT_SELF_SUPPLY,
    Referee_Command_ID_EVENT_SELF_REFEREE_WARNING = 0x0104,
    Referee_Command_ID_EVENT_SELF_DART_STATUS = 0x0105,
    Referee_Command_ID_ROBOT_STATUS = 0x0201,
    Referee_Command_ID_ROBOT_POWER_HEAT = 0x0202,
    Referee_Command_ID_ROBOT_POSITION = 0x0203,
    Referee_Command_ID_ROBOT_BUFF = 0x0204,
    Referee_Command_ID_ROBOT_AERIAL_STATUS,
    Referee_Command_ID_ROBOT_DAMAGE = 0x0206,
    Referee_Command_ID_ROBOT_BOOSTER = 0x0207,
    Referee_Command_ID_ROBOT_REMAINING_AMMO = 0x0208,
    Referee_Command_ID_ROBOT_RFID = 0x0209,
    Referee_Command_ID_ROBOT_DART_COMMAND = 0x020A,
    Referee_Command_ID_ROBOT_SENTRY_LOCATION = 0x020B,
    Referee_Command_ID_ROBOT_RADAR_MARK = 0x020C,
    Referee_Command_ID_ROBOT_SENTRY_DECISION = 0x020D,
    Referee_Command_ID_ROBOT_RADAR_DECISION = 0x020E,
    Referee_Command_ID_INTERACTION = 0x0301,
    Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CUSTOM_CONTROLLER = 0x0302,
    Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_MINIMAP = 0x0303,
    Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_REMOTE_CONTROL = 0x0304,
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_RADAR = 0x0305,
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_CUSTOM_CONTROLLER = 0x0306,
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_SENTRY_SEMIAUTOMATIC_MINIMAP = 0x0307,
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_ROBOT_MINIMAP = 0x0308,
};

/**
 * @brief 裁判系统 interaction 0x0301子命令码类型
 *
 */
enum RefereeInteractionCommandID : uint16_t
{
    Referee_Interaction_Command_ID_UI_LAYER_DELETE = 0x0100,
    Referee_Interaction_Command_ID_UI_GRAPHIC_1,
    Referee_Interaction_Command_ID_UI_GRAPHIC_2,
    Referee_Interaction_Command_ID_UI_GRAPHIC_5,
    Referee_Interaction_Command_ID_UI_GRAPHIC_7,
    Referee_Interaction_Command_ID_UI_GRAPHIC_STRING = 0x0110,
    Referee_Interaction_Command_ID_SENTRY = 0x0120,
    Referee_Interaction_Command_ID_RADAR = 0x0121,
};

/**
 * @brief 通用单方机器人ID
 *
 */
enum RefereeDataRobotID : uint8_t
{
    Referee_Data_Robot_ID_NULL = 0,
    Referee_Data_Robot_ID_HERO_1,
    Referee_Data_Robot_ID_ENGINEER_2,
    Referee_Data_Robot_ID_INFANTRY_3,
    Referee_Data_Robot_ID_INFANTRY_4,
    Referee_Data_Robot_ID_INFANTRY_5,
    Referee_Data_Robot_ID_AERIAL_6,
    Referee_Data_Robot_ID_SENTRY_7,
    Referee_Data_Robot_ID_DART_8,
    Referee_Data_Robot_ID_RADAR_9,
    Referee_Data_Robot_ID_BASE_10,
    Referee_Data_Robot_ID_OUTPOST_11,
};

/**
 * @brief 通用双方机器人ID
 *
 */
enum RefereeDataRobotsID : uint8_t
{
    Referee_Data_Robots_ID_NO = 0,
    Referee_Data_Robots_ID_RED_HERO_1,
    Referee_Data_Robots_ID_RED_ENGINEER_2,
    Referee_Data_Robots_ID_RED_INFANTRY_3,
    Referee_Data_Robots_ID_RED_INFANTRY_4,
    Referee_Data_Robots_ID_RED_INFANTRY_5,
    Referee_Data_Robots_ID_RED_AERIAL_6,
    Referee_Data_Robots_ID_RED_SENTRY_7,
    Referee_Data_Robots_ID_RED_DART_8,
    Referee_Data_Robots_ID_RED_RADAR_9,
    Referee_Data_Robots_ID_RED_BASE_10,
    Referee_Data_Robots_ID_RED_OUTPOST_11,
    Referee_Data_Robots_ID_BLUE_HERO_1 = 101,
    Referee_Data_Robots_ID_BLUE_ENGINEER_2,
    Referee_Data_Robots_ID_BLUE_INFANTRY_3,
    Referee_Data_Robots_ID_BLUE_INFANTRY_4,
    Referee_Data_Robots_ID_BLUE_INFANTRY_5,
    Referee_Data_Robots_ID_BLUE_AERIAL_6,
    Referee_Data_Robots_ID_BLUE_SENTRY_7,
    Referee_Data_Robots_ID_BLUE_DART_8,
    Referee_Data_Robots_ID_BLUE_RADAR_9,
    Referee_Data_Robots_ID_BLUE_BASE_10,
    Referee_Data_Robots_ID_BLUE_OUTPOST_11,
};
/**
 * @brief 通用双方机器人ID
 *
 */
enum RefereeDataRobotsClientID : uint16_t
{
    Referee_Data_Robots_Client_ID_NO = 0,
    Referee_Data_Robots_Client_ID_RED_HERO_1 = 0x0101,
    Referee_Data_Robots_Client_ID_RED_ENGINEER_2,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_3,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_4,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_5,
    Referee_Data_Robots_Client_ID_RED_AERIAL_6,
    Referee_Data_Robots_Client_ID_BLUE_HERO_1 = 0x0165,
    Referee_Data_Robots_Client_ID_BLUE_ENGINEER_2,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_3,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_4,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_5,
    Referee_Data_Robots_Client_ID_BLUE_AERIAL_6,
    Referee_Data_Robots_Server = 0x8080,
};

/**
 * @brief 比赛类型 0x0001
 *
 */
enum RefereeGameStatusType
{
    Referee_Game_Status_Type_RMUC = 1,
    Referee_Game_Status_Type_SINGLE,
    Referee_Game_Status_Type_ICRA,
    Referee_Game_Status_Type_RMUL_3V3,
    Referee_Game_Status_Type_RMUL_1V1,
};

/**
 * @brief 比赛阶段 0x0001
 *
 */
enum RefereeGameStatusStage
{
    Referee_Game_Status_Stage_NOT_STARTED = 0,
    Referee_Game_Status_Stage_READY,
    Referee_Game_Status_Stage_15s_SELF_TESTING,
    Referee_Game_Status_Stage_5S_COUNTDOWN,
    Referee_Game_Status_Stage_BATTLE,
    Referee_Game_Status_Stage_SETTLEMENT,
};

/**
 * @brief 比赛结果 0x0002
 *
 */
enum RefereeGameResult : uint8_t
{
    Referee_Game_Result_DRAW = 0,
    Referee_Game_Result_RED_WIN,
    Referee_Game_Result_BLUE_WIN,
};

/**
 * @brief 飞镖命中目标
 *
 */
enum RefereeDartHitTarget : uint8_t
{
    Referee_Dart_Hit_Target_NULL = 0,
    Referee_Dart_Hit_Target_OUTPOST,
    Referee_Dart_Hit_Target_BASE_STATIC,
    Referee_Dart_Hit_Target_BASE_RANDOM,
};

/**
 * @brief 补给站状态
 *
 */
enum RefereeDataEventSupplyStatus : uint8_t
{
    Referee_Data_Event_Supply_Status_CLOSED = 0,
    Referee_Data_Event_Supply_Status_READY,
    Referee_Data_Event_Supply_Status_DROPPING,
};

/**
 * @brief 补给站提供子弹数量
 *
 */
enum RefereeDataEventSupplyAmmoNumber : uint8_t
{
    Referee_Data_Event_Supply_Ammo_Number_50 = 50,
    Referee_Data_Event_Supply_Ammo_Number_100 = 100,
    Referee_Data_Event_Supply_Ammo_Number_150 = 150,
    Referee_Data_Event_Supply_Ammo_Number_200 = 200,
};

/**
 * @brief 裁判警告等级
 *
 */
enum RefereeDataEventRefereeWarningLevel : uint8_t
{
    Referee_Data_Referee_Warning_Level_BOTH_YELLOW = 1,
    Referee_Data_Referee_Warning_Level_YELLOW,
    Referee_Data_Referee_Warning_Level_RED,
    Referee_Data_Referee_Warning_Level_FAIL,
};

/**
 * @brief 裁判警告等级
 *
 */
enum RefereeDataEventAerialStatus : uint8_t
{
    Referee_Data_Event_Aerial_Status_COOLING = 0,
    Referee_Data_Event_Aerial_Status_READY,
    Referee_Data_Event_Aerial_Status_EXECUTING,
};

/**
 * @brief 伤害类型
 *
 */
enum RefereeDataEventRobotDamageType
{
    Referee_Data_Robot_Damage_Type_ARMOR_ATTACKED = 0,
    Referee_Data_Robot_Damage_Type_MODULE_OFFLINE,
    Referee_Data_Robot_Damage_Type_BOOSTER_SPEED,
    Referee_Data_Robot_Damage_Type_BOOSTER_HEAT,
    Referee_Data_Robot_Damage_Type_CHASSIS_POWER,
    Referee_Data_Robot_Damage_Type_ARMOR_COLLISION,
};

/**
 * @brief 子弹类型
 *
 */
enum RefereeDataRobotAmmoType : uint8_t
{
    Referee_Data_Robot_Ammo_Type_BOOSTER_17MM = 1,
    Referee_Data_Robot_Ammo_Type_BOOSTER_42mm,
};

/**
 * @brief 发射机构类型
 *
 */
enum RefereeDataRobotBoosterType : uint8_t
{
    Referee_Data_Robot_Booster_Type_BOOSTER_17MM_1 = 1,
    Referee_Data_Robot_Booster_Type_BOOSTER_17MM_2,
    Referee_Data_Robot_Booster_Type_BOOSTER_42mm,
};

/**
 * @brief 飞镖发射口状态
 *
 */
enum RefereeDataRobotDartCommandStatus : uint8_t
{
    Referee_Data_Robot_Dart_Command_Status_OPEN = 0,
    Referee_Data_Robot_Dart_Command_Status_CLOSED,
    Referee_Data_Robot_Dart_Command_Status_EXECUTING,
};

/**
 * @brief 飞镖击打目标
 *
 */
enum RefereeDataRobotDartCommandTarget : uint8_t
{
    Referee_Data_Robot_Dart_Command_Target_OUTPOST = 0,
    Referee_Data_Robot_Dart_Command_Target_BASE,
};

/**
 * @brief 图形操作交互信息
 *
 */
enum RefereeDataInteractionLayerDeleteOperation : uint8_t
{
    Referee_Data_Interaction_Layer_Delete_Operation_NULL = 0,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ONE,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ALL,
};

/**
 * @brief 图形操作
 *
 */
enum RefereeDataInteractionGraphicOperation
{
    Referee_Data_Interaction_Graphic_Operation_NULL = 0,
    Referee_Data_Interaction_Graphic_Operation_ADD,
    Referee_Data_Interaction_Graphic_Operation_CHANGE,
    Referee_Data_Interaction_Graphic_Operation_DELETE,
};

/**
 * @brief 图形类型
 *
 */
enum RefereeDataInteractionGraphicType
{
    Referee_Data_Interaction_Graphic_Type_LINE = 0,
    Referee_Data_Interaction_Graphic_Type_RECTANGLE,
    Referee_Data_Interaction_Graphic_Type_CIRCLE,
    Referee_Data_Interaction_Graphic_Type_OVAL,
    Referee_Data_Interaction_Graphic_Type_ARC,
    Referee_Data_Interaction_Graphic_Type_FLOAT,
    Referee_Data_Interaction_Graphic_Type_INTEGER,
    Referee_Data_Interaction_Graphic_Type_STRING,
};

/**
 * @brief 图形颜色
 *
 */
enum RefereeDataInteractionGraphicColor
{
    Referee_Data_Interaction_Graphic_Color_MAIN = 0,
    Referee_Data_Interaction_Graphic_Color_YELLOW,
    Referee_Data_Interaction_Graphic_Color_GREEN,
    Referee_Data_Interaction_Graphic_Color_ORANGE,
    Referee_Data_Interaction_Graphic_Color_PURPLE,
    Referee_Data_Interaction_Graphic_Color_PINK,
    Referee_Data_Interaction_Graphic_Color_CYAN,
    Referee_Data_Interaction_Graphic_Color_BLACK,
    Referee_Data_Interaction_Graphic_Color_WHITE,
};

/**
 * @brief 图形操作交互信息
 *
 */
enum RefereeDataInteractionSemiautomaticCommand : uint8_t
{
    Referee_Data_Interaction_Semiautomatic_Command_ATTACK = 1,
    Referee_Data_Interaction_Semiautomatic_Command_DEFENCE,
    Referee_Data_Interaction_Semiautomatic_Command_MOVE,
};

/**
 * @brief 图形配置结构体
 *
 */
struct RefereeDataInteractionGraphicConfig
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Layer_Num : 4;
    uint32_t Color_Enum : 4;
    uint32_t Details_A : 9;
    uint32_t Details_B : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Details_C : 10;
    uint32_t Details_D : 11;
    uint32_t Details_E : 11;
} __attribute__((packed));

// TODO
// /**
//  * @brief 图形配置结构体
//  *
//  */
// struct RefereeDataInteractionGraphicConfig_test
// {
//     uint8_t Index[3];
//     RefereeDataInteractionGraphicOperation Operation : 3;
//     RefereeDataInteractionGraphicType Type : 3;
//     uint8_t Layer_Num : 4;
//     RefereeDataInteractionGraphicColor Color : 4;
//     uint32_t Details_A : 9;
//     uint32_t Details_B : 9;
//     uint32_t Line_Width : 10;
//     uint32_t Start_X : 11;
//     uint32_t Start_Y : 11;
//     uint32_t Details_C : 10;
//     uint32_t Details_D : 11;
//     uint32_t Details_E : 11;
// } __attribute__((packed));

/**
 * @brief 裁判系统源数据
 *
 */
struct RefereeUartData
{
    uint8_t Frame_Header = 0xa5;
    uint16_t Data_Length;
    uint8_t Sequence;
    uint8_t CRC_8;
    RefereeCommandID Referee_Command_ID;
    uint8_t Data[121];
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0001比赛状态, 3Hz发送
 *
 */
struct RefereeRxDataGameStatus
{
    uint8_t Type_Enum : 4;
    uint8_t Stage_Enum : 4;
    uint16_t Remaining_Time;
    uint64_t Timestamp;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0002比赛结果, 比赛结束后发送
 *
 */
struct RefereeRxDataGameResult
{
    RefereeGameResult Result;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0003机器人血量, 1Hz
 *
 */
struct RefereeRxDataGameRobotHP
{
    uint16_t Red_Hero_1;
    uint16_t Red_Engineer_2;
    uint16_t Red_Infantry_3;
    uint16_t Red_Infantry_4;
    uint16_t Red_Infantry_5;
    uint16_t Red_Sentry_7;
    uint16_t Red_Outpost_11;
    uint16_t Red_Base_10;
    uint16_t Blue_Hero_1;
    uint16_t Blue_Engineer_2;
    uint16_t Blue_Infantry_3;
    uint16_t Blue_Infantry_4;
    uint16_t Blue_Infantry_5;
    uint16_t Blue_Sentry_7;
    uint16_t Blue_Outpost_11;
    uint16_t Blue_Base_10;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0101场地事件, 1Hz发送
 *
 */
struct RefereeRxDataEventSelfData
{
    uint32_t Supply_Front_Status_Enum : 1;
    uint32_t Supply_Inner_Status_Enum : 1;
    uint32_t Supply_Status_Enum : 1;
    uint32_t Power_Rune_Status_Enum : 1;
    uint32_t Power_Rune_Small_Status_Enum : 1;
    uint32_t Power_Rune_Big_Status_Enum : 1;
    uint32_t Highland_2_Status_Enum : 2;
    uint32_t Highland_3_Status_Enum : 2;
    uint32_t Highland_4_Status_Enum : 2;
    uint32_t Base_Virtual_Shield_Percent : 7;
    uint32_t Enemy_Dart_Hit_Time : 9;
    uint32_t Enemy_Dart_Hit_Target_Enum : 2;
    uint32_t Middle_Buff_Status_Enum : 2;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0102补给站状态, 补给请求后对应发送
 *
 */
struct RefereeRxDataEventSelfSupply
{
    uint8_t Reserved;
    RefereeDataRobotsID Robot;
    RefereeDataEventSupplyStatus Status;
    RefereeDataEventSupplyAmmoNumber Ammo_Number;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0104裁判警告信息, 判罚发生后发送
 *
 */
struct RefereeRxDataEventRefereeWarning
{
    RefereeDataEventRefereeWarningLevel Level;
    RefereeDataRobotID Robot_ID;
    uint8_t Count;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0105飞镖15s倒计时, 1Hz发送
 *
 */
struct RefereeRxDataEventDartStatus
{
    uint8_t Dart_Remaining_Time;
    uint16_t Dart_Hit_Target_Enum_Last : 2;
    uint16_t Dart_Hit_Target_Count : 3;
    uint16_t Dart_Hit_Target_Enum_Now : 2;
    uint16_t Reserved : 9;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0201机器人状态, 10Hz发送
 *
 */
struct RefereeRxDataRobotStatus
{
    RefereeDataRobotsID Robot_ID;
    uint8_t Level;
    uint16_t HP;
    uint16_t HP_Max;
    uint16_t Booster_Heat_CD;
    uint16_t Booster_Heat_Max;
    uint16_t Chassis_Power_Max;
    uint8_t PM01_Gimbal_Status_Enum : 1;
    uint8_t PM01_Chassis_Status_Enum : 1;
    uint8_t PM01_Booster_Status_Enum : 1;
    uint8_t Reserved : 5;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0202当前机器人实时功率热量, 50Hz发送
 * 电压mV, 电流mA
 *
 */
struct RefereeRxDataRobotPowerHeat
{
    uint16_t reserverd1;
    uint16_t reserverd2;
    float reserverd3;
    uint16_t Chassis_Energy_Buffer;
    uint16_t Booster_17mm_1_Heat;
    uint16_t Booster_17mm_2_Heat;
    uint16_t Booster_42mm_Heat;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0203当前机器人实时位置, 10Hz发送
 *
 */
struct RefereeRxDataRobotPosition
{
    float Location_X;
    float Location_Y;
    float Location_Yaw;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0204当前机器人增益, 1Hz发送
 *
 */
struct RefereeRxDataRobotBuff
{
    uint8_t HP_Buff_Percent;
    uint8_t Booster_Heat_CD_Buff_Value;
    uint8_t Defend_Buff_Percent;
    uint8_t Defend_Debuff_Percent;
    uint16_t Damage_Buff_Percent;
    uint8_t remaining_energy;
    uint16_t CRC_16;
} __attribute__((packed));

// /**
//  * @brief 裁判系统经过处理的数据, 0x0205无人机可攻击时间, 10Hz发送
//  *
//  */
// struct RefereeRxDataRobotAerialStatus
// {
//     RefereeDataEventAerialStatus Aerial_Status;
//     uint8_t Remaining_Time;
//     uint16_t CRC_16;
// } __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0206伤害情况, 伤害发生后发送
 *
 */
struct RefereeRxDataRobotDamage
{
    uint8_t Armor_ID : 4;
    uint8_t Type_Enum : 4;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0207子弹信息, 射击发生后发送
 *
 */
struct RefereeRxDataRobotBooster
{
    RefereeDataRobotAmmoType Ammo_Type;
    RefereeDataRobotBoosterType Booster_Type;
    uint8_t Frequency;
    float Speed;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0208子弹剩余信息, 10Hz发送
 *
 */
struct RefereeRxDataRobotRemainingAmmo
{
    uint16_t Booster_17mm;
    uint16_t Booster_42mm;
    uint16_t Money;
    uint16_t projectile_allowance_fortress;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0209RFID状态信息, 1Hz发送
 *
 */
struct RefereeRxDataRobotRFID
{
    uint32_t Base_Status_Enum : 1;                  //己方基地增益点
    uint32_t Highland_2_Self_Status_Enum : 1;       //己方中央高地增益点
    uint32_t Highland_2_Enemy_Status_Enum : 1;      //对方中央高地增益点
    uint32_t Highland_3_Self_Status_Enum : 1;       //己方梯形高地增益点
    uint32_t Highland_3_Enemy_Status_Enum : 1;      //对方梯形高地增益点
    uint32_t Flyover_1_Self_Status_Enum : 1;        //己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    uint32_t Flyover_2_Self_Status_Enum : 1;        //己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
    uint32_t Flyover_1_Enemy_Status_Enum : 1;       //对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
    uint32_t Flyover_2_Enemy_Status_Enum : 1;       //对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
    uint32_t Highland_4_Self_Status_Enum_Down : 1;  //己方地形跨越增益点（中央高地下方）
    uint32_t Highland_4_Self_Status_Enum_Up : 1;    //己方地形跨越增益点（中央高地上方）
    uint32_t Highland_4_Enemy_Status_Enum_Down : 1; //对方地形跨越增益点（中央高地下方）
    uint32_t Highland_4_Enemy_Status_Enum_Up : 1;   //对方地形跨越增益点（中央高地上方）
    uint32_t Road_Self_Status_Enum_Down : 1;        //己方地形跨越增益点（公路下方）
    uint32_t Road_Self_Status_Enum_Up : 1;          //己方地形跨越增益点（公路上方）
    uint32_t Road_Enemy_Status_Enum_Down : 1;       //对方地形跨越增益点（公路下方）
    uint32_t Road_Enemy_Status_Enum_Up : 1;         //对方地形跨越增益点（公路上方）
    uint32_t Self_Fortress_Status_Enum : 1;         //己方堡垒增益点
    uint32_t Self_Outpost_Status_Enum : 1;          //己方前哨站增益点
    uint32_t Supply_Zone_Not_Overlapping : 1;       //己方与兑换区不重叠的补给区/RMUL 补给区
    uint32_t Supply_Zone_Overlapping : 1;           //己方与兑换区重叠的补给区
    uint32_t Engineer_Self_Status_Enum : 1;         //己方大资源岛增益点
    uint32_t Engineer_Enemy_Status_Enum : 1;        //对方大资源岛增益点
    uint32_t Center_Status_Enum : 1;                //中心增益点（仅 RMUL 适用）
    uint32_t Enemy_Outpost_Status_Enum : 1;         //对方堡垒增益点
    uint32_t Reserved : 7;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020a飞镖状态, 10Hz发送
 *
 */
struct RefereeRxDataRobotDartCommand
{
    RefereeDataRobotDartCommandStatus Status;
    uint8_t Reserved;
    uint16_t Switch_Remaining_Time;
    uint16_t Launch_Remaining_Time;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020b哨兵获取己方位置信息, 1Hz发送
 *
 */
struct RefereeRxDataRobotSentryLocation
{
    float Hero_1_X;
    float Hero_1_Y;
    float Engineer_2_X;
    float Engineer_2_Y;
    float Infantry_3_X;
    float Infantry_3_Y;
    float Infantry_4_X;
    float Infantry_4_Y;
    float reserved1;
    float reserved2;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020c雷达标记进度, 1Hz发送
 *
 */
struct RefereeRxDataRobotRadarMark
{
    uint8_t Hero_1_Mark : 1;
    uint8_t Engineer_2_Mark : 1;
    uint8_t Infantry_3_Mark : 1;
    uint8_t Infantry_4_Mark : 1;
    uint8_t Sentry_7_Mark : 1;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020d哨兵决策信息, 1Hz发送
 *
 */
struct RefereeRxDataRobotSentryDecision
{
    uint32_t Ammo_Exchange_Number : 11;
    uint32_t Ammo_Exchange_Time : 4;
    uint32_t HP_Exchange_Time : 4;
    uint32_t Sentry_Free_Reborn_CanConfirm : 1;
    uint32_t Sentry_Free_Reborn_Can_Immediately : 1;
    uint32_t Sentry_Reborn_Cost : 10;
    uint32_t Reserved : 1;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020e雷达决策信息, 1Hz发送
 *
 */
struct RefereeRxDataRobotRadarDecision
{
    uint8_t Double_Damage_Chance : 2;
    uint8_t Double_Damage_Enemy_Status_Enum : 1;
    uint8_t Reserved : 5;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301机器人间交互信息, 用户自主发送
 * Header 0x0200~0x02ff, 机器人之间通信
 * Header 0x0100~0x0104, UI绘制图形
 * Header 0x0110, UI绘制字符
 * Header 0x0120, 哨兵决策
 * Header 0x0121, 雷达决策
 * Data 最大112
 *
 */
// struct Struct_Referee_Data_Interaction
// {
//     uint16_t Header;
//     RefereeDataRobotsID Sender;
//     RefereeDataRobotsID Receiver;
//     uint8_t Data[113];
//     uint16_t CRC_16;
// }__attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301图形删除交互信息, 用户自主发送
 * Header 0x0100
 *
 */
struct RefereeTxDataInteractionLayerDelete
{
    uint16_t Header = Referee_Interaction_Command_ID_UI_LAYER_DELETE;
    RefereeDataRobotsID Sender;
    uint8_t Reserved;
    RefereeDataRobotsClientID Receiver;
    RefereeDataInteractionLayerDeleteOperation Operation;
    uint8_t Delete_Serial;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画一个图形交互信息, 用户自主发送
 *
 */
struct RefereeTxDataInteractionGraphic1
{
    uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_1;
    RefereeDataRobotsID Sender;
    uint8_t Reserved;
    RefereeDataRobotsClientID Receiver;
    RefereeDataInteractionGraphicConfig Graphic[1];
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画两个图形交互信息, 用户自主发送
 *
 */
struct RefereeTxDataInteractionGraphic2
{
    uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_2;
    RefereeDataRobotsID Sender;
    uint8_t Reserved;
    RefereeDataRobotsClientID Receiver;
    RefereeDataInteractionGraphicConfig Graphic[2];
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画五个图形交互信息, 用户自主发送
 *
 */
struct RefereeTxDataInteractionGraphic5
{
    uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_5;
    RefereeDataRobotsID Sender;
    uint8_t Reserved;
    RefereeDataRobotsClientID Receiver;
    RefereeDataInteractionGraphicConfig Graphic[5];
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画七个图形交互信息, 用户自主发送
 *
 */
struct RefereeTxDataInteractionGraphic7
{
    uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_7;
    RefereeDataRobotsID Sender;
    uint8_t Reserved;
    RefereeDataRobotsClientID Receiver;
    RefereeDataInteractionGraphicConfig Graphic[7];
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画字符图形交互信息, 用户自主发送
 *
 */
struct RefereeTxDataInteractionGraphicString
{
    uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_STRING;
    RefereeDataRobotsID Sender;
    uint8_t Reserved;
    RefereeDataRobotsClientID Receiver;
    RefereeDataInteractionGraphicConfig Graphic_String;
    uint8_t String[30];
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301哨兵自主决策交互信息, 哨兵自主发送
 *
 */
struct RefereeTxDataInteractionSentry
{
    uint16_t Header = Referee_Interaction_Command_ID_SENTRY;
    uint32_t Confirm_Respawn_Status_Enum : 1;
    uint32_t Confirm_Exchange_Respawn_Status_Enum : 1;
    uint32_t Request_Exchange_Ammo_Number : 11;
    uint32_t Request_Exchange_Ammo_Time : 4;
    uint32_t Request_Exchange_HP_Time : 4;
    uint32_t Reserved : 11;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301雷达自主决策交互信息, 雷达自主发送
 *
 */
struct RefereeTxDataInteractionRadar
{
    uint16_t Header = Referee_Interaction_Command_ID_RADAR;
    RefereeDataStatus Request_Double_Damage;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0302自定义控制器交互信息, 用户自主发送
 * TODO 视情况赋予Data含义
 *
 */
struct RefereeRxDataInteractionCustomController
{
    uint8_t Data[30];
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0303客户端发送小地图交互信息, 用户自主最高2Hz发送
 *
 */
struct RefereeRxDataInteractionRobotReceiveClientMinimap
{
    float Coordinate_X;
    float Coordinate_Y;
    RefereeDataStatus Keyboard;
    RefereeDataRobotsID Enemy_ID;
    RefereeDataRobotsID Source_ID;
    uint8_t Reserved;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0304图传键鼠遥控交互信息, 30Hz发送
 * TODO 等待扩展
 *
 */
struct RefereeRxDataInteractionRobotReceiveClientRemoteControl
{
    uint16_t Mouse_X;
    uint16_t Mouse_Y;
    uint16_t Mouse_Z;
    RefereeDataStatus Mouse_Left_Key_Status;
    RefereeDataStatus Mouse_Right_Key_Status;
    uint16_t Keyboard_Key;
    uint16_t Reserved;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0305客户端接收小地图交互信息, 用户自主最高10Hz发送
 *
 */
struct RefereeRxDataInteractionClientReceiveRadar
{
    RefereeDataRobotsID Robot_ID;
    float Coordinate_X;
    float Coordinate_Y;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0306客户端接收模拟键鼠遥控交互信息, 用户自主最高30Hz发送
 * TODO 等待扩展
 *
 */
struct RefereeTxDataInteractionClientReceiveCustomController
{
    uint16_t Keyboard_Key;
    uint16_t Mouse_X : 12;
    RefereeDataStatus Mouse_Left_Key_Status : 4;
    uint16_t Mouse_Y : 12;
    RefereeDataStatus Mouse_Right_Key_Status : 4;
    uint16_t Reserved;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0307客户端接收模拟键鼠遥控交互信息, 用户自主最高30Hz发送
 *
 */
struct RefereeTxDataInteractionClientReceiveSentrySemiautomaticMinimap
{
    RefereeDataInteractionSemiautomaticCommand Command;
    uint16_t Start_X;
    uint16_t Start_Y;
    int8_t Delta_X_List[49];
    int8_t Delta_Y_List[49];
    RefereeDataRobotsClientID Sender_ID;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0308客户端接收模拟键鼠遥控交互信息, 用户自主最高30Hz发送
 *
 */
struct RefereeTxDataInteractionClientReceiveRobotMinimap
{
    RefereeDataRobotsClientID Sender_ID;
    RefereeDataRobotsClientID Receiver_ID;
    uint8_t Data[30];
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief Specialized, 裁判系统
 *
 */
class Referee
{
public:
    void Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header = 0xa5);

    inline RefereeStatus GetStatus();

    inline RefereeDataStatus GetRefereeTrustStatus();

    inline RefereeGameStatusType GetGameType();

    inline RefereeGameStatusStage GetGameStage();

    inline uint16_t GetRemainingTime();

    inline uint64_t GetTimestamp();

    inline RefereeGameResult GetResult();

    inline uint16_t GetHP(RefereeDataRobotsID Robots_ID);

    inline RefereeDataStatus GetSupplyFrontStatus();

    inline RefereeDataStatus GetSupplyInnerStatus();

    inline RefereeDataStatus GetSupplyStatus();

    inline RefereeDataStatus GetPowerRuneStatus();

    inline RefereeDataStatus GetPowerRuneSmallStatus();

    inline RefereeDataStatus GetPowerRuneBigStatus();

    inline RefereeDataStatus GetHighland2Status();

    inline RefereeDataStatus GetHighland3Status();

    inline RefereeDataStatus GetHighland4Status();

    inline uint8_t GetBaseVirtualShieldPercent();

    inline uint16_t GetEnemyDartHitTime();

    inline RefereeDartHitTarget GetEnemyDartHitTargetEnum();

    inline RefereeDataStatus GetMiddleBuffStatus();

    inline RefereeDataRobotsID GetSupplyRequestRobot();

    inline RefereeDataEventSupplyStatus GetSupplyRequestStatus();

    inline RefereeDataEventSupplyAmmoNumber GetSupplyAmmoNumber();

    inline RefereeDataEventRefereeWarningLevel GetRefereeWarning();

    inline RefereeDataRobotID GetRefereeWarningRobot();

    inline uint8_t GetDartRemainingTime();

    inline RefereeDartHitTarget GetLastDartHitTarget();

    inline uint8_t GetDartHitTargetCount();

    inline RefereeDartHitTarget GetNowDartHitTarget();

    inline RefereeDataRobotsID GetSelfID();

    inline uint8_t GetSelfLevel();

    inline uint16_t GetSelfHP();

    inline uint16_t GetSelfHPMax();

    inline uint16_t GetSelfBoosterHeatCD();

    inline uint16_t GetSelfBoosterHeatMax();

    inline uint16_t GetSelfChassisPowerMax();

    inline RefereeDataStatus GetSelfPM01GimbalStatus();

    inline RefereeDataStatus GetSelfPM01ChassisStatus();

    inline RefereeDataStatus GetSelfPM01BoosterStatus();

    inline float GetChassisVoltage();

    inline float GetChassisCurrent();

    inline float GetChassisPower();

    inline uint16_t GetChassisEnergyBuffer();

    inline uint16_t GetBooster17mm_1_Heat();

    inline uint16_t GetBooster17mm_2_Heat();

    inline uint16_t GetBooster42mmHeat();

    inline float GetLocationX();

    inline float GetLocationY();

    inline float GetLocationYaw();

    inline uint8_t GetHPBuffPercent();

    inline uint8_t GetBoosterHeatCDBuffValue();

    inline uint8_t GetDefendBuffPercent();

    inline uint8_t GetDefendDebuffPercent();

    inline uint8_t GetDamageBuffPercent();

    inline RefereeDataEventAerialStatus GetAerialStatus();

    inline uint8_t GetAerialRemainingTime();

    inline uint8_t GetArmorAttackedID();

    inline RefereeDataEventRobotDamageType GetAttackedType();

    inline RefereeDataRobotAmmoType GetShootAmmoType();

    inline RefereeDataRobotBoosterType GetShootBoosterType();

    inline uint8_t GetShootFrequency();

    inline float GetShootSpeed();

    inline uint16_t Get17mmRemaining();

    inline uint16_t Get42mmRemaining();

    inline uint16_t GetMoneyRemaining();

    inline RefereeDataStatus GetBaseRFIDStatus();

    inline RefereeDataStatus GetHighland_2_SelfRfidStatus();

    inline RefereeDataStatus GetHighland_2_EnemyRfidStatus();

    inline RefereeDataStatus GetHighland_3_SelfRfidStatus();

    inline RefereeDataStatus GetHighland_3_EnemyRfidStatus();

    inline RefereeDataStatus GetHighland_4_SelfRfidStatus();

    inline RefereeDataStatus GetHighland_4_EnemyRfidStatus();

    inline RefereeDataStatus GetEnergyRfidStatus();

    inline RefereeDataStatus GetFlyover_1_SelfRfidStatus();

    inline RefereeDataStatus GetFlyover_2_SelfRfidStatus();

    inline RefereeDataStatus GetFlyover_1_EnemyRfidStatus();

    inline RefereeDataStatus GetFlyover_2_EnemyRfidStatus();

    inline RefereeDataStatus GetOutpostRfidStatus();

    inline RefereeDataStatus GetHpRfidStatus();

    inline RefereeDataStatus GetSentrySelfRfidStatus();

    inline RefereeDataStatus GetSentryEnemyRfidStatus();

    inline RefereeDataStatus GetEngineerSelfRfidStatus();

    inline RefereeDataStatus GetEngineerEnemyRfidStatus();

    inline RefereeDataStatus GetEngineerExchangeRfidStatus();

    inline RefereeDataStatus GetCenterRfidStatus();

    inline RefereeDataRobotDartCommandStatus GetDartCommandStatus();

    inline uint16_t GetDartSwitchRemainingTime();

    inline uint16_t GetDartLaunchRemainingTime();

    inline float GetSentryLocationHero_1_X();

    inline float GetSentryLocationHero_1_Y();

    inline float GetSentryLocationEngineer_2_X();

    inline float GetSentryLocationEngineer_2_Y();

    inline float GetSentryLocationInfantry_3_X();

    inline float GetSentryLocationInfantry_3_Y();

    inline float GetSentryLocationInfantry_4_X();

    inline float GetSentryLocationInfantry_4_Y();

    inline float GetSentryLocationInfantry_5_X();

    inline float GetSentryLocationInfantry_5_Y();

    inline uint8_t GetRadarMarkStatusHero_1();

    inline uint8_t GetRadarMarkStatusEngineer_2();

    inline uint8_t GetRadarMarkStatusInfantry_3();

    inline uint8_t GetRadarMarkStatusInfantry_4();

    inline uint8_t GetRadarMarkStatusInfantry_5();

    inline uint8_t GetRadarMarkStatusSentry_7();

    inline uint16_t GetSentryDecisionAmmoExchangeNumber();

    inline uint8_t GetSentryDecisionAmmoExchangeTime();

    inline uint8_t GetSentryDecisionHpExchangeTime();

    inline uint8_t GetRadarDecisionDoubleDamageChance();

    inline RefereeDataStatus GetRadarDecisionDoubleDamageEnemyStatus();

    inline void SetRefereeTrustStatus(RefereeDataStatus __Referee_Trust_Status);

    inline void SetRefereeUiChangeFlagClear();

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiClear(uint8_t Layer_Num, uint8_t Graphic_Num);

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiLine(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y);

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiRectangle(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y);

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiCircle(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Radius);

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiOval(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Length_X, uint32_t Length_Y);

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiArc(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Angle_Start, uint32_t Angle_End, uint32_t Length_X, uint32_t Length_Y);

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiFloat(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, float Float);

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiInteger(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, int32_t Integer);

    inline RefereeDataInteractionGraphicConfig *SetRefereeUiString(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, uint32_t String_Length);

    void UartSendInteractionUiLayerDelete(RefereeDataInteractionLayerDeleteOperation Layer_Delete_Operation, uint8_t Layer);

    void UartSendInteractionUiGraphic1(RefereeDataInteractionGraphicConfig *Graphic_1);

    void UartSendInteractionUiGraphic2(RefereeDataInteractionGraphicConfig *Graphic_1, RefereeDataInteractionGraphicConfig *Graphic_2);

    void UartSendInteractionUiGraphic5(RefereeDataInteractionGraphicConfig *Graphic_1, RefereeDataInteractionGraphicConfig *Graphic_2, RefereeDataInteractionGraphicConfig *Graphic_3, RefereeDataInteractionGraphicConfig *Graphic_4, RefereeDataInteractionGraphicConfig *Graphic_5);

    void UartSendInteractionUiGraphic7(RefereeDataInteractionGraphicConfig *Graphic_1, RefereeDataInteractionGraphicConfig *Graphic_2, RefereeDataInteractionGraphicConfig *Graphic_3, RefereeDataInteractionGraphicConfig *Graphic_4, RefereeDataInteractionGraphicConfig *Graphic_5, RefereeDataInteractionGraphicConfig *Graphic_6, RefereeDataInteractionGraphicConfig *Graphic_7);

    void UartSendInteractionUiGraphicString(RefereeDataInteractionGraphicConfig *Graphic_String, const char *String_Content);

    void UartRxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    void AlivePeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的UART
    UartManageObject *uart_manage_object_;
    // 数据包头标
    uint8_t frame_header_;

    // 常量

    // 内部变量

    // 当前时刻的裁判系统接收flag
    uint32_t flag_ = 0;
    // 前一时刻的裁判系统接收flag
    uint32_t pre_flag_ = 0;

    // 发送序列号
    uint8_t sequence_ = 0;

    // UI是否是初次绘制, 没绘制过是0
    uint8_t ui_change_flag_[10][10] = {0};

    // 读变量

    // 裁判系统状态
    RefereeStatus referee_status_ = Referee_Status_DISABLE;
    // 比赛状态
    RefereeRxDataGameStatus game_status_;
    // 比赛结果
    RefereeRxDataGameResult game_result_;
    // 机器人血量
    RefereeRxDataGameRobotHP game_robot_hp_;

    // 场地事件
    RefereeRxDataEventSelfData event_self_data_;
    // 补给站状态
    RefereeRxDataEventSelfSupply event_self_supply_;
    // 裁判警告信息
    RefereeRxDataEventRefereeWarning event_referee_warning_;
    // 飞镖15s倒计时
    RefereeRxDataEventDartStatus event_dart_status_;

    // 机器人状态
    RefereeRxDataRobotStatus robot_status_;
    // 当前机器人实时功率热量
    RefereeRxDataRobotPowerHeat robot_power_heat_;
    // 当前机器人实时位置
    RefereeRxDataRobotPosition robot_position_;
    // 当前机器人增益
    RefereeRxDataRobotBuff robot_buff_;
    // // 无人机可攻击时间
    // RefereeRxDataRobotAerialStatus robot_aerial_status_;
    // 伤害情况
    RefereeRxDataRobotDamage robot_damage_;
    // 子弹信息
    RefereeRxDataRobotBooster robot_booster_;
    // 子弹剩余信息
    RefereeRxDataRobotRemainingAmmo robot_remaining_ammo_;
    // RFID状态信息
    RefereeRxDataRobotRFID robot_rfid_;
    // 飞镖状态
    RefereeRxDataRobotDartCommand robot_dart_command_;
    // 哨兵获取己方位置信息
    RefereeRxDataRobotSentryLocation robot_sentry_location_;
    // 雷达标记进度
    RefereeRxDataRobotRadarMark robot_radar_mark_;
    // 哨兵决策信息
    RefereeRxDataRobotSentryDecision robot_sentry_decision_;
    // 雷达决策信息
    RefereeRxDataRobotRadarDecision robot_radar_decision_;

    // 写变量

    RefereeDataInteractionGraphicConfig graphic_config_[10][10];

    // 读写变量

    // 裁判系统是否可信
    RefereeDataStatus referee_trust_status_ = Referee_Data_Status_ENABLE;

    // 内部函数

    uint8_t VerifyCrc8(uint8_t *Message, uint32_t Length);

    uint16_t VerifyCrc16(uint8_t *Message, uint32_t Length);

    void DataProcess(uint16_t Length);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取裁判系统状态
 *
 * @return RefereeStatus 裁判系统状态
 */
inline RefereeStatus Referee::GetStatus()
{
    return (referee_status_);
}

/**
 * @brief 获取裁判系统可信状态
 *
 * @return RefereeDataStatus 裁判系统可信状态
 */
inline RefereeDataStatus Referee::GetRefereeTrustStatus()
{
    return (referee_trust_status_);
}

/**
 * @brief 获取比赛类型
 *
 * @return RefereeGameStatusType 比赛类型
 */
inline RefereeGameStatusType Referee::GetGameType()
{
    return (static_cast<RefereeGameStatusType>(game_status_.Type_Enum));
}

/**
 * @brief 获取比赛阶段
 *
 * @return RefereeGameStatusStage 比赛阶段
 */
inline RefereeGameStatusStage Referee::GetGameStage()
{
    return (static_cast<RefereeGameStatusStage>(game_status_.Stage_Enum));
}

/**
 * @brief 获取当前阶段剩余时间
 *
 * @return uint16_t 当前阶段剩余时间
 */
inline uint16_t Referee::GetRemainingTime()
{
    return (game_status_.Remaining_Time);
}

/**
 * @brief 获取系统时间戳
 *
 * @return uint64_t 系统时间戳
 */
inline uint64_t Referee::GetTimestamp()
{
    return (game_status_.Timestamp);
}

/**
 * @brief 获取比赛结果
 *
 * @return RefereeGameResult 比赛结果
 */
inline RefereeGameResult Referee::GetResult()
{
    return (game_result_.Result);
}

/**
 * @brief 获取机器人血量
 *
 * @param Robots_ID 通用双方机器人ID
 * @return uint16_t 机器人血量
 */
inline uint16_t Referee::GetHP(RefereeDataRobotsID Robots_ID)
{
    switch (Robots_ID)
    {
    case (Referee_Data_Robots_ID_RED_HERO_1):
    {
        return (game_robot_hp_.Red_Hero_1);
    }
        break;
    case (Referee_Data_Robots_ID_RED_ENGINEER_2):
    {
        return (game_robot_hp_.Red_Engineer_2);
    }
        break;
    case (Referee_Data_Robots_ID_RED_INFANTRY_3):
    {
        return (game_robot_hp_.Red_Infantry_3);
    }
        break;
    case (Referee_Data_Robots_ID_RED_INFANTRY_4):
    {
        return (game_robot_hp_.Red_Infantry_4);
    }
        break;
    case (Referee_Data_Robots_ID_RED_INFANTRY_5):
    {
        return (game_robot_hp_.Red_Infantry_5);
    }
        break;
    case (Referee_Data_Robots_ID_RED_SENTRY_7):
    {
        return (game_robot_hp_.Red_Sentry_7);
    }
        break;
    case (Referee_Data_Robots_ID_RED_OUTPOST_11):
    {
        return (game_robot_hp_.Red_Outpost_11);
    }
        break;
    case (Referee_Data_Robots_ID_RED_BASE_10):
    {
        return (game_robot_hp_.Red_Base_10);
    }
        break;
    case (Referee_Data_Robots_ID_BLUE_HERO_1):
    {
        return (game_robot_hp_.Blue_Hero_1);
    }
        break;
    case (Referee_Data_Robots_ID_BLUE_ENGINEER_2):
    {
        return (game_robot_hp_.Blue_Engineer_2);
    }
        break;
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_3):
    {
        return (game_robot_hp_.Blue_Infantry_3);
    }
        break;
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_4):
    {
        return (game_robot_hp_.Blue_Infantry_4);
    }
        break;
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_5):
    {
        return (game_robot_hp_.Blue_Infantry_5);
    }
        break;
    case (Referee_Data_Robots_ID_BLUE_SENTRY_7):
    {
        return (game_robot_hp_.Blue_Sentry_7);
    }
        break;
    case (Referee_Data_Robots_ID_BLUE_OUTPOST_11):
    {
        return (game_robot_hp_.Blue_Outpost_11);
    }
        break;
    case (Referee_Data_Robots_ID_BLUE_BASE_10):
    {
        return (game_robot_hp_.Blue_Base_10);
    }
        break;
    }
}

/**
 * @brief 获取补给站前占领状态
 *
 * @return RefereeDataStatus 补给站前占领状态
 */
inline RefereeDataStatus Referee::GetSupplyFrontStatus()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Supply_Front_Status_Enum));
}

/**
 * @brief 获取补给站内占领状态
 *
 * @return RefereeDataStatus 补给站内占领状态
 */
inline RefereeDataStatus Referee::GetSupplyInnerStatus()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Supply_Inner_Status_Enum));
}

/**
 * @brief 获取补给站占领状态
 *
 * @return RefereeDataStatus 补给站占领状态
 */
inline RefereeDataStatus Referee::GetSupplyStatus()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Supply_Status_Enum));
}

/**
 * @brief 获取能量机关占领状态
 *
 * @return RefereeDataStatus 能量机关占领状态
 */
inline RefereeDataStatus Referee::GetPowerRuneStatus()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Power_Rune_Status_Enum));
}

/**
 * @brief 获取小能量机关激活状态
 *
 * @return RefereeDataStatus 小能量机关激活状态
 */
inline RefereeDataStatus Referee::GetPowerRuneSmallStatus()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Power_Rune_Small_Status_Enum));
}

/**
 * @brief 获取大能量机关激活状态
 *
 * @return RefereeDataStatus 大能量机关激活状态
 */
inline RefereeDataStatus Referee::GetPowerRuneBigStatus()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Power_Rune_Big_Status_Enum));
}

/**
 * @brief 获取环形高地占领状态
 *
 * @return RefereeDataStatus 环形高地占领状态
 */
inline RefereeDataStatus Referee::GetHighland2Status()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Highland_2_Status_Enum));
}

/**
 * @brief 获取梯形高地3占领状态
 *
 * @return RefereeDataStatus 梯形高地3占领状态
 */
inline RefereeDataStatus Referee::GetHighland3Status()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Highland_3_Status_Enum));
}

/**
 * @brief 获取梯形高地4占领状态
 *
 * @return RefereeDataStatus 梯形高地4占领状态
 */
inline RefereeDataStatus Referee::GetHighland4Status()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Highland_4_Status_Enum));
}

/**
 * @brief 获取基地护盾状态
 *
 * @return uint8_t 基地护盾状态
 */
inline uint8_t Referee::GetBaseVirtualShieldPercent()
{
    return (event_self_data_.Base_Virtual_Shield_Percent);
}

/**
 * @brief 获取敌方飞镖最后一次命中己方时间
 *
 * @return uint16_t 敌方飞镖最后一次命中己方时间
 */
inline uint16_t Referee::GetEnemyDartHitTime()
{
    return (event_self_data_.Enemy_Dart_Hit_Time);
}

/**
 * @brief 获取敌方飞镖最后一次命中己方建筑物
 *
 * @return RefereeDartHitTarget 敌方飞镖最后一次命中己方建筑物
 */
inline RefereeDartHitTarget Referee::GetEnemyDartHitTargetEnum()
{
    return (static_cast<RefereeDartHitTarget>(event_self_data_.Enemy_Dart_Hit_Target_Enum));
}

/**
 * @brief 获取中心增益点占领状态
 *
 * @return RefereeDataStatus 中心增益点占领状态
 */
inline RefereeDataStatus Referee::GetMiddleBuffStatus()
{
    return (static_cast<RefereeDataStatus>(event_self_data_.Middle_Buff_Status_Enum));
}

/**
 * @brief 获取请求补给的机器人ID
 *
 * @return RefereeDataRobotsID 请求补给的机器人ID
 */
inline RefereeDataRobotsID Referee::GetSupplyRequestRobot()
{
    return (event_self_supply_.Robot);
}

/**
 * @brief 获取补给站的补给状态
 *
 * @return RefereeDataEventSupplyStatus 补给站的补给状态
 */
inline RefereeDataEventSupplyStatus Referee::GetSupplyRequestStatus()
{
    return (event_self_supply_.Status);
}

/**
 * @brief 获取补给子弹数量
 *
 * @return RefereeDataEventSupplyAmmoNumber 补给子弹数量
 */
inline RefereeDataEventSupplyAmmoNumber Referee::GetSupplyAmmoNumber()
{
    return (event_self_supply_.Ammo_Number);
}

/**
 * @brief 获取裁判判罚信息
 *
 * @return RefereeDataEventRefereeWarningLevel 裁判判罚信息
 */
inline RefereeDataEventRefereeWarningLevel Referee::GetRefereeWarning()
{
    return (event_referee_warning_.Level);
}

/**
 * @brief 获取裁判判罚机器人
 *
 * @return RefereeDataRobotID 裁判判罚机器人
 */
inline RefereeDataRobotID Referee::GetRefereeWarningRobot()
{
    return (event_referee_warning_.Robot_ID);
}

/**
 * @brief 获取飞镖剩余时间
 *
 * @return uint8_t 飞镖剩余时间
 */
inline uint8_t Referee::GetDartRemainingTime()
{
    return (event_dart_status_.Dart_Remaining_Time);
}

/**
 * @brief 获取飞镖上一次击中的目标
 *
 * @return RefereeDartHitTarget 飞镖上一次击中的目标
 */
inline RefereeDartHitTarget Referee::GetLastDartHitTarget()
{
    return (static_cast<RefereeDartHitTarget>(event_dart_status_.Dart_Hit_Target_Enum_Last));
}

/**
 * @brief 获取飞镖上一次击中目标的次数
 *
 * @return uint8_t 飞镖上一次击中目标的次数
 */
inline uint8_t Referee::GetDartHitTargetCount()
{
    return (event_dart_status_.Dart_Hit_Target_Count);
}

/**
 * @brief 获取飞镖当前的目标
 *
 * @return RefereeDartHitTarget 飞镖当前的目标
 */
inline RefereeDartHitTarget Referee::GetNowDartHitTarget()
{
    return (static_cast<RefereeDartHitTarget>(event_dart_status_.Dart_Hit_Target_Enum_Now));
}

/**
 * @brief 获取自身ID
 *
 * @return RefereeDataRobotsID 自身ID
 */
inline RefereeDataRobotsID Referee::GetSelfID()
{
    return (robot_status_.Robot_ID);
}

/**
 * @brief 获取自身等级
 *
 * @return uint8_t 自身等级
 */
inline uint8_t Referee::GetSelfLevel()
{
    return (robot_status_.Level);
}

/**
 * @brief 获取自身血量
 *
 * @return uint16_t 自身血量
 */
inline uint16_t Referee::GetSelfHP()
{
    return (robot_status_.HP);
}

/**
 * @brief 获取自身血量上限
 *
 * @return uint16_t 自身血量上限
 */
inline uint16_t Referee::GetSelfHPMax()
{
    return (robot_status_.HP_Max);
}

/**
 * @brief 获取发射机构冷却
 *
 * @return uint16_t 发射机构冷却
 */
inline uint16_t Referee::GetSelfBoosterHeatCD()
{
    return (robot_status_.Booster_Heat_CD);
}

/**
 * @brief 获取发射机构热量上限
 *
 * @return uint16_t 发射机构热量上限
 */
inline uint16_t Referee::GetSelfBoosterHeatMax()
{
    return (robot_status_.Booster_Heat_Max);
}

/**
 * @brief 获取底盘功率上限
 *
 * @return uint16_t 底盘功率上限
 */
inline uint16_t Referee::GetSelfChassisPowerMax()
{
    return (robot_status_.Chassis_Power_Max);
}

/**
 * @brief 获取Gimbal供电状态
 *
 * @return RefereeDataStatus Gimbal供电状态
 */
inline RefereeDataStatus Referee::GetSelfPM01GimbalStatus()
{
    return (static_cast<RefereeDataStatus>(robot_status_.PM01_Gimbal_Status_Enum));
}

/**
 * @brief 获取Chassis供电状态
 *
 * @return RefereeDataStatus Chassis供电状态
 */
inline RefereeDataStatus Referee::GetSelfPM01ChassisStatus()
{
    return (static_cast<RefereeDataStatus>(robot_status_.PM01_Chassis_Status_Enum));
}

/**
 * @brief 获取Booster供电状态
 *
 * @return RefereeDataStatus Booster供电状态
 */
inline RefereeDataStatus Referee::GetSelfPM01BoosterStatus()
{
    return (static_cast<RefereeDataStatus>(robot_status_.PM01_Booster_Status_Enum));
}

// /**
//  * @brief 获取底盘电压
//  *
//  * @return float 底盘电压
//  */
// inline float Referee::GetChassisVoltage()
// {
//     return (robot_power_heat_.Chassis_Voltage / 1000.0f);
// }

// /**
//  * @brief 获取底盘电流
//  *
//  * @return float 底盘电流
//  */
// inline float Referee::GetChassisCurrent()
// {
//     return (robot_power_heat_.Chassis_Current / 1000.0f);
// }

// /**
//  * @brief 获取底盘功率
//  *
//  * @return float 底盘功率
//  */
// inline float Referee::GetChassisPower()
// {
//     return (robot_power_heat_.Chassis_Power);
// }

/**
 * @brief 获取底盘能量缓冲
 *
 * @return uint16_t 底盘能量缓冲
 */
inline uint16_t Referee::GetChassisEnergyBuffer()
{
    return (robot_power_heat_.Chassis_Energy_Buffer);
}

/**
 * @brief 获取17mm1热量
 *
 * @return uint16_t 17mm1热量
 */
inline uint16_t Referee::GetBooster17mm_1_Heat()
{
    return (robot_power_heat_.Booster_17mm_1_Heat);
}

/**
 * @brief 获取17mm2热量
 *
 * @return uint16_t 17mm2热量
 */
inline uint16_t Referee::GetBooster17mm_2_Heat()
{
    return (robot_power_heat_.Booster_17mm_2_Heat);
}

/**
 * @brief 获取42mm热量
 *
 * @return uint16_t 42mm热量
 */
inline uint16_t Referee::GetBooster42mmHeat()
{
    return (robot_power_heat_.Booster_42mm_Heat);
}

/**
 * @brief 获取自身位置x
 *
 * @return float 自身位置x
 */
inline float Referee::GetLocationX()
{
    return (robot_position_.Location_X);
}

/**
 * @brief 获取自身位置y
 *
 * @return float 自身位置y
 */
inline float Referee::GetLocationY()
{
    return (robot_position_.Location_Y);
}

/**
 * @brief 获取自身方向yaw
 *
 * @return float 自身方向yaw
 */
inline float Referee::GetLocationYaw()
{
    return (robot_position_.Location_Yaw);
}

/**
 * @brief 获取补血buff百分比
 *
 * @return uint8_t 补血buff百分比
 */
inline uint8_t Referee::GetHPBuffPercent()
{
    return (robot_buff_.HP_Buff_Percent);
}

/**
 * @brief 获取冷却缩减buff倍数
 *
 * @return uint8_t 冷却缩减buff倍数
 */
inline uint8_t Referee::GetBoosterHeatCDBuffValue()
{
    return (robot_buff_.Booster_Heat_CD_Buff_Value);
}

/**
 * @brief 获取防御加成buff百分比
 *
 * @return uint8_t 防御加成buff百分比
 */
inline uint8_t Referee::GetDefendBuffPercent()
{
    return (robot_buff_.Defend_Buff_Percent);
}

/**
 * @brief 获取负防御加成buff百分比
 *
 * @return uint8_t 负防御加成buff百分比
 */
inline uint8_t Referee::GetDefendDebuffPercent()
{
    return (robot_buff_.Defend_Buff_Percent);
}

/**
 * @brief 获取攻击加成buff状态
 *
 * @return uint8_t 攻击加成buff状态
 */
inline uint8_t Referee::GetDamageBuffPercent()
{
    return (robot_buff_.Damage_Buff_Percent);
}

// /**
//  * @brief 获取无人机状态
//  *
//  * @return RefereeDataEventAerialStatus 无人机状态
//  */
// inline RefereeDataEventAerialStatus Referee::GetAerialStatus()
// {
//     return (robot_aerial_status_.Aerial_Status);
// }

// /**
//  * @brief 获取无人机时间
//  *
//  * @return uint8_t 无人机时间
//  */
// inline uint8_t Referee::GetAerialRemainingTime()
// {
//     return (robot_aerial_status_.Remaining_Time);
// }

/**
 * @brief 获取受击装甲板ID
 *
 * @return uint8_t 受击装甲板ID
 */
inline uint8_t Referee::GetArmorAttackedID()
{
    return (robot_damage_.Armor_ID);
}

/**
 * @brief 获取受击类型
 *
 * @return RefereeDataEventRobotDamageType 受击类型
 */
inline RefereeDataEventRobotDamageType Referee::GetAttackedType()
{
    return (static_cast<RefereeDataEventRobotDamageType>(robot_damage_.Type_Enum));
}

/**
 * @brief 获取射击子弹类型
 *
 * @return RefereeDataRobotAmmoType 射击子弹类型
 */
inline RefereeDataRobotAmmoType Referee::GetShootAmmoType()
{
    return (robot_booster_.Ammo_Type);
}

/**
 * @brief 获取发射机构类型
 *
 * @return RefereeDataRobotBoosterType 发射机构类型
 */
inline RefereeDataRobotBoosterType Referee::GetShootBoosterType()
{
    return (robot_booster_.Booster_Type);
}

/**
 * @brief 获取射频, Hz
 *
 * @return uint8_t 射频, Hz
 */
inline uint8_t Referee::GetShootFrequency()
{
    return (robot_booster_.Frequency);
}

/**
 * @brief 获取射速
 *
 * @return float 射速
 */
inline float Referee::GetShootSpeed()
{
    return (robot_booster_.Speed);
}

/**
 * @brief 获取17mm弹丸剩余数
 *
 * @return uint16_t 17mm弹丸剩余数
 */
inline uint16_t Referee::Get17mmRemaining()
{
    return (robot_remaining_ammo_.Booster_17mm);
}

/**
 * @brief 获取42mm弹丸剩余数
 *
 * @return uint16_t 42mm弹丸剩余数
 */
inline uint16_t Referee::Get42mmRemaining()
{
    return (robot_remaining_ammo_.Booster_42mm);
}

/**
 * @brief 获取金币剩余数
 *
 * @return uint16_t 金币剩余数
 */
inline uint16_t Referee::GetMoneyRemaining()
{
    return (robot_remaining_ammo_.Money);
}

/**
 * @brief 获取基地增益RFID状态
 *
 * @return RefereeDataStatus 基地增益RFID状态
 */
inline RefereeDataStatus Referee::GetBaseRFIDStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Base_Status_Enum));
}

/**
 * @brief 获取己方环形高地RFID状态
 *
 * @return RefereeDataStatus 己方环形高地RFID状态
 */
inline RefereeDataStatus Referee::GetHighland_2_SelfRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Highland_2_Self_Status_Enum));
}

/**
 * @brief 获取敌方环形高地RFID状态
 *
 * @return RefereeDataStatus 敌方环形高地RFID状态
 */
inline RefereeDataStatus Referee::GetHighland_2_EnemyRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Highland_2_Enemy_Status_Enum));
}

/**
 * @brief 获取己方高地3RFID状态
 *
 * @return RefereeDataStatus 己方环形高地3RFID状态
 */
inline RefereeDataStatus Referee::GetHighland_3_SelfRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Highland_3_Self_Status_Enum));
}

/**
 * @brief 获取敌方高地3RFID状态
 *
 * @return RefereeDataStatus 敌方高地3RFID状态
 */
inline RefereeDataStatus Referee::GetHighland_3_EnemyRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Highland_3_Enemy_Status_Enum));
}

// /**
//  * @brief 获取己方高地4RFID状态
//  *
//  * @return RefereeDataStatus 己方环形高地4RFID状态
//  */
// inline RefereeDataStatus Referee::GetHighland_4_SelfRfidStatus()
// {
//     return (static_cast<RefereeDataStatus>(robot_rfid_.Highland_4_Self_Status_Enum));
// }

// /**
//  * @brief 获取敌方高地4RFID状态
//  *
//  * @return RefereeDataStatus 敌方高地4RFID状态
//  */
// inline RefereeDataStatus Referee::GetHighland_4_EnemyRfidStatus()
// {
//     return (static_cast<RefereeDataStatus>(robot_rfid_.Highland_4_Enemy_Status_Enum));
// }

// /**
//  * @brief 获取能量机关增益RFID状态
//  *
//  * @return RefereeDataStatus 能量机关增益RFID状态
//  */
// inline RefereeDataStatus Referee::GetEnergyRfidStatus()
// {
//     return (static_cast<RefereeDataStatus>(robot_rfid_.Energy_Status_Enum));
// }

/**
 * @brief 获取己方1阶段飞坡增益RFID状态
 *
 * @return RefereeDataStatus 己方1阶段飞坡增益RFID状态
 */
inline RefereeDataStatus Referee::GetFlyover_1_SelfRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Flyover_1_Self_Status_Enum));
}

/**
 * @brief 获取己方2阶段飞坡增益RFID状态
 *
 * @return RefereeDataStatus 己方2阶段飞坡增益RFID状态
 */
inline RefereeDataStatus Referee::GetFlyover_2_SelfRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Flyover_2_Self_Status_Enum));
}

/**
 * @brief 获取敌方1阶段飞坡增益RFID状态
 *
 * @return RefereeDataStatus 敌方1阶段飞坡增益RFID状态
 */
inline RefereeDataStatus Referee::GetFlyover_1_EnemyRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Flyover_1_Enemy_Status_Enum));
}

/**
 * @brief 获取敌方2阶段飞坡增益RFID状态
 *
 * @return RefereeDataStatus 敌方2阶段飞坡增益RFID状态
 */
inline RefereeDataStatus Referee::GetFlyover_2_EnemyRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Flyover_2_Enemy_Status_Enum));
}

// /**
//  * @brief 获取前哨站增益RFID状态
//  *
//  * @return RefereeDataStatus 前哨站增益RFID状态
//  */
// inline RefereeDataStatus Referee::GetOutpostRfidStatus()
// {
//     return (static_cast<RefereeDataStatus>(robot_rfid_.Outpost_Status_Enum));
// }

// /**
//  * @brief 获取补血点增益RFID状态
//  *
//  * @return RefereeDataStatus 补血点增益RFID状态
//  */
// inline RefereeDataStatus Referee::GetHpRfidStatus()
// {
//     return (static_cast<RefereeDataStatus>(robot_rfid_.HP_Status_Enum));
// }

// /**
//  * @brief 获取己方哨兵巡逻区增益RFID状态
//  *
//  * @return RefereeDataStatus 己方哨兵巡逻区增益RFID状态
//  */
// inline RefereeDataStatus Referee::GetSentrySelfRfidStatus()
// {
//     return (static_cast<RefereeDataStatus>(robot_rfid_.Sentry_Self_Status_Enum));
// }

// /**
//  * @brief 获取敌方哨兵巡逻区增益RFID状态
//  *
//  * @return RefereeDataStatus 敌方哨兵巡逻区增益RFID状态
//  */
// inline RefereeDataStatus Referee::GetSentryEnemyRfidStatus()
// {
//     return (static_cast<RefereeDataStatus>(robot_rfid_.Sentry_Enemy_Status_Enum));
// }

/**
 * @brief 获取己方工程采矿区增益RFID状态
 *
 * @return RefereeDataStatus 己方工程采矿区增益RFID状态
 */
inline RefereeDataStatus Referee::GetEngineerSelfRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Engineer_Self_Status_Enum));
}

/**
 * @brief 获取敌方工程采矿区增益RFID状态
 *
 * @return RefereeDataStatus 敌方工程采矿区增益RFID状态
 */
inline RefereeDataStatus Referee::GetEngineerEnemyRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Engineer_Enemy_Status_Enum));
}

// /**
//  * @brief 获取工程兑换站增益RFID状态
//  *
//  * @return RefereeDataStatus 工程兑换站增益RFID状态
//  */
// inline RefereeDataStatus Referee::GetEngineerExchangeRfidStatus()
// {
//     return (static_cast<RefereeDataStatus>(robot_rfid_.Engineer_Exchange_Status_Enum));
// }

/**
 * @brief 获取中心增益点RFID状态
 *
 * @return RefereeDataStatus 中心增益点RFID状态
 */
inline RefereeDataStatus Referee::GetCenterRfidStatus()
{
    return (static_cast<RefereeDataStatus>(robot_rfid_.Center_Status_Enum));
}

/**
 * @brief 获取飞镖发射口状态
 *
 * @return RefereeDataRobotDartCommandStatus 飞镖发射口状态
 */
inline RefereeDataRobotDartCommandStatus Referee::GetDartCommandStatus()
{
    return (robot_dart_command_.Status);
}

/**
 * @brief 获取飞镖切换击打目标时的比赛剩余时间
 *
 * @return uint16_t 飞镖切换击打目标时的比赛剩余时间
 */
inline uint16_t Referee::GetDartSwitchRemainingTime()
{
    return (robot_dart_command_.Switch_Remaining_Time);
}

/**
 * @brief 获取最后一次操作手下达发射指令时的比赛剩余时间
 *
 * @return uint16_t 最后一次操作手下达发射指令时的比赛剩余时间
 */
inline uint16_t Referee::GetDartLaunchRemainingTime()
{
    return (robot_dart_command_.Launch_Remaining_Time);
}

/**
 * @brief 获取己方英雄坐标x
 *
 * @return float 己方英雄坐标x
 */
inline float Referee::GetSentryLocationHero_1_X()
{
    return (robot_sentry_location_.Hero_1_X);
}

/**
 * @brief 获取己方英雄坐标y
 *
 * @return float 己方英雄坐标y
 */
inline float Referee::GetSentryLocationHero_1_Y()
{
    return (robot_sentry_location_.Hero_1_Y);
}

/**
 * @brief 获取己方工程坐标x
 *
 * @return float 己方工程坐标x
 */
inline float Referee::GetSentryLocationEngineer_2_X()
{
    return (robot_sentry_location_.Engineer_2_X);
}

/**
 * @brief 获取己方工程坐标y
 *
 * @return float 己方工程坐标y
 */
inline float Referee::GetSentryLocationEngineer_2_Y()
{
    return (robot_sentry_location_.Engineer_2_Y);
}

/**
 * @brief 获取己方步兵坐标x
 *
 * @return float 己方步兵坐标x
 */
inline float Referee::GetSentryLocationInfantry_3_X()
{
    return (robot_sentry_location_.Infantry_3_X);
}

/**
 * @brief 获取己方步兵坐标y
 *
 * @return float 己方步兵坐标y
 */
inline float Referee::GetSentryLocationInfantry_3_Y()
{
    return (robot_sentry_location_.Infantry_3_Y);
}

/**
 * @brief 获取己方步兵坐标x
 *
 * @return float 己方步兵坐标x
 */
inline float Referee::GetSentryLocationInfantry_4_X()
{
    return (robot_sentry_location_.Infantry_4_X);
}

/**
 * @brief 获取己方步兵坐标y
 *
 * @return float 己方步兵坐标y
 */
inline float Referee::GetSentryLocationInfantry_4_Y()
{
    return (robot_sentry_location_.Infantry_4_Y);
}

/**
//  * @brief 获取己方步兵坐标x
//  *
//  * @return float 己方步兵坐标x
//  */
// inline float Referee::GetSentryLocationInfantry_5_X()
// {
//     return (robot_sentry_location_.Infantry_5_X);
// }

// /**
//  * @brief 获取己方步兵坐标y
//  *
//  * @return float 己方步兵坐标y
//  */
// inline float Referee::GetSentryLocationInfantry_5_Y()
// {
//     return (robot_sentry_location_.Infantry_5_Y);
// }

/**
 * @brief 获取雷达对敌方英雄标记进度
 *
 * @return uint8_t 雷达对敌方英雄标记进度
 */
inline uint8_t Referee::GetRadarMarkStatusHero_1()
{
    return (robot_radar_mark_.Hero_1_Mark);
}

/**
 * @brief 获取雷达对敌方工程标记进度
 *
 * @return uint8_t 雷达对敌方工程标记进度
 */
inline uint8_t Referee::GetRadarMarkStatusEngineer_2()
{
    return (robot_radar_mark_.Engineer_2_Mark);
}

/**
 * @brief 获取雷达对敌方步兵标记进度
 *
 * @return uint8_t 雷达对敌方步兵标记进度
 */
inline uint8_t Referee::GetRadarMarkStatusInfantry_3()
{
    return (robot_radar_mark_.Infantry_3_Mark);
}

/**
 * @brief 获取雷达对敌方步兵标记进度
 *
 * @return uint8_t 雷达对敌方步兵标记进度
 */
inline uint8_t Referee::GetRadarMarkStatusInfantry_4()
{
    return (robot_radar_mark_.Infantry_4_Mark);
}

// /**
//  * @brief 获取雷达对敌方步兵标记进度
//  *
//  * @return uint8_t 雷达对敌方步兵标记进度
//  */
// inline uint8_t Referee::GetRadarMarkStatusInfantry_5()
// {
//     return (robot_radar_mark_.Infantry_5_Mark);
// }

/**
 * @brief 获取雷达对敌方哨兵标记进度
 *
 * @return uint8_t 雷达对敌方哨兵标记进度
 */
inline uint8_t Referee::GetRadarMarkStatusSentry_7()
{
    return (robot_radar_mark_.Sentry_7_Mark);
}

/**
 * @brief 获取哨兵决策购买子弹数量
 *
 * @return uint16_t 哨兵决策购买子弹数量
 */
inline uint16_t Referee::GetSentryDecisionAmmoExchangeNumber()
{
    return (robot_sentry_decision_.Ammo_Exchange_Number);
}

/**
 * @brief 获取哨兵决策购买子弹次数
 *
 * @return uint8_t 哨兵决策购买子弹次数
 */
inline uint8_t Referee::GetSentryDecisionAmmoExchangeTime()
{
    return (robot_sentry_decision_.Ammo_Exchange_Time);
}

/**
 * @brief 获取哨兵决策购买血量次数
 *
 * @return uint8_t 哨兵决策购买血量次数
 */
inline uint8_t Referee::GetSentryDecisionHpExchangeTime()
{
    return (robot_sentry_decision_.HP_Exchange_Time);
}

/**
 * @brief 获取雷达决策重伤敌方机会
 *
 * @return uint8_t 雷达决策重伤敌方机会
 */
inline uint8_t Referee::GetRadarDecisionDoubleDamageChance()
{
    return (robot_radar_decision_.Double_Damage_Chance);
}

/**
 * @brief 获取雷达决策重伤敌方状态
 *
 * @return RefereeDataStatus 雷达决策重伤敌方状态
 */
inline RefereeDataStatus Referee::GetRadarDecisionDoubleDamageEnemyStatus()
{
    return (static_cast<RefereeDataStatus>(robot_radar_decision_.Double_Damage_Enemy_Status_Enum));
}

/**
 * @brief 设定裁判系统是否可信
 *
 * @return RefereeDataStatus 裁判系统是否可信
 */
inline void Referee::SetRefereeTrustStatus(RefereeDataStatus __Referee_Trust_Status)
{
    referee_trust_status_ = __Referee_Trust_Status;
}

/**
 * @brief 设定裁判系统UI清除
 *
 */
inline void Referee::SetRefereeUiChangeFlagClear()
{
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            ui_change_flag_[i][j] = 0;
        }
    }
}

/**
 * @brief 设定裁判系统UI清除
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiClear(uint8_t Layer_Num, uint8_t Graphic_Num)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;

    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_DELETE;

    ui_change_flag_[Layer_Num][Graphic_Num] = 0;

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

/**
 * @brief 设定裁判系统UI直线
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param End_X 终点x
 * @param End_Y 终点y
 * @return 对应图层指针
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiLine(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (ui_change_flag_[Layer_Num][Graphic_Num] == 0)
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ui_change_flag_[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    graphic_config_[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_LINE;
    graphic_config_[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Color_Enum = Color;
    graphic_config_[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    graphic_config_[Layer_Num][Graphic_Num].Start_X = Start_X;
    graphic_config_[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    graphic_config_[Layer_Num][Graphic_Num].Details_D = End_X;
    graphic_config_[Layer_Num][Graphic_Num].Details_E = End_Y;

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

/**
 * @brief 设定裁判系统UI矩形
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param End_X 终点x
 * @param End_Y 终点y
 * @return 对应图层指针
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiRectangle(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (ui_change_flag_[Layer_Num][Graphic_Num] == 0)
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ui_change_flag_[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    graphic_config_[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_RECTANGLE;
    graphic_config_[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Color_Enum = Color;
    graphic_config_[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    graphic_config_[Layer_Num][Graphic_Num].Start_X = Start_X;
    graphic_config_[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    graphic_config_[Layer_Num][Graphic_Num].Details_D = End_X;
    graphic_config_[Layer_Num][Graphic_Num].Details_E = End_Y;

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

/**
 * @brief 设定裁判系统UI圆形
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Center_X 圆心x
 * @param Center_Y 圆心y
 * @param Radius 半径
 * @return 对应图层指针
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiCircle(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Radius)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (ui_change_flag_[Layer_Num][Graphic_Num] == 0)
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ui_change_flag_[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    graphic_config_[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_CIRCLE;
    graphic_config_[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Color_Enum = Color;
    graphic_config_[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    graphic_config_[Layer_Num][Graphic_Num].Start_X = Center_X;
    graphic_config_[Layer_Num][Graphic_Num].Start_Y = Center_Y;
    graphic_config_[Layer_Num][Graphic_Num].Details_C = Radius;

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

/**
 * @brief 设定裁判系统UI椭圆形
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Center_X 圆心x
 * @param Center_Y 圆心y
 * @param Length_X x半轴长度
 * @param Length_Y y半轴长度
 * @return 对应图层指针
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiOval(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Length_X, uint32_t Length_Y)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (ui_change_flag_[Layer_Num][Graphic_Num] == 0)
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ui_change_flag_[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    graphic_config_[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_OVAL;
    graphic_config_[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Color_Enum = Color;
    graphic_config_[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    graphic_config_[Layer_Num][Graphic_Num].Start_X = Center_X;
    graphic_config_[Layer_Num][Graphic_Num].Start_Y = Center_Y;
    graphic_config_[Layer_Num][Graphic_Num].Details_D = Length_X;
    graphic_config_[Layer_Num][Graphic_Num].Details_E = Length_Y;

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

/**
 * @brief 设定裁判系统UI圆弧形
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Center_X 圆心x
 * @param Center_Y 圆心y
 * @param Angle_Start 起始角度
 * @param Angle_End 终止角度
 * @param Length_X x半轴长度
 * @param Length_Y y半轴长度
 * @return 对应图层指针
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiArc(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Angle_Start, uint32_t Angle_End, uint32_t Length_X, uint32_t Length_Y)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (ui_change_flag_[Layer_Num][Graphic_Num] == 0)
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ui_change_flag_[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    graphic_config_[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_ARC;
    graphic_config_[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Color_Enum = Color;
    graphic_config_[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    graphic_config_[Layer_Num][Graphic_Num].Start_X = Center_X;
    graphic_config_[Layer_Num][Graphic_Num].Start_Y = Center_Y;
    graphic_config_[Layer_Num][Graphic_Num].Details_A = Angle_Start;
    graphic_config_[Layer_Num][Graphic_Num].Details_B = Angle_End;
    graphic_config_[Layer_Num][Graphic_Num].Details_D = Length_X;
    graphic_config_[Layer_Num][Graphic_Num].Details_E = Length_Y;

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

/**
 * @brief 设定裁判系统UI浮点数
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param Font_Width 字体大小
 * @param Float 数值
 * @return 对应图层指针
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiFloat(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, float Float)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (ui_change_flag_[Layer_Num][Graphic_Num] == 0)
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ui_change_flag_[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    graphic_config_[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_FLOAT;
    graphic_config_[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Color_Enum = Color;
    graphic_config_[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    graphic_config_[Layer_Num][Graphic_Num].Start_X = Start_X;
    graphic_config_[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    graphic_config_[Layer_Num][Graphic_Num].Details_A = Font_Width;
    int32_t *tmp_pointer = (int32_t *) ((uint32_t) &graphic_config_[Layer_Num][Graphic_Num] + 11);
    *tmp_pointer = (int32_t) (Float * 1000.0f);

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

/**
 * @brief 设定裁判系统UI整型数
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param Font_Width 字体大小
 * @param Integer 数值
 * @return 对应图层指针
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiInteger(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, int32_t Integer)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (ui_change_flag_[Layer_Num][Graphic_Num] == 0)
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ui_change_flag_[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    graphic_config_[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_INTEGER;
    graphic_config_[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Color_Enum = Color;
    graphic_config_[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    graphic_config_[Layer_Num][Graphic_Num].Start_X = Start_X;
    graphic_config_[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    graphic_config_[Layer_Num][Graphic_Num].Details_A = Font_Width;
    int32_t *tmp_pointer = (int32_t *) ((uint32_t) &graphic_config_[Layer_Num][Graphic_Num] + 11);
    *tmp_pointer = (int32_t) (Integer);

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

/**
 * @brief 设定裁判系统UI字符串
 *
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param Font_Width 字体大小
 * @param String_Length 字符串长度
 * @return
 */
inline RefereeDataInteractionGraphicConfig *Referee::SetRefereeUiString(uint8_t Layer_Num, uint8_t Graphic_Num, RefereeDataInteractionGraphicColor Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, uint32_t String_Length)
{
    graphic_config_[Layer_Num][Graphic_Num].Index[0] = '0';
    graphic_config_[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (ui_change_flag_[Layer_Num][Graphic_Num] == 0)
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ui_change_flag_[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        graphic_config_[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    graphic_config_[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_STRING;
    graphic_config_[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    graphic_config_[Layer_Num][Graphic_Num].Color_Enum = Color;
    graphic_config_[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    graphic_config_[Layer_Num][Graphic_Num].Start_X = Start_X;
    graphic_config_[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    graphic_config_[Layer_Num][Graphic_Num].Details_A = Font_Width;
    graphic_config_[Layer_Num][Graphic_Num].Details_B = String_Length;

    return (&graphic_config_[Layer_Num][Graphic_Num]);
}

#endif

/************************ COPYRIGHT(C) HNUST-DUST **************************/
