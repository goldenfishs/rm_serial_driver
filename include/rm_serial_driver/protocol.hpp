#pragma once

#define ID_MCU (0xC4)
#define ID_REF (0xC5)

#include <cstdint>

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        float q0;
        float q1;
        float q2;
        float q3;
    } quat; /* 四元数 */
    struct __attribute__((packed))
    {
        float yaw;
        float pit;
        float rol;
    } gimbal; /* 欧拉角 */
    uint8_t notice; /* 控制命令 */
} DataMCU_t;

typedef struct __attribute__((packed))
{
    uint8_t id;
    DataMCU_t data;
    uint16_t crc16;
} PackageMCU_t;

typedef struct __attribute__((packed))
{
    uint16_t team;         /* 本身队伍 */
    uint16_t time;         /* 比赛开始时间 */
    uint8_t sentry_hp;     /* 哨兵血量 */
    uint8_t ballet_remain; /* 剩余弹量 */
} DataReferee_t;

typedef struct __attribute__((packed))
{
    uint8_t id;
    DataReferee_t data;
    uint16_t crc16;
} PackageReferee_t;

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        float yaw;
        float pit;
        float rol;
    } delta_eulr;
    struct __attribute__((packed))
    {
        float vx;
        float vy;
        float wz;
    } delta_pos;
    uint8_t notice; /* 控制命令 */
} DataAI_t;

typedef struct __attribute__((packed))
{
    DataAI_t data;
    uint16_t crc16;
} PackageAI_t;