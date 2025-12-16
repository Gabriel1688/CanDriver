#ifndef OPENARM_CAN_COMMON_H
#define OPENARM_CAN_COMMON_H

#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8
typedef unsigned char __u8;
typedef unsigned int canid_t;
#define CAN_SFF_MASK 0x000007FFU

struct can_frame {
    canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    __u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    __u8    __pad;   /* padding */
    __u8    __res0;  /* reserved / padding */
    __u8    __res1;  /* reserved / padding */
    __u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};
#pragma pack(push, 1)
struct can_frame_ex {
    uint8_t  FrameHeader = 0x08;          // 0：标准帧 0： 数据帧, DLC = 8;
    uint32_t FrameId = 0x01;              // CAN ID 使用电机ID作为CAN ID
    uint8_t data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};
#pragma pack(pop)

#endif  // OPENARM_CAN_COMMON_H
