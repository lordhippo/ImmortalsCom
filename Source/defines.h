#pragma once

#define PAYLOAD_SIZE 32

#define PROTO_VERSION_FIXED    0x1
#define PROTO_VERSION_VAR      0x2
#define TYPE_COMMAND           0x1
#define TYPE_CONFIG            0x2
#define TYPE_MATRIX            0x3
#define TYPE_FEEDBACK          0x4
#define TYPE_FEEDBACK_CUSTOM   0x5

#define MESSAGE_HEADER(v,t) ((v << 4) | t)
#define MESSAGE_VERSION(a) (a >> 4)
#define MESSAGE_TYPE(a) (a & 0x0F)
