#ifndef __UAC_10_HANDPHONE_H_
#define __UAC_10_HANDPHONE_H_
#include "stdint.h"

/* UAC1.0 Request Type */
#define USB_UAC_REQ_TYPE_MASK           0x1F
#define USB_UAC_REQ_TYPE_ID_INF         0x01
#define USB_UAC_REQ_TYPE_ENDP           0x02

/* UAC1.0 Endpoint Control Selectors */
#define UAC_CS_SAMPLING_FREQ_CONTROL    0x01
#define UAC_CS_PITCH_CONTROL            0x02

/* UAC1.0 Class-Specific Requests */
#define UAC_SET_CUR                     0x01
#define UAC_GET_CUR                     0x81
#define UAC_SET_MIN                     0x02
#define UAC_GET_MIN                     0x82
#define UAC_SET_MAX                     0x03
#define UAC_GET_MAX                     0X83
#define UAC_SET_RES                     0x04
#define UAC_GET_RES                     0x84
#define UAC_SET_MEM                     0x05
#define UAC_GET_MEM                     0x85
#define UAC_GET_STAT                    0xFF

/* UAC1.0 Headphone Feature Unit Control Requests */
#define UAC_CS_MUTE_CONTROL             0x01
#define UAC_CS_VOLUME_CONTROL           0x02

/* UAC1.0 Feature Unit Control Requests Define */
#define UAC_FEATURE_VOLUME_MAX          0
#define UAC_FEATURE_VOLUME_MIN          0xA000
#define UAC_FEATURE_VOLUME_RES          0x0100

#define SAMPLE_RATE 16000

#define SOUND_OUT_BUFFER_SIZE 1024

extern unsigned int sound_out_buffer[];

typedef struct
{
    struct
    {
        uint8_t mute;
        int16_t volume_l;
        int16_t volume_r;
    } feature_unit;
} uac_headphone_unit_t;

extern uac_headphone_unit_t uac_headphone_unit;

void I2SInit(void);
void I2S_Tx_Handler(const unsigned char *buffer, unsigned int length);
void AUDIO_ChangesHandler(void);
void AUDIO_SetInterfaceHandler(void);
void AUDIO_SofHandler(void);

#endif