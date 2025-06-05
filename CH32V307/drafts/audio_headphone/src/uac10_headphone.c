#include "debug.h"
#include "uac10_headphone.h"
#include "ch32v30x_usbfs_device.h"

uac_headphone_unit_t uac_headphone_unit =
{
    .feature_unit.mute = 0,
    .feature_unit.volume_l = -96 * 256,
    .feature_unit.volume_r = -96 * 256,
};

static uac_headphone_unit_t uac_headphone_unit_prev =
{
    .feature_unit.mute = 0,
    .feature_unit.volume_l = -96 * 256,
    .feature_unit.volume_r = -96 * 256,
};

static unsigned int play;
unsigned int sound_out_buffer[SOUND_OUT_BUFFER_SIZE];
unsigned int *sound_out_buffer_p;

//#define COMMANDS_BUFFER_SIZE 6000

//static unsigned short commands_buffer[COMMANDS_BUFFER_SIZE];
//static unsigned int commands_buffer_index = 0;
//static unsigned int start = 0;

/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the DMAy Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}

void I2SInit(void)
{
    play = 0;
    sound_out_buffer_p = sound_out_buffer;
    memset(sound_out_buffer, 0, sizeof(sound_out_buffer));
}

/*static void add_command(unsigned short command)
{
    if (commands_buffer_index >= COMMANDS_BUFFER_SIZE)
        return;
    commands_buffer[commands_buffer_index++] = command;
}*/

void I2S_Tx_Handler(const unsigned char *buffer, unsigned int length)
{
    //start = 1;
    //add_command((unsigned short)length);

    play++;
    if (play == 1)
        sound_out_buffer_p = sound_out_buffer;
    length >>= 2;
    unsigned int *b = (unsigned int*)buffer;
    while (length--)
    {
        *sound_out_buffer_p++ = *b++;
        if (sound_out_buffer_p == sound_out_buffer + SOUND_OUT_BUFFER_SIZE)
            sound_out_buffer_p = sound_out_buffer;
    }
    if (play == 2)
        DMA_Cmd(DMA1_Channel5, ENABLE);
}

void AUDIO_SetInterfaceHandler(void)
{
    //add_command(0x8000 | (unsigned short)USBFS_Interface);
    // start i2s output
    /*if (play)
    {
        DMA_Cmd(DMA1_Channel5, DISABLE);
        play = 0;
        memset(sound_out_buffer, 0, sizeof(sound_out_buffer));
    }*/
}

void AUDIO_SofHandler(void)
{
    //if (start)
    //    add_command(0x4000);
    //todo
}

void AUDIO_ChangesHandler(void)
{
    //todo
}
