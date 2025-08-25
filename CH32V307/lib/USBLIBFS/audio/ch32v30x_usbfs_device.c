/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbfs_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/08/20
* Description        : This file provides all the usbfs firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "delay.h"
#include "ch32v30x_usbfs_device.h"

/*******************************************************************************/
/* Variable Definition */
/* Global */
const    uint8_t  *pUSBFS_Descr;

/* Setup Request */
volatile uint8_t  USBFS_SetupReqCode;
volatile uint8_t  USBFS_SetupReqType;
volatile uint16_t USBFS_SetupReqValue;
volatile uint16_t USBFS_SetupReqIndex;
volatile uint16_t USBFS_SetupReqLen;

/* USB Device Status */
volatile uint8_t  USBFS_DevConfig;
volatile uint8_t  USBFS_DevAddr;
volatile uint8_t  USBFS_DevSleepStatus;
volatile uint8_t  USBFS_DevEnumStatus;
volatile uint8_t  USBFS_Interface;
/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBFS_EP0_Buf[ DEF_USBD_UEP0_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBFS_EP3_Buf[ DEF_USBD_EP3_FS_SIZE];
/* USB IN Endpoint Busy Flag */
volatile uint8_t  USBFS_Endp_Busy[ DEF_UEP_NUM ];

/******************************************************************************/
/* Interrupt Service Routine Declaration*/
#if __GNUC__ > 13
void USBFS_IRQHandler(void) __attribute__((naked));
#else
void USBFS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#endif

/*********************************************************************
 * @fn      USBFS_RCC_Init
 *
 * @brief   Initializes the usbfs clock configuration.
 *
 * @return  none
 */
void USBFS_RCC_Init(void)
{
#ifdef CH32V30x_D8C
    RCC_USBCLK48MConfig( RCC_USBCLK48MCLKSource_USBPHY );
    RCC_USBHSPLLCLKConfig( RCC_HSBHSPLLCLKSource_HSE );
    RCC_USBHSConfig( RCC_USBPLL_Div2 );
    RCC_USBHSPLLCKREFCLKConfig( RCC_USBHSPLLCKREFCLK_4M );
    RCC_USBHSPHYPLLALIVEcmd( ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHS, ENABLE );
#else
    if( SystemCoreClock == 144000000 )
    {
        RCC_USBFSCLKConfig( RCC_USBFSCLKSource_PLLCLK_Div3 );
    }
    else if( SystemCoreClock == 96000000 ) 
    {
        RCC_USBFSCLKConfig( RCC_USBFSCLKSource_PLLCLK_Div2 );
    }
    else if( SystemCoreClock == 48000000 ) 
    {
        RCC_USBFSCLKConfig( RCC_USBFSCLKSource_PLLCLK_Div1 );
    }
#endif
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBFS, ENABLE );
}

/*********************************************************************
 * @fn      USBFS_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBFS_Device_Endp_Init( void )
{
    USBFSD->UEP2_3_MOD = USBFS_UEP3_RX_EN;

    USBFSD->UEP0_DMA = (uint32_t)USBFS_EP0_Buf;
    USBFSD->UEP3_DMA = (uint32_t)USBFS_EP3_Buf;

    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
    USBFSD->UEP3_RX_CTRL = USBFS_UEP_R_RES_ACK;

    /* Clear End-points Busy Status */
    for(uint8_t i=0; i<DEF_UEP_NUM; i++ )
    {
        USBFS_Endp_Busy[ i ] = 0;
    }
}

/*********************************************************************
 * @fn      USBFS_Device_Init
 *
 * @brief   Initializes USB device.
 *
 * @return  none
 */
void USBFS_Device_Init( FunctionalState sta )
{
    if( sta )
    {
        USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
		USBFSH->BASE_CTRL = 0x00;
        USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;// | USBFS_UIE_DEV_SOF;
        USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
        USBFS_Device_Endp_Init( );
        USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
        NVIC_EnableIRQ(USBFS_IRQn);
    }
    else
    {
        USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
        USBFSH->BASE_CTRL = 0x00;
        NVIC_DisableIRQ(USBFS_IRQn);
    }
}

/*********************************************************************
 * @fn      USBFS_Endp_DataUp
 *
 * @brief   USBFS device data upload
 *
 * @return  none
 */
uint8_t USBFS_Endp_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod)
{
    uint8_t endp_mode;
    uint8_t buf_load_offset;

    /* DMA config, endp_ctrl config, endp_len config */
    if( (endp>=DEF_UEP1) && (endp<=DEF_UEP7) )
    {
        if( USBFS_Endp_Busy[ endp ] == 0 )
        {
            if( (endp == DEF_UEP1) || (endp == DEF_UEP4) )
            {
                /* endp1/endp4 */
                endp_mode = USBFSD_UEP_MOD(0);
                if( endp == DEF_UEP1 )
                {
                    endp_mode = (uint8_t)(endp_mode>>4);
                }
            }
            else if( (endp == DEF_UEP2) || (endp == DEF_UEP3) )
            {
                /* endp2/endp3 */
                endp_mode = USBFSD_UEP_MOD(1);
                if( endp == DEF_UEP3 )
                {
                    endp_mode = (uint8_t)(endp_mode>>4);
                }
            }
            else if( (endp == DEF_UEP5) || (endp == DEF_UEP6) )
            {
                /* endp5/endp6 */
                endp_mode = USBFSD_UEP_MOD(2);
                if( endp == DEF_UEP6 )
                {
                    endp_mode = (uint8_t)(endp_mode>>4);
                }
            }
            else
            {
                /* endp7 */
                endp_mode = USBFSD_UEP_MOD(3);
            }

            if( endp_mode & USBFSD_UEP_TX_EN )
            {
                if( endp_mode & USBFSD_UEP_RX_EN )
                {
                    buf_load_offset = 64;
                }
                else
                {
                    buf_load_offset = 0;
                }

                if( buf_load_offset == 0 )
                {
                    if( mod == DEF_UEP_DMA_LOAD )
                    {
                        /* DMA mode */
                        USBFSD_UEP_DMA(endp) = (uint16_t)(uint32_t)pbuf;
                    }
                    else
                    {
                        /* copy mode */
                        memcpy( USBFSD_UEP_BUF(endp), pbuf, len );
                    }
                }
                else
                {
                    memcpy( USBFSD_UEP_BUF(endp)+buf_load_offset, pbuf, len );
                }
                /* Set end-point busy */
                USBFS_Endp_Busy[ endp ] = 0x01;                
                /* tx length */
                USBFSD_UEP_TLEN(endp) = len;
                /* response ack */
                USBFSD_UEP_TX_CTRL(endp) = (USBFSD_UEP_TX_CTRL(endp) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
            }
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
    return 0;
}

static void USB_FS_TokenInHandler(uint8_t intst)
{
    switch ( intst & ( USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK ) )
    {
        /* end-point 0 data in interrupt */
        case USBFS_UIS_TOKEN_IN | DEF_UEP0:
            if( USBFS_SetupReqLen == 0 )
            {
                USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
            }
            if ( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
            {
                /* Non-standard request endpoint 0 Data upload */
            }
            else
            {
                switch( USBFS_SetupReqCode )
                {
                    case USB_GET_DESCRIPTOR:
                        uint16_t len = USBFS_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
                        memcpy( USBFS_EP0_Buf, pUSBFS_Descr, len );
                        USBFS_SetupReqLen -= len;
                        pUSBFS_Descr += len;
                        USBFSD->UEP0_TX_LEN   = len;
                        USBFSD->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;
                        break;

                    case USB_SET_ADDRESS:
                        USBFSD->DEV_ADDR = (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | USBFS_DevAddr;
                        break;

                    default:
                        break;
                }
            }
            break;

        default :
            break;
    }
}

static uint8_t USB_FS_TokenOutHandler(uint8_t intst)
{
    uint8_t errflag = 0;

    switch ( intst & ( USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK ) )
    {
        /* end-point 0 data out interrupt */
        case USBFS_UIS_TOKEN_OUT | DEF_UEP0:
            volatile unsigned short len = USBFSD->RX_LEN;
            (void)len;
            if ( intst & USBFS_UIS_TOG_OK )
            {
                USBFS_SetupReqLen = 0;
                if( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    if( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS )
                    {
                        if ((USBFS_SetupReqType & USB_UAC_REQ_TYPE_MASK) == USB_UAC_REQ_TYPE_ID_INF)
                        {
                            switch( USBFS_SetupReqCode )
                            {
                                case UAC_SET_CUR:
                                    /* Entity ID: 2 */
                                    if (USBFS_SetupReqIndex == 0x0200)
                                    {
                                        switch ((USBFS_SetupReqValue >> 8) & 0xFF)
                                        {
                                            case UAC_CS_MUTE_CONTROL:
                                                uac_headphone_unit.feature_unit.mute = USBFS_EP0_Buf[ 0 ];
                                                break;
                                            case UAC_CS_VOLUME_CONTROL:
                                                if ((USBFS_SetupReqValue & 0xFF) == 0x01 )
                                                {
                                                    uac_headphone_unit.feature_unit.volume_l = USBFS_EP0_Buf[0] | (USBFS_EP0_Buf[1] << 8);
                                                }
                                                else if ((USBFS_SetupReqValue & 0xFF) == 0x02 )
                                                {
                                                    uac_headphone_unit.feature_unit.volume_r = USBFS_EP0_Buf[0] | (USBFS_EP0_Buf[1] << 8);
                                                }
                                                else
                                                {
                                                    errflag = 0xFF;
                                                }
                                                break;
                                            default:
                                                errflag = 0xFF;
                                                break;
                                        }
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;
                                case UAC_SET_MIN:
                                case UAC_SET_MAX:
                                case UAC_SET_RES:
                                case UAC_SET_MEM:
                                    errflag = 0xFF;
                                    break;
                                default:
                                    break;
                            }
                        }
                        else if((USBFS_SetupReqType & USB_UAC_REQ_TYPE_MASK) == USB_UAC_REQ_TYPE_ENDP)
                        {
                            switch( USBFS_SetupReqCode )
                            {
                                case UAC_CS_SAMPLING_FREQ_CONTROL:
                                    /* add your code here */
                                    break;
                                case UAC_CS_PITCH_CONTROL:
                                    /* add your code here */
                                    break;
                                default:
                                    errflag = 0xFF;
                                    break;
                            }
                        }
                        else
                        {
                            errflag = 0xFF;
                        }
                    }
                }
                else
                {
                    /* Standard request end-point 0 Data download */
                    /* Add your code here */
                }
            }

            if( USBFS_SetupReqLen == 0 )
            {
                USBFSD->UEP0_TX_LEN  = 0;
                USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
            }
            break;
        /* end-point 3 data out interrupt */
        case USBFS_UIS_TOKEN_OUT | DEF_UEP3:
            if ( intst & USBFS_UIS_TOG_OK )
                I2S_Tx_Handler(USBFS_EP3_Buf, USBFSD->RX_LEN > DEF_USBD_FS_ISO_PACK_SIZE ? DEF_USBD_FS_ISO_PACK_SIZE : USBFSD->RX_LEN);
            break;
    }

    return errflag;
}

static uint8_t USB_FS_GetCurHandler(void)
{
    uint8_t errflag = 0;
    /* Entity ID: 2 */
    if (USBFS_SetupReqIndex == 0x0200)
    {
        switch ((USBFS_SetupReqValue >> 8) & 0xFF)
        {
            case UAC_CS_MUTE_CONTROL:
                USBFS_EP0_Buf[ 0 ] = uac_headphone_unit.feature_unit.mute;
                break;
            case UAC_CS_VOLUME_CONTROL:
                if ((USBFS_SetupReqValue & 0xFF) == 0x01 )
                {
                    *(uint16_t *)USBFS_EP0_Buf = uac_headphone_unit.feature_unit.volume_l;
                }
                else if ((USBFS_SetupReqValue & 0xFF) == 0x02 )
                {
                    *(uint16_t *)USBFS_EP0_Buf = uac_headphone_unit.feature_unit.volume_r;
                }
                else
                {
                    errflag = 0xFF;
                }
                break;
            default:
                errflag = 0xFF;
                break;
        }
    }
    else
    {
        errflag = 0xFF;
    }

    return errflag;
}

static uint8_t USB_FS_GetMinHandler(void)
{
    uint8_t errflag = 0;

    /* Entity ID: 2 */
    if (USBFS_SetupReqIndex == 0x0200)
    {
        switch ((USBFS_SetupReqValue >> 8) & 0xFF)
        {
            case UAC_CS_VOLUME_CONTROL:
                if ((USBFS_SetupReqValue & 0xFF) == 0x01 )
                {
                    *(uint16_t *)USBFS_EP0_Buf = UAC_FEATURE_VOLUME_MIN;
                }
                else if ((USBFS_SetupReqValue & 0xFF) == 0x02 )
                {
                    *(uint16_t *)USBFS_EP0_Buf = UAC_FEATURE_VOLUME_MIN;
                }
                else
                {
                    errflag = 0xFF;
                }
                break;
            default:
                errflag = 0xFF;
                break;
        }
    }
    else
    {
        errflag = 0xFF;
    }

    return errflag;
}

static uint8_t USB_FS_GetMaxHandler(void)
{
    uint8_t errflag = 0;

    /* Entity ID: 2 */
    if (USBFS_SetupReqIndex == 0x0200)
    {
        switch ((USBFS_SetupReqValue >> 8) & 0xFF)
        {
            case UAC_CS_VOLUME_CONTROL:
                if ((USBFS_SetupReqValue & 0xFF) == 0x01 )
                {
                    *(uint16_t *)USBFS_EP0_Buf = UAC_FEATURE_VOLUME_MAX;
                }
                else if ((USBFS_SetupReqValue & 0xFF) == 0x02 )
                {
                    *(uint16_t *)USBFS_EP0_Buf = UAC_FEATURE_VOLUME_MAX;
                }
                else
                {
                    errflag = 0xFF;
                }
                break;
            default:
                errflag = 0xFF;
                break;
        }
    }
    else
    {
        errflag = 0xFF;
    }

    return errflag;
}

static uint8_t USB_FS_GetResHandler(void)
{
    uint8_t errflag = 0;

    /* Entity ID: 2 */
    if (USBFS_SetupReqIndex == 0x0200)
    {
        switch ((USBFS_SetupReqValue >> 8) & 0xFF)
        {
            case UAC_CS_VOLUME_CONTROL:
                if ((USBFS_SetupReqValue & 0xFF) == 0x01 )
                {
                    *(uint16_t *)USBFS_EP0_Buf = UAC_FEATURE_VOLUME_RES;
                }
                else if ((USBFS_SetupReqValue & 0xFF) == 0x02 )
                {
                    *(uint16_t *)USBFS_EP0_Buf = UAC_FEATURE_VOLUME_RES;
                }
                else
                {
                    errflag = 0xFF;
                }
                break;
            default:
                errflag = 0xFF;
                break;
        }
    }
    else
    {
        errflag = 0xFF;
    }

    return errflag;
}

static uint8_t USB_FS_GetDescriptorHandler(void)
{
    uint16_t len = 0;
    uint8_t errflag = 0;

    switch( (uint8_t)( USBFS_SetupReqValue >> 8 ) )
    {
        /* get usb device descriptor */
        case USB_DESCR_TYP_DEVICE:
            pUSBFS_Descr = MyDevDescr;
            len = DEF_USBD_DEVICE_DESC_LEN;
            break;

        /* get usb configuration descriptor */
        case USB_DESCR_TYP_CONFIG:
            pUSBFS_Descr = MyCfgDescr;
            len = DEF_USBD_CONFIG_DESC_LEN;
            break;

        /* get usb string descriptor */
        case USB_DESCR_TYP_STRING:
            switch( (uint8_t)( USBFS_SetupReqValue & 0xFF ) )
            {
                /* Descriptor 0, Language descriptor */
                case DEF_STRING_DESC_LANG:
                    pUSBFS_Descr = MyLangDescr;
                    len = DEF_USBD_LANG_DESC_LEN;
                    break;

                /* Descriptor 1, Manufacturers String descriptor */
                case DEF_STRING_DESC_MANU:
                    pUSBFS_Descr = MyManuInfo;
                    len = DEF_USBD_MANU_DESC_LEN;
                    break;

                /* Descriptor 2, Product String descriptor */
                case DEF_STRING_DESC_PROD:
                    pUSBFS_Descr = MyProdInfo;
                    len = DEF_USBD_PROD_DESC_LEN;
                    break;

                /* Descriptor 3, Serial-number String descriptor */
                case DEF_STRING_DESC_SERN:
                    pUSBFS_Descr = MySerNumInfo;
                    len = DEF_USBD_SN_DESC_LEN;
                    break;

                default:
                    errflag = 0xFF;
                    break;
            }
            break;

        default :
            errflag = 0xFF;
            break;
    }

    /* Copy Descriptors to Endp0 DMA buffer */
    if( USBFS_SetupReqLen>len )
    {
        USBFS_SetupReqLen = len;
    }
    len = (USBFS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
    memcpy( USBFS_EP0_Buf, pUSBFS_Descr, len );
    pUSBFS_Descr += len;

    return errflag;
}

static uint8_t USB_FS_ClearFeatureHandler(void)
{
    uint8_t errflag = 0;

    if ( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
    {
        /* clear one device feature */
        if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
        {
            /* clear usb sleep status, device not prepare to sleep */
            USBFS_DevSleepStatus &= ~0x01;
        }
    }
    else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
    {
        /* Clear End-point Feature */
        if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
        {
            switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
            {
                case ( DEF_UEP_OUT | DEF_UEP1 ):
                    /* Set End-point 1 OUT ACK*/
                    USBFSD->UEP1_RX_CTRL = USBFS_UEP_R_RES_ACK;
                    break;

                case ( DEF_UEP_IN | DEF_UEP2 ):
                    /* Set End-point 2 IN NAK */
                    USBFSD->UEP2_TX_CTRL = USBFS_UEP_T_RES_NAK;
                    break;

                case ( DEF_UEP_OUT | DEF_UEP3 ):
                    /* Set End-point 3 OUT ACK */
                    USBFSD->UEP3_RX_CTRL = USBFS_UEP_R_RES_ACK;
                    break;

                case ( DEF_UEP_IN | DEF_UEP4 ):
                    /* Set End-point 4 IN NAK */
                    USBFSD->UEP4_TX_CTRL = USBFS_UEP_T_RES_NAK;
                    break;

                case ( DEF_UEP_OUT | DEF_UEP5 ):
                    /* Set End-point 5 OUT ACK */
                    USBFSD->UEP5_RX_CTRL = USBFS_UEP_R_RES_ACK;
                    break;

                case ( DEF_UEP_IN | DEF_UEP6 ):
                    /* Set End-point 6 IN NAK */
                    USBFSD->UEP6_TX_CTRL = USBFS_UEP_T_RES_NAK;
                    break;

                default:
                    errflag = 0xFF;
                    break;
            }
        }
        else
        {
            errflag = 0xFF;
        }
    }
    else
    {
        errflag = 0xFF;
    }

    return errflag;
}

static uint8_t USB_FS_SetFeatureHandler(void)
{
    uint8_t errflag = 0;

    if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
    {
        /* Set Device Feature */
        if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
        {
            if( MyCfgDescr[ 7 ] & 0x20 )
            {
                /* Set Wake-up flag, device prepare to sleep */
                USBFS_DevSleepStatus |= 0x01;
            }
            else
            {
                errflag = 0xFF;
            }
        }
        else
        {
            errflag = 0xFF;
        }
    }
    else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
    {
        /* Set End-point Feature */
        if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
        {
            /* Set end-points status stall */
            switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
            {
                case ( DEF_UEP_OUT | DEF_UEP1 ):
                    /* Set End-point 1 OUT STALL */
                    USBFSD->UEP1_RX_CTRL = ( USBFSD->UEP1_RX_CTRL & ~USBFS_UEP_R_RES_MASK ) | USBFS_UEP_R_RES_STALL;
                    break;

                case ( DEF_UEP_IN | DEF_UEP2 ):
                    /* Set End-point 2 IN STALL */
                    USBFSD->UEP2_TX_CTRL = ( USBFSD->UEP2_TX_CTRL & ~USBFS_UEP_T_RES_MASK ) | USBFS_UEP_T_RES_STALL;
                    break;

                case ( DEF_UEP_OUT | DEF_UEP3 ):
                    /* Set End-point 3 OUT STALL */
                    USBFSD->UEP3_RX_CTRL = ( USBFSD->UEP3_RX_CTRL & ~USBFS_UEP_R_RES_MASK ) | USBFS_UEP_R_RES_STALL;
                    break;

                case ( DEF_UEP_IN | DEF_UEP4 ):
                    /* Set End-point 4 IN STALL */
                    USBFSD->UEP4_TX_CTRL = ( USBFSD->UEP4_TX_CTRL & ~USBFS_UEP_T_RES_MASK ) | USBFS_UEP_T_RES_STALL;
                    break;

                case ( DEF_UEP_OUT | DEF_UEP5 ):
                    /* Set End-point 5 OUT STALL */
                    USBFSD->UEP5_RX_CTRL = ( USBFSD->UEP5_RX_CTRL & ~USBFS_UEP_R_RES_MASK ) | USBFS_UEP_R_RES_STALL;
                    break;

                case ( DEF_UEP_IN | DEF_UEP6 ):
                    /* Set End-point 6 IN STALL */
                    USBFSD->UEP6_TX_CTRL = ( USBFSD->UEP6_TX_CTRL & ~USBFS_UEP_T_RES_MASK ) | USBFS_UEP_T_RES_STALL;
                    break;

                default:
                    errflag = 0xFF;
                    break;
            }
        }
        else
        {
            errflag = 0xFF;
        }
    }
    else
    {
        errflag = 0xFF;
    }

    return errflag;
}

static uint8_t USB_FS_GetStatusHandler(void)
{
    uint8_t errflag = 0;

    USBFS_EP0_Buf[ 0 ] = 0x00;
    USBFS_EP0_Buf[ 1 ] = 0x00;
    if ( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
    {
        if( USBFS_DevSleepStatus & 0x01 )
        {
            USBFS_EP0_Buf[ 0 ] = 0x02;
        }
    }
    else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
    {
        switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
        {
            case ( DEF_UEP_OUT | DEF_UEP1 ):
                if( ( (USBFSD->UEP1_RX_CTRL) & USBFS_UEP_R_RES_MASK ) == USBFS_UEP_R_RES_STALL )
                {
                    USBFS_EP0_Buf[ 0 ] = 0x01;
                }
                break;

            case ( DEF_UEP_IN | DEF_UEP2 ):
                if( ( (USBFSD->UEP2_TX_CTRL) & USBFS_UEP_T_RES_MASK ) == USBFS_UEP_T_RES_STALL )
                {
                    USBFS_EP0_Buf[ 0 ] = 0x01;
                }
                break;

            case ( DEF_UEP_OUT | DEF_UEP3 ):
                if( ( (USBFSD->UEP3_RX_CTRL) & USBFS_UEP_R_RES_MASK ) == USBFS_UEP_R_RES_STALL )
                {
                    USBFS_EP0_Buf[ 0 ] = 0x01;
                }
                break;

            case ( DEF_UEP_IN | DEF_UEP4 ):
                if( ( (USBFSD->UEP4_TX_CTRL) & USBFS_UEP_T_RES_MASK ) == USBFS_UEP_T_RES_STALL )
                {
                    USBFS_EP0_Buf[ 0 ] = 0x01;
                }
                break;

            case ( DEF_UEP_OUT | DEF_UEP5 ):
                if( ( (USBFSD->UEP5_RX_CTRL) & USBFS_UEP_R_RES_MASK ) == USBFS_UEP_R_RES_STALL )
                {
                    USBFS_EP0_Buf[ 0 ] = 0x01;
                }
                break;

            case ( DEF_UEP_IN | DEF_UEP6 ):
                if( ( (USBFSD->UEP6_TX_CTRL) & USBFS_UEP_T_RES_MASK ) == USBFS_UEP_T_RES_STALL )
                {
                    USBFS_EP0_Buf[ 0 ] = 0x01;
                }
                break;

            default:
                errflag = 0xFF;
                break;
        }
    }
    else
    {
        errflag = 0xFF;
    }

    if ( USBFS_SetupReqLen > 2 )
    {
        USBFS_SetupReqLen = 2;
    }

    return errflag;
}

static uint8_t USB_FS_TokenSetupHandler(void)
{
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG|USBFS_UEP_R_RES_NAK;
    /* Store All Setup Values */
    USBFS_SetupReqType  = pUSBFS_SetupReqPak->bRequestType;
    USBFS_SetupReqCode  = pUSBFS_SetupReqPak->bRequest;
    USBFS_SetupReqLen   = pUSBFS_SetupReqPak->wLength;
    USBFS_SetupReqValue = pUSBFS_SetupReqPak->wValue;
    USBFS_SetupReqIndex = pUSBFS_SetupReqPak->wIndex;
    uint8_t errflag = 0;
    if ( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
    {
        if( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS )
        {
            if ((USBFS_SetupReqType & USB_UAC_REQ_TYPE_MASK) == USB_UAC_REQ_TYPE_ID_INF)
            {
                switch( USBFS_SetupReqCode )
                {
                    case UAC_GET_CUR:
                        errflag = USB_FS_GetCurHandler();
                        break;
                    case UAC_GET_MIN:
                        errflag = USB_FS_GetMinHandler();
                        break;
                    case UAC_GET_MAX:
                        errflag = USB_FS_GetMaxHandler();
                        break;
                    case UAC_GET_RES:
                        errflag = USB_FS_GetResHandler();
                        break;
                    case UAC_SET_CUR:
                        break;
                    default:
                        errflag = 0xFF;
                        break;
                }
            }
            else if ((USBFS_SetupReqType & USB_UAC_REQ_TYPE_MASK) == USB_UAC_REQ_TYPE_ENDP)
            {
                switch( USBFS_SetupReqCode )
                {
                    case UAC_CS_SAMPLING_FREQ_CONTROL:
                    case UAC_CS_PITCH_CONTROL:
                        break;
                    default:
                        errflag = 0xFF;
                        break;
                }
            }
            else
            {
                errflag = 0xFF;
            }
        }
    }
    else
    {
        /* usb standard request processing */
        switch( USBFS_SetupReqCode )
        {
            /* get device/configuration/string/report/... descriptors */
            case USB_GET_DESCRIPTOR:
                errflag = USB_FS_GetDescriptorHandler();
                break;

            /* Set usb address */
            case USB_SET_ADDRESS:
                USBFS_DevAddr = (uint8_t)( USBFS_SetupReqValue & 0xFF );
                break;

            /* Get usb configuration now set */
            case USB_GET_CONFIGURATION:
                USBFS_EP0_Buf[0] = USBFS_DevConfig;
                if ( USBFS_SetupReqLen > 1 )
                {
                    USBFS_SetupReqLen = 1;
                }
                break;

            /* Set usb configuration to use */
            case USB_SET_CONFIGURATION:
                USBFS_DevConfig = (uint8_t)( USBFS_SetupReqValue & 0xFF );
                USBFS_DevEnumStatus = 0x01;
                break;

            /* Clear or disable one usb feature */
            case USB_CLEAR_FEATURE:
                errflag = USB_FS_ClearFeatureHandler();
                break;

            /* set or enable one usb feature */
            case USB_SET_FEATURE:
                errflag = USB_FS_SetFeatureHandler();
                break;

            /* This request allows the host to select another setting for the specified interface  */
            case USB_GET_INTERFACE:
                USBFS_EP0_Buf[0] = USBFS_Interface;
                if ( USBFS_SetupReqLen > 1 )
                {
                    USBFS_SetupReqLen = 1;
                }
                break;

            case USB_SET_INTERFACE:
                USBFS_Interface = USBFS_SetupReqValue & 0xFF;
                AUDIO_SetInterfaceHandler();
                //todo
                break;

            /* host get status of specified device/interface/end-points */
            case USB_GET_STATUS:
                errflag = USB_FS_GetStatusHandler();
                break;

            default:
                errflag = 0xFF;
                break;
        }
    }

    /* errflag = 0xFF means a request not support or some errors occurred, else correct */
    if( errflag == 0xFF)
    {
        /* if one request not support, return stall */
        USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL;
        USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL;
    }
    else
    {
        /* end-point 0 data Tx/Rx */
        if( USBFS_SetupReqType & DEF_UEP_IN )
        {
            /* tx */
            uint16_t len = ( USBFS_SetupReqLen > DEF_USBD_UEP0_SIZE ) ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
            USBFS_SetupReqLen -= len;
            USBFSD->UEP0_TX_LEN  = len;
            USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
        }
        else
        {
            /* rx */
            if( USBFS_SetupReqLen == 0 )
            {
                USBFSD->UEP0_TX_LEN  = 0;
                USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
            }
            else
            {
                USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
            }
        }
    }

    return errflag;
}

/*********************************************************************
 * @fn      USBFS_IRQHandler
 *
 * @brief   This function handles USBFS exception.
 *
 * @return  none
 */
void USBFS_IRQHandler( void )
{
    uint8_t  intflag, intst;

    intflag = USBFSD->INT_FG;
    intst = USBFSD->INT_ST;

    if( intflag & USBFS_UIF_TRANSFER )
    {
        switch ( intst & USBFS_UIS_TOKEN_MASK )
        {
            /* data-in stage processing */
            case USBFS_UIS_TOKEN_IN:
                USB_FS_TokenInHandler(intst);
                break;

            /* data-out stage processing */
            case USBFS_UIS_TOKEN_OUT:
                USB_FS_TokenOutHandler(intst);
                break;

            /* Setup stage processing */
            case USBFS_UIS_TOKEN_SETUP:
                USB_FS_TokenSetupHandler();
                break;

            /* Sof pack processing */
            case USBFS_UIS_TOKEN_SOF:
                AUDIO_SofHandler();
                break;

            default :
                break;
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
    }
    else if( intflag & USBFS_UIF_BUS_RST )
    {
        /* usb reset interrupt processing */
        USBFS_DevConfig = 0;
        USBFS_DevAddr = 0;
        USBFS_DevSleepStatus = 0;
        USBFS_DevEnumStatus = 0;

        USBFSD->DEV_ADDR = 0;
        USBFS_Device_Endp_Init( );
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    }
    else if( intflag & USBFS_UIF_SUSPEND )
    {
        /* usb suspend interrupt processing */
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
        Delay_Us(10);
        if ( USBFSD->MIS_ST & USBFS_UMS_SUSPEND )
        {
            USBFS_DevSleepStatus |= 0x02;
            if( USBFS_DevSleepStatus == 0x03 )
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USBFS_DevSleepStatus &= ~0x02;
        }
    }
    else
    {
        /* other interrupts */
        USBFSD->INT_FG = intflag;
    }
#if __GNUC__ > 13
    asm volatile ("mret");
#endif
}

/*********************************************************************
 * @fn      USBFS_Send_Resume
 *
 * @brief   Send usb k signal, Wake up usb host
 *
 * @return  none
 */
void USBFS_Send_Resume( void )
{
    USBFSD->UDEV_CTRL ^= USBFS_UD_LOW_SPEED;
    Delay_Ms( 8 );
    USBFSD->UDEV_CTRL ^= USBFS_UD_LOW_SPEED;
    Delay_Ms( 1 );
}

