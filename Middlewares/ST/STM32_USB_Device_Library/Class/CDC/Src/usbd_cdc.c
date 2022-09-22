/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the high layer firmware functions to manage the
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *
  *  @verbatim
  *
  *          ===================================================================
  *                                CDC Class Driver Description
  *          ===================================================================
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  *
  *           These aspects may be enriched or modified for a specific user application.
  *
  *            This driver doesn't implement the following aspects of the specification
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);
static uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length);
uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Variables
  * @{
  */


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_CDC =
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  NULL,
  NULL,
  NULL,
  NULL,
  USBD_CDC_GetFSCfgDesc,
  NULL,
  USBD_CDC_GetDeviceQualifierDescriptor,
};

/* USB CDC device Configuration Descriptor */
//__ALIGN_BEGIN uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
__ALIGN_BEGIN static uint8_t USBD_CDC_CfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_CDC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: 1 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */

  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
} ;

static uint8_t CDCInEpAdd = CDC_IN_EP;
static uint8_t CDCOutEpAdd = CDC_OUT_EP;
static uint8_t CDCCmdEpAdd = CDC_CMD_EP;


/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Functions
  * @{
  */

/**
  * @brief  USBD_CDC_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx)
{


	UNUSED(cfgidx);
	  USBD_CDC_HandleTypeDef *hcdc;

	  hcdc = (USBD_CDC_HandleTypeDef *)USBD_malloc(sizeof(USBD_CDC_HandleTypeDef));

	  if (hcdc == NULL)
	  {
	    pdev->pClassDataCmsit[pdev->classId] = NULL;
	    return (uint8_t)USBD_EMEM;
	  }

	  (void)USBD_memset(hcdc, 0, sizeof(USBD_CDC_HandleTypeDef));

	  pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
	  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];


	  if (pdev->dev_speed == USBD_SPEED_HIGH)
	  {
	    /* Open EP IN */
	    (void)USBD_LL_OpenEP(pdev, CDCInEpAdd, USBD_EP_TYPE_BULK,
	                         CDC_DATA_HS_IN_PACKET_SIZE);

	    pdev->ep_in[CDCInEpAdd & 0xFU].is_used = 1U;

	    /* Open EP OUT */
	    (void)USBD_LL_OpenEP(pdev, CDCOutEpAdd, USBD_EP_TYPE_BULK,
	                         CDC_DATA_HS_OUT_PACKET_SIZE);

	    pdev->ep_out[CDCOutEpAdd & 0xFU].is_used = 1U;

	    /* Set bInterval for CDC CMD Endpoint */
	    pdev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = CDC_HS_BINTERVAL;
	  }
	  else
	  {
	    /* Open EP IN */
	    (void)USBD_LL_OpenEP(pdev, CDCInEpAdd, USBD_EP_TYPE_BULK,
	                         CDC_DATA_FS_IN_PACKET_SIZE);

	    pdev->ep_in[CDCInEpAdd & 0xFU].is_used = 1U;

	    /* Open EP OUT */
	    (void)USBD_LL_OpenEP(pdev, CDCOutEpAdd, USBD_EP_TYPE_BULK,
	                         CDC_DATA_FS_OUT_PACKET_SIZE);

	    pdev->ep_out[CDCOutEpAdd & 0xFU].is_used = 1U;

	    /* Set bInterval for CMD Endpoint */
	    pdev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = CDC_FS_BINTERVAL;
	  }

	  /* Open Command IN EP */
	  (void)USBD_LL_OpenEP(pdev, CDCCmdEpAdd, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
	  pdev->ep_in[CDCCmdEpAdd & 0xFU].is_used = 1U;

	  hcdc->RxBuffer = NULL;

	  /* Init  physical Interface components */
	  ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init();

	  /* Init Xfer states */
	  hcdc->TxState = 0U;
	  hcdc->RxState = 0U;

	  if (hcdc->RxBuffer == NULL)
	  {
	    return (uint8_t)USBD_EMEM;
	  }

	  if (pdev->dev_speed == USBD_SPEED_HIGH)
	  {
	    /* Prepare Out endpoint to receive next packet */
	    (void)USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer,
	                                 CDC_DATA_HS_OUT_PACKET_SIZE);
	  }
	  else
	  {
	    /* Prepare Out endpoint to receive next packet */
	    (void)USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer,
	                                 CDC_DATA_FS_OUT_PACKET_SIZE);
	  }

	  return (uint8_t)USBD_OK;

//  uint8_t ret = 0;
//  USBD_CDC_HandleTypeDef   *hcdc;
//
//  if(pdev->dev_speed == USBD_SPEED_HIGH  )
//  {
//    /* Open EP IN */
//    USBD_LL_OpenEP(pdev,
//                   CDC_IN_EP,
//                   USBD_EP_TYPE_BULK,
//                   CDC_DATA_HS_IN_PACKET_SIZE);
//
//    /* Open EP OUT */
//    USBD_LL_OpenEP(pdev,
//                   CDC_OUT_EP,
//                   USBD_EP_TYPE_BULK,
//                   CDC_DATA_HS_OUT_PACKET_SIZE);
//
//  }
//  else
//  {
//    /* Open EP IN */
//    USBD_LL_OpenEP(pdev,
//                   CDC_IN_EP,
//                   USBD_EP_TYPE_BULK,
//                   CDC_DATA_FS_IN_PACKET_SIZE);
//
//    /* Open EP OUT */
//    USBD_LL_OpenEP(pdev,
//                   CDC_OUT_EP,
//                   USBD_EP_TYPE_BULK,
//                   CDC_DATA_FS_OUT_PACKET_SIZE);
//  }
//
//  pdev->pClassData = USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));
//
//  if(pdev->pClassData == NULL)
//  {
//    ret = 1;
//  }
//  else
//  {
//    hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//    /* Init  physical Interface components */
//   // ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init();
//    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init();
//
//    /* Init Xfer states */
//    hcdc->TxState =0;
//    hcdc->RxState =0;
//
//    if(pdev->dev_speed == USBD_SPEED_HIGH  )
//    {
//      /* Prepare Out endpoint to receive next packet */
//      USBD_LL_PrepareReceive(pdev,
//                             CDC_OUT_EP,
//                             hcdc->RxBuffer,
//                             CDC_DATA_HS_OUT_PACKET_SIZE);
//    }
//    else
//    {
//      /* Prepare Out endpoint to receive next packet */
//      USBD_LL_PrepareReceive(pdev,
//                             CDC_OUT_EP,
//                             hcdc->RxBuffer,
//                             CDC_DATA_FS_OUT_PACKET_SIZE);
//    }
//
//
//  }
//  return ret;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{

	UNUSED(cfgidx);




	  /* Close EP IN */
	  (void)USBD_LL_CloseEP(pdev, CDCInEpAdd);
	  pdev->ep_in[CDCInEpAdd & 0xFU].is_used = 0U;

	  /* Close EP OUT */
	  (void)USBD_LL_CloseEP(pdev, CDCOutEpAdd);
	  pdev->ep_out[CDCOutEpAdd & 0xFU].is_used = 0U;

	  /* Close Command IN EP */
	  (void)USBD_LL_CloseEP(pdev, CDCCmdEpAdd);
	  pdev->ep_in[CDCCmdEpAdd & 0xFU].is_used = 0U;
	  pdev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = 0U;

	  /* DeInit  physical Interface components */
	  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
	  {
	    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
	    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
	    pdev->pClassDataCmsit[pdev->classId] = NULL;
	    pdev->pClassData = NULL;
	  }

	  return (uint8_t)USBD_OK;


//  uint8_t ret = 0;
//
//  /* Open EP IN */
//  USBD_LL_CloseEP(pdev,
//              CDC_IN_EP);
//
//  /* Open EP OUT */
//  USBD_LL_CloseEP(pdev,
//              CDC_OUT_EP);
//
//
//
//  /* DeInit  physical Interface components */
//  if(pdev->pClassData != NULL)
//  {
//    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
//    USBD_free(pdev->pClassData);
//    pdev->pClassData = NULL;
//  }
//
//  return ret;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{


	 USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	  uint16_t len;
	  uint8_t ifalt = 0U;
	  uint16_t status_info = 0U;
	  USBD_StatusTypeDef ret = USBD_OK;

	  if (hcdc == NULL)
	  {
	    return (uint8_t)USBD_FAIL;
	  }

	  switch (req->bmRequest & USB_REQ_TYPE_MASK)
	  {
	    case USB_REQ_TYPE_CLASS:
	      if (req->wLength != 0U)
	      {
	        if ((req->bmRequest & 0x80U) != 0U)
	        {
	          ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
	                                                                           (uint8_t *)hcdc->data,
	                                                                           req->wLength);

	          len = MIN(CDC_REQ_MAX_DATA_SIZE, req->wLength);
	          (void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->data, len);
	        }
	        else
	        {
	          hcdc->CmdOpCode = req->bRequest;
	          hcdc->CmdLength = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);

	          (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->data, hcdc->CmdLength);
	        }
	      }
	      else
	      {
	        ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
	                                                                         (uint8_t *)req, 0U);
	      }
	      break;

	    case USB_REQ_TYPE_STANDARD:
	      switch (req->bRequest)
	      {
	        case USB_REQ_GET_STATUS:
	          if (pdev->dev_state == USBD_STATE_CONFIGURED)
	          {
	            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
	          }
	          else
	          {
	            USBD_CtlError(pdev, req);
	            ret = USBD_FAIL;
	          }
	          break;

	        case USB_REQ_GET_INTERFACE:
	          if (pdev->dev_state == USBD_STATE_CONFIGURED)
	          {
	            (void)USBD_CtlSendData(pdev, &ifalt, 1U);
	          }
	          else
	          {
	            USBD_CtlError(pdev, req);
	            ret = USBD_FAIL;
	          }
	          break;

	        case USB_REQ_SET_INTERFACE:
	          if (pdev->dev_state != USBD_STATE_CONFIGURED)
	          {
	            USBD_CtlError(pdev, req);
	            ret = USBD_FAIL;
	          }
	          break;

	        case USB_REQ_CLEAR_FEATURE:
	          break;

	        default:
	          USBD_CtlError(pdev, req);
	          ret = USBD_FAIL;
	          break;
	      }
	      break;

	    default:
	      USBD_CtlError(pdev, req);
	      ret = USBD_FAIL;
	      break;
	  }

	  return (uint8_t)ret;


//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//  static uint8_t ifalt = 0;
//
//  switch (req->bmRequest & USB_REQ_TYPE_MASK)
//  {
//  case USB_REQ_TYPE_CLASS :
//    if (req->wLength)
//    {
//      if (req->bmRequest & 0x80)
//      {
//        ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
//                                                          (uint8_t *)hcdc->data,
//                                                          req->wLength);
//          USBD_CtlSendData (pdev,
//                            (uint8_t *)hcdc->data,
//                            req->wLength);
//      }
//      else
//      {
//        hcdc->CmdOpCode = req->bRequest;
//        hcdc->CmdLength = req->wLength;
//
//        USBD_CtlPrepareRx (pdev,
//                           (uint8_t *)hcdc->data,
//                           req->wLength);
//      }
//
//    }
//    else
//    {
//      ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
//                                                        (uint8_t*)req,
//                                                        0);
//    }
//    break;
//
//  case USB_REQ_TYPE_STANDARD:
//    switch (req->bRequest)
//    {
//    case USB_REQ_GET_INTERFACE :
//      USBD_CtlSendData (pdev,
//                        &ifalt,
//                        1);
//      break;
//
//    case USB_REQ_SET_INTERFACE :
//      break;
//    }
//
//  default:
//    break;
//  }
//  return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

	 USBD_CDC_HandleTypeDef *hcdc;
		  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;

		  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
		  {
		    return (uint8_t)USBD_FAIL;
		  }

		  hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

		  if ((pdev->ep_in[epnum & 0xFU].total_length > 0U) &&
		      ((pdev->ep_in[epnum & 0xFU].total_length % hpcd->IN_ep[epnum & 0xFU].maxpacket) == 0U))
		  {
		    /* Update the packet total length */
		    pdev->ep_in[epnum & 0xFU].total_length = 0U;

		    /* Send ZLP */
		    (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
		  }
		  else
		  {
		    hcdc->TxState = 0U;

		    if (((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt != NULL)
		    {
		      ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt(hcdc->TxBuffer, &hcdc->TxLength, epnum);
		    }
		  }

		  return (uint8_t)USBD_OK;
//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//  if(pdev->pClassData != NULL)
//  {
//
//    hcdc->TxState = 0;
//
//    return USBD_OK;
//  }
//  else
//  {
//    return USBD_FAIL;
//  }
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{



	  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	   if (pdev->pClassDataCmsit[pdev->classId] == NULL)
	   {
	     return (uint8_t)USBD_FAIL;
	   }

	   /* Get the received data length */
	   hcdc->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

	   /* USB data will be immediately processed, this allow next USB traffic being
	   NAKed till the end of the application Xfer */

	   ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Receive(hcdc->RxBuffer, &hcdc->RxLength);

	   return (uint8_t)USBD_OK;


//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//  /* Get the received data length */
//  hcdc->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
//
//  /* USB data will be immediately processed, this allow next USB traffic being
//  NAKed till the end of the application Xfer */
//  if(pdev->pClassData != NULL)
//  {
//    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Receive(hcdc->RxBuffer, &hcdc->RxLength);
//
//    return USBD_OK;
//  }
//  else
//  {
//    return USBD_FAIL;
//  }
}



/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{

	 USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	  if (hcdc == NULL)
	  {
	    return (uint8_t)USBD_FAIL;
	  }

	  if ((pdev->pUserData[pdev->classId] != NULL) && (hcdc->CmdOpCode != 0xFFU))
	  {
	    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(hcdc->CmdOpCode,
	                                                                     (uint8_t *)hcdc->data,
	                                                                     (uint16_t)hcdc->CmdLength);
	    hcdc->CmdOpCode = 0xFFU;
	  }

	  return (uint8_t)USBD_OK;


}
//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//  if((pdev->pUserData[pdev->classId] != NULL) && (hcdc->CmdOpCode != 0xFF))
//  {
//    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(hcdc->CmdOpCode,
//                                                      (uint8_t *)hcdc->data,
//                                                      hcdc->CmdLength);
//      hcdc->CmdOpCode = 0xFF;
//
//  }
//  return USBD_OK;
//}

/**
  * @brief  USBD_CDC_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length)
{

	 USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_CMD_EP);
	  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_OUT_EP);
	  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_IN_EP);

	  if (pEpCmdDesc != NULL)
	  {
	    pEpCmdDesc->bInterval = CDC_FS_BINTERVAL;
	  }

	  if (pEpOutDesc != NULL)
	  {
	    pEpOutDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
	  }

	  if (pEpInDesc != NULL)
	  {
	    pEpInDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
	  }

	  *length = (uint16_t)sizeof(USBD_CDC_CfgDesc);
	  return USBD_CDC_CfgDesc;


//  *length = sizeof (USBD_CDC_CfgFSDesc);
//  return USBD_CDC_CfgFSDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_CDC_DeviceQualifierDesc);
  return USBD_CDC_DeviceQualifierDesc;
}

/**
* @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                      USBD_CDC_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;

  if(fops != NULL)
  {
    pdev->pUserData[pdev->classId]= fops;
    ret = USBD_OK;
  }

  return ret;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */





uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint32_t length)
{
	  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	  if (hcdc == NULL)
	   {
	     return (uint8_t)USBD_FAIL;
	   }

	   hcdc->TxBuffer = pbuff;
	   hcdc->TxLength = length;

	   return (uint8_t)USBD_OK;


//	  if (hcdc == NULL)
//	  {
//	    return (uint8_t)USBD_FAIL;
//	  }
//
//	  hcdc->RxBuffer = pbuff;
//
//	  return (uint8_t)USBD_OK;


//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//  hcdc->TxBuffer = pbuff;
//  hcdc->TxLength = length;
//
//  return USBD_OK;
}


/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{

	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	  if (hcdc == NULL)
	  {
	    return (uint8_t)USBD_FAIL;
	  }

	  hcdc->RxBuffer = pbuff;

	  return (uint8_t)USBD_OK;

//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//  hcdc->RxBuffer = pbuff;
//
//  return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */

uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];


  USBD_StatusTypeDef ret = USBD_BUSY;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (hcdc->TxState == 0U)
  {
    /* Tx Transfer in progress */
    hcdc->TxState = 1U;

    /* Update the packet total length */
    pdev->ep_in[CDCInEpAdd & 0xFU].total_length = hcdc->TxLength;

    /* Transmit next packet */
    (void)USBD_LL_Transmit(pdev, CDCInEpAdd, hcdc->TxBuffer, hcdc->TxLength);

    ret = USBD_OK;
  }

  return (uint8_t)ret;
}

//uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
//{
//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//  if(pdev->pClassData != NULL)
//  {
//    if(hcdc->TxState == 0)
//    {
//      /* Tx Transfer in progress */
//      hcdc->TxState = 1;
//
//      /* Transmit next packet */
//      USBD_LL_Transmit(pdev,
//                       CDC_IN_EP,
//                       hcdc->TxBuffer,
//                       hcdc->TxLength);
//
//      return USBD_OK;
//    }
//    else
//    {
//      return USBD_BUSY;
//    }
//  }
//  else
//  {
//    return USBD_FAIL;
//  }
//}


/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
 // USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  /* Suspend or Resume USB Out process */

	 if (pdev->pClassDataCmsit[pdev->classId] == NULL)
	  {
	    return (uint8_t)USBD_FAIL;
	  }

	  if (pdev->dev_speed == USBD_SPEED_HIGH)
	  {
	    /* Prepare Out endpoint to receive next packet */
	    (void)USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer,
	                                 CDC_DATA_HS_OUT_PACKET_SIZE);
	  }
	  else
	  {
	    /* Prepare Out endpoint to receive next packet */
	    (void)USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer,
	                                 CDC_DATA_FS_OUT_PACKET_SIZE);
	  }

	  return (uint8_t)USBD_OK;



	//  if(pdev->pClassData != NULL)
//  {
//    if(pdev->dev_speed == USBD_SPEED_HIGH  )
//    {
//      /* Prepare Out endpoint to receive next packet */
//      USBD_LL_PrepareReceive(pdev,
//                             CDC_OUT_EP,
//                             hcdc->RxBuffer,
//                             CDC_DATA_HS_OUT_PACKET_SIZE);
//    }
//    else
//    {
//      /* Prepare Out endpoint to receive next packet */
//      USBD_LL_PrepareReceive(pdev,
//                             CDC_OUT_EP,
//                             hcdc->RxBuffer,
//                             CDC_DATA_FS_OUT_PACKET_SIZE);
//    }
//    return USBD_OK;
//  }
//  else
//  {
//    return USBD_FAIL;
//  }
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
