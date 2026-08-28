/****************************************************************************
 * drivers/usbdev/usbdev_custom_hid.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/queue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/hid.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/usbdev_custom_hid.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CUSTOM_HID_NPOLLWAITERS
#  define CONFIG_CUSTOM_HID_NPOLLWAITERS  2
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER  100
#endif

/* Device / configuration constants *****************************************/

#define CUSTOM_HID_CONFIGID           1
#define CUSTOM_HID_CONFIGIDNONE       0
#define CUSTOM_HID_NINTERFACES        1
#define CUSTOM_HID_NENDPOINTS         2
#define CUSTOM_HID_VERSIONNO          0x0100  /* Device release 1.00 (BCD) */
#define CUSTOM_HID_HIDVERSION         0x0111  /* HID class release 1.11 (BCD) */
#define CUSTOM_HID_STR_LANGUAGE       0x0409  /* en-us */

/* String descriptor IDs */

#define CUSTOM_HID_LANGSTRID          0
#define CUSTOM_HID_MFGSTRID           1
#define CUSTOM_HID_PRODUCTSTRID       2
#define CUSTOM_HID_SERIALSTRID        3
#define CUSTOM_HID_NSTRIDS            3   /* Not counting language ID */

/* Endpoint index into devinfo.epno[] */

#define CUSTOM_HID_EP_INTIN_IDX       0
#define CUSTOM_HID_EP_INTOUT_IDX      1

#define CUSTOM_HID_MKEPINTIN(di)      (USB_DIR_IN  | (di)->epno[CUSTOM_HID_EP_INTIN_IDX])
#define CUSTOM_HID_MKEPINTOUT(di)     (USB_DIR_OUT | (di)->epno[CUSTOM_HID_EP_INTOUT_IDX])

/* Total size of the configuration descriptor bundle:
 *   config(9) + interface(9) + HID(9) + EP-IN(7) + EP-OUT(7) = 41
 */

#define CUSTOM_HID_CFGDESC_TOTALLEN \
  (USB_SIZEOF_CFGDESC + USB_SIZEOF_IFDESC + 9 + 2 * USB_SIZEOF_EPDESC)

/* Big enough for the largest descriptor returned via EP0 (report desc) */

#define CUSTOM_HID_MXDESCLEN          128
#define CUSTOM_HID_MAXSTRLEN          (CUSTOM_HID_MXDESCLEN - 2)

/* Receive message FIFO depth */

#define CUSTOM_HID_RXFIFO_DEPTH       (CONFIG_CUSTOM_HID_NRDREQS + 2)

#define CUSTOM_HID_DEVNAME_SIZE       16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of write requests */

struct custom_hid_wrreq_s
{
  FAR struct custom_hid_wrreq_s *flink;  /* Singly linked list */
  FAR struct usbdev_req_s       *req;    /* The contained request */
};

/* Container to support a list of read requests */

struct custom_hid_rdreq_s
{
  FAR struct custom_hid_rdreq_s *flink;  /* Singly linked list */
  FAR struct usbdev_req_s       *req;    /* The contained request */
};

/* One received HID output report */

struct custom_hid_msg_s
{
  uint16_t len;
  uint8_t  data[CONFIG_CUSTOM_HID_EPSIZE];
};

/* This structure describes the internal state of the driver */

struct custom_hid_dev_s
{
  FAR struct usbdev_s     *usbdev;    /* usbdev driver pointer */
  uint8_t                  config;    /* Selected configuration number */
  uint8_t                  nwrq;      /* Number of write requests available */
  uint8_t                  minor;     /* Device minor number */
  uint8_t                  idlerate;  /* HID SET_IDLE rate */
  uint8_t                  protocol;  /* HID SET_PROTOCOL value */
  bool                     connected; /* True: configured and usable */
  bool                     unlinked;  /* True: char device unlinked */
  uint8_t                  crefs;     /* Open reference count */

  FAR struct usbdev_ep_s  *epintin;   /* Interrupt IN endpoint */
  FAR struct usbdev_ep_s  *epintout;  /* Interrupt OUT endpoint */
  FAR struct usbdev_req_s *ctrlreq;   /* Preallocated EP0 request */

  struct sq_queue_s        txfree;    /* Available write containers */

  struct usbdev_devinfo_s  devinfo;

  mutex_t                  lock;       /* Char-device exclusive access */
  sem_t                    rxsem;      /* Signals data available to read() */
  spinlock_t               spinlock;   /* Protects the RX FIFO / tx list */

  /* Receive FIFO (fed by the interrupt OUT EP and by EP0 SET_REPORT) */

  uint8_t                  rxhead;
  uint8_t                  rxtail;
  uint8_t                  rxcount;
  struct custom_hid_msg_s  rxfifo[CUSTOM_HID_RXFIFO_DEPTH];

  FAR struct pollfd       *fds[CONFIG_CUSTOM_HID_NPOLLWAITERS];

  /* Preallocated request containers */

  struct custom_hid_wrreq_s wrreqs[CONFIG_CUSTOM_HID_NWRREQS];
  struct custom_hid_rdreq_s rdreqs[CONFIG_CUSTOM_HID_NRDREQS];
};

/* The internal version of the class driver */

struct custom_hid_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct custom_hid_dev_s *dev;
};

/* This is what is allocated */

struct custom_hid_alloc_s
{
  struct custom_hid_dev_s    dev;
  struct custom_hid_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* USB class device *********************************************************/

static int  custom_hid_bind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev);
static void custom_hid_unbind(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev);
static int  custom_hid_setup(FAR struct usbdevclass_driver_s *driver,
                             FAR struct usbdev_s *dev,
                             FAR const struct usb_ctrlreq_s *ctrl,
                             FAR uint8_t *dataout, size_t outlen);
static void custom_hid_disconnect(FAR struct usbdevclass_driver_s *driver,
                                  FAR struct usbdev_s *dev);

/* Completion callbacks *****************************************************/

static void custom_hid_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                     FAR struct usbdev_req_s *req);
static void custom_hid_rdcomplete(FAR struct usbdev_ep_s *ep,
                                  FAR struct usbdev_req_s *req);
static void custom_hid_wrcomplete(FAR struct usbdev_ep_s *ep,
                                  FAR struct usbdev_req_s *req);

/* Char device operations ***************************************************/

static int     custom_hid_open(FAR struct file *filep);
static int     custom_hid_close(FAR struct file *filep);
static ssize_t custom_hid_read(FAR struct file *filep, FAR char *buffer,
                               size_t len);
static ssize_t custom_hid_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len);
static int     custom_hid_poll(FAR struct file *filep,
                               FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
  custom_hid_bind,        /* bind */
  custom_hid_unbind,      /* unbind */
  custom_hid_setup,       /* setup */
  custom_hid_disconnect,  /* disconnect */
  NULL,                   /* suspend */
  NULL,                   /* resume */
};

static const struct file_operations g_custom_hid_fops =
{
  custom_hid_open,   /* open */
  custom_hid_close,  /* close */
  custom_hid_read,   /* read */
  custom_hid_write,  /* write */
  NULL,              /* seek */
  NULL,              /* ioctl */
  NULL,              /* mmap */
  NULL,              /* truncate */
  custom_hid_poll    /* poll */
};

/* USB device descriptor (per-interface class) */

static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {
    LSBYTE(0x0200), MSBYTE(0x0200)              /* usb 2.00 */
  },
  0,                                            /* classid (per interface) */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_CUSTOM_HID_EP0MAXPACKET,               /* mxpacketsize (ep0) */
  {
    LSBYTE(CONFIG_CUSTOM_HID_VENDORID),         /* vendor */
    MSBYTE(CONFIG_CUSTOM_HID_VENDORID)
  },
  {
    LSBYTE(CONFIG_CUSTOM_HID_PRODUCTID),        /* product */
    MSBYTE(CONFIG_CUSTOM_HID_PRODUCTID)
  },
  {
    LSBYTE(CUSTOM_HID_VERSIONNO),               /* device */
    MSBYTE(CUSTOM_HID_VERSIONNO)
  },
  CUSTOM_HID_MFGSTRID,                          /* imfgr */
  CUSTOM_HID_PRODUCTSTRID,                      /* iproduct */
  CUSTOM_HID_SERIALSTRID,                       /* serno */
  1                                             /* nconfigs */
};

/* HID report descriptor (vendor-defined, matches GD32 standard library) */

static const uint8_t g_hid_report_desc[] =
{
  0x06, 0x00, 0xff,   /* USAGE_PAGE (Vendor Defined 0xFF00) */
  0x09, 0x00,         /* USAGE (Custom Device) */
  0xa1, 0x01,         /* COLLECTION (Application) */

  /* LED1 (report id 0x11) */

  0x85, 0x11, 0x09, 0x01, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x08, 0x95, 0x01, 0x91, 0x82,

  /* LED2 (report id 0x12) */

  0x85, 0x12, 0x09, 0x02, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x08, 0x95, 0x01, 0x91, 0x82,

  /* LED3 (report id 0x13) */

  0x85, 0x13, 0x09, 0x03, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x08, 0x95, 0x01, 0x91, 0x82,

  /* LED4 (report id 0x14) */

  0x85, 0x14, 0x09, 0x04, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x08, 0x95, 0x01, 0x91, 0x82,

  /* Wakeup key (report id 0x15) */

  0x85, 0x15, 0x09, 0x05, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x01, 0x81, 0x02,
  0x75, 0x07, 0x81, 0x03,

  /* Tamper key (report id 0x16) */

  0x85, 0x16, 0x09, 0x06, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x01, 0x81, 0x02,
  0x75, 0x07, 0x81, 0x03,

  0xc0                /* END_COLLECTION */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: custom_hid_mkhiddesc
 *
 * Description:
 *   Fill in the HID descriptor.
 *
 ****************************************************************************/

static void custom_hid_mkhiddesc(FAR struct usbhid_descriptor_s *hiddesc)
{
  hiddesc->len       = sizeof(struct usbhid_descriptor_s);
  hiddesc->type      = USBHID_DESCTYPE_HID;
  hiddesc->hid[0]    = LSBYTE(CUSTOM_HID_HIDVERSION);
  hiddesc->hid[1]    = MSBYTE(CUSTOM_HID_HIDVERSION);
  hiddesc->country   = USBHID_COUNTRY_NONE;
  hiddesc->ndesc     = 1;
  hiddesc->classdesc = USBHID_DESCTYPE_REPORT;
  hiddesc->desclen[0] = LSBYTE(sizeof(g_hid_report_desc));
  hiddesc->desclen[1] = MSBYTE(sizeof(g_hid_report_desc));
}

/****************************************************************************
 * Name: custom_hid_mkepdesc
 *
 * Description:
 *   Fill in one interrupt endpoint descriptor.
 *
 ****************************************************************************/

static void custom_hid_mkepdesc(FAR struct usb_epdesc_s *epdesc,
                                uint8_t addr)
{
  epdesc->len            = USB_SIZEOF_EPDESC;
  epdesc->type           = USB_DESC_TYPE_ENDPOINT;
  epdesc->addr           = addr;
  epdesc->attr           = USB_EP_ATTR_XFER_INT;
  epdesc->mxpacketsize[0] = LSBYTE(CONFIG_CUSTOM_HID_EPSIZE);
  epdesc->mxpacketsize[1] = MSBYTE(CONFIG_CUSTOM_HID_EPSIZE);
  epdesc->interval       = CONFIG_CUSTOM_HID_EPINTERVAL;
}

/****************************************************************************
 * Name: custom_hid_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor bundle.
 *
 ****************************************************************************/

static int custom_hid_mkcfgdesc(FAR uint8_t *buf,
                                FAR struct usbdev_devinfo_s *devinfo)
{
  FAR struct usb_cfgdesc_s *cfgdesc = (FAR struct usb_cfgdesc_s *)buf;
  FAR struct usb_ifdesc_s *ifdesc;
  FAR struct usbhid_descriptor_s *hiddesc;
  FAR struct usb_epdesc_s *epdesc;
  int offset = 0;

  /* Configuration descriptor */

  cfgdesc->len         = USB_SIZEOF_CFGDESC;
  cfgdesc->type        = USB_DESC_TYPE_CONFIG;
  cfgdesc->totallen[0] = LSBYTE(CUSTOM_HID_CFGDESC_TOTALLEN);
  cfgdesc->totallen[1] = MSBYTE(CUSTOM_HID_CFGDESC_TOTALLEN);
  cfgdesc->ninterfaces = CUSTOM_HID_NINTERFACES;
  cfgdesc->cfgvalue    = CUSTOM_HID_CONFIGID;
  cfgdesc->icfg        = 0;
  cfgdesc->attr        = USB_CONFIG_ATTR_ONE | USB_CONFIG_ATTR_SELFPOWER;
  cfgdesc->mxpower     = (CONFIG_USBDEV_MAXPOWER + 1) / 2;
  offset += USB_SIZEOF_CFGDESC;

  /* Interface descriptor */

  ifdesc            = (FAR struct usb_ifdesc_s *)(buf + offset);
  ifdesc->len       = USB_SIZEOF_IFDESC;
  ifdesc->type      = USB_DESC_TYPE_INTERFACE;
  ifdesc->ifno      = devinfo->ifnobase;
  ifdesc->alt       = 0;
  ifdesc->neps      = CUSTOM_HID_NENDPOINTS;
  ifdesc->classid   = USB_CLASS_HID;
  ifdesc->subclass  = USBHID_SUBCLASS_NONE;
  ifdesc->protocol  = USBHID_PROTOCOL_NONE;
  ifdesc->iif       = 0;
  offset += USB_SIZEOF_IFDESC;

  /* HID descriptor */

  hiddesc = (FAR struct usbhid_descriptor_s *)(buf + offset);
  custom_hid_mkhiddesc(hiddesc);
  offset += sizeof(struct usbhid_descriptor_s);

  /* Interrupt IN endpoint descriptor */

  epdesc = (FAR struct usb_epdesc_s *)(buf + offset);
  custom_hid_mkepdesc(epdesc, CUSTOM_HID_MKEPINTIN(devinfo));
  offset += USB_SIZEOF_EPDESC;

  /* Interrupt OUT endpoint descriptor */

  epdesc = (FAR struct usb_epdesc_s *)(buf + offset);
  custom_hid_mkepdesc(epdesc, CUSTOM_HID_MKEPINTOUT(devinfo));
  offset += USB_SIZEOF_EPDESC;

  return offset;
}

/****************************************************************************
 * Name: custom_hid_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor.
 *
 ****************************************************************************/

static int custom_hid_mkstrdesc(uint8_t id,
                                FAR struct usb_strdesc_s *strdesc)
{
  FAR uint8_t *data = (FAR uint8_t *)(strdesc + 1);
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
    case CUSTOM_HID_LANGSTRID:
      strdesc->len  = 4;
      strdesc->type = USB_DESC_TYPE_STRING;
      data[0] = LSBYTE(CUSTOM_HID_STR_LANGUAGE);
      data[1] = MSBYTE(CUSTOM_HID_STR_LANGUAGE);
      return 4;

    case CUSTOM_HID_MFGSTRID:
      str = CONFIG_CUSTOM_HID_VENDORSTR;
      break;

    case CUSTOM_HID_PRODUCTSTRID:
      str = CONFIG_CUSTOM_HID_PRODUCTSTR;
      break;

    case CUSTOM_HID_SERIALSTRID:
      str = CONFIG_CUSTOM_HID_SERIALSTR;
      break;

    default:
      return -EINVAL;
    }

  /* The string is utf16-le.  Convert 7-bit ascii to utf16-le. */

  len = strlen(str);
  if (len > (CUSTOM_HID_MAXSTRLEN / 2))
    {
      len = (CUSTOM_HID_MAXSTRLEN / 2);
    }

  for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
      data[ndata]     = str[i];
      data[ndata + 1] = 0;
    }

  strdesc->len  = ndata + 2;
  strdesc->type = USB_DESC_TYPE_STRING;
  return strdesc->len;
}

/****************************************************************************
 * Name: custom_hid_rxpush
 *
 * Description:
 *   Push a received report into the RX FIFO and wake any waiters.  Must be
 *   called with the spinlock held.
 *
 ****************************************************************************/

static void custom_hid_rxpush(FAR struct custom_hid_dev_s *priv,
                              FAR const uint8_t *data, uint16_t len)
{
  FAR struct custom_hid_msg_s *msg;

  if (priv->rxcount >= CUSTOM_HID_RXFIFO_DEPTH)
    {
      /* FIFO full: drop the oldest message */

      priv->rxtail = (priv->rxtail + 1) % CUSTOM_HID_RXFIFO_DEPTH;
      priv->rxcount--;
    }

  if (len > CONFIG_CUSTOM_HID_EPSIZE)
    {
      len = CONFIG_CUSTOM_HID_EPSIZE;
    }

  msg = &priv->rxfifo[priv->rxhead];
  msg->len = len;
  memcpy(msg->data, data, len);

  priv->rxhead = (priv->rxhead + 1) % CUSTOM_HID_RXFIFO_DEPTH;
  priv->rxcount++;
}

/****************************************************************************
 * Name: custom_hid_resetconfig
 ****************************************************************************/

static void custom_hid_resetconfig(FAR struct custom_hid_dev_s *priv)
{
  if (priv->config != CUSTOM_HID_CONFIGIDNONE)
    {
      priv->config    = CUSTOM_HID_CONFIGIDNONE;
      priv->connected = false;

      EP_DISABLE(priv->epintin);
      EP_DISABLE(priv->epintout);
    }
}

/****************************************************************************
 * Name: custom_hid_setconfig
 ****************************************************************************/

static int custom_hid_setconfig(FAR struct custom_hid_dev_s *priv,
                                uint8_t config)
{
  struct usb_epdesc_s epdesc;
  irqstate_t flags;
  int ret;
  int i;

  if (config == priv->config)
    {
      return OK;
    }

  custom_hid_resetconfig(priv);

  if (config == CUSTOM_HID_CONFIGIDNONE)
    {
      return OK;
    }

  if (config != CUSTOM_HID_CONFIGID)
    {
      return -EINVAL;
    }

  /* Configure the IN interrupt endpoint */

  custom_hid_mkepdesc(&epdesc, CUSTOM_HID_MKEPINTIN(&priv->devinfo));
  ret = EP_CONFIGURE(priv->epintin, &epdesc, false);
  if (ret < 0)
    {
      goto errout;
    }

  priv->epintin->priv = priv;

  /* Configure the OUT interrupt endpoint */

  custom_hid_mkepdesc(&epdesc, CUSTOM_HID_MKEPINTOUT(&priv->devinfo));
  ret = EP_CONFIGURE(priv->epintout, &epdesc, true);
  if (ret < 0)
    {
      goto errout;
    }

  priv->epintout->priv = priv;

  /* Queue read requests on the OUT endpoint */

  for (i = 0; i < CONFIG_CUSTOM_HID_NRDREQS; i++)
    {
      FAR struct usbdev_req_s *req = priv->rdreqs[i].req;
      req->callback = custom_hid_rdcomplete;
      req->len      = CONFIG_CUSTOM_HID_EPSIZE;
      ret = EP_SUBMIT(priv->epintout, req);
      if (ret < 0)
        {
          goto errout;
        }
    }

  flags = spin_lock_irqsave(&priv->spinlock);
  priv->config    = config;
  priv->connected = true;
  spin_unlock_irqrestore(&priv->spinlock, flags);

  return OK;

errout:
  custom_hid_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: custom_hid_ep0incomplete
 ****************************************************************************/

static void custom_hid_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                     FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      uerr("ERROR: EP0 result: %d\n", req->result);
    }
}

/****************************************************************************
 * Name: custom_hid_rdcomplete
 *
 * Description:
 *   Handle completion of a read request on the interrupt OUT endpoint.
 *
 ****************************************************************************/

static void custom_hid_rdcomplete(FAR struct usbdev_ep_s *ep,
                                  FAR struct usbdev_req_s *req)
{
  FAR struct custom_hid_dev_s *priv =
    (FAR struct custom_hid_dev_s *)ep->priv;
  irqstate_t flags;
  int ret;

  switch (req->result)
    {
    case OK:
      if (req->xfrd > 0)
        {
          flags = spin_lock_irqsave(&priv->spinlock);
          custom_hid_rxpush(priv, req->buf, req->xfrd);
          spin_unlock_irqrestore(&priv->spinlock, flags);

          nxsem_post(&priv->rxsem);
          poll_notify(priv->fds, CONFIG_CUSTOM_HID_NPOLLWAITERS, POLLIN);
        }
      break;

    case -ESHUTDOWN:  /* Disconnection */
      return;

    default:
      uerr("ERROR: RX result: %d\n", req->result);
      break;
    }

  /* Re-arm the read request */

  req->len = CONFIG_CUSTOM_HID_EPSIZE;
  ret = EP_SUBMIT(ep, req);
  if (ret < 0)
    {
      uerr("ERROR: EP_SUBMIT failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: custom_hid_wrcomplete
 *
 * Description:
 *   Handle completion of a write request on the interrupt IN endpoint.
 *
 ****************************************************************************/

static void custom_hid_wrcomplete(FAR struct usbdev_ep_s *ep,
                                  FAR struct usbdev_req_s *req)
{
  FAR struct custom_hid_dev_s *priv =
    (FAR struct custom_hid_dev_s *)ep->priv;
  FAR struct custom_hid_wrreq_s *container =
    (FAR struct custom_hid_wrreq_s *)req->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->spinlock);
  sq_addlast((FAR sq_entry_t *)container, &priv->txfree);
  priv->nwrq++;
  spin_unlock_irqrestore(&priv->spinlock, flags);

  if (req->result != OK && req->result != -ESHUTDOWN)
    {
      uerr("ERROR: TX result: %d\n", req->result);
    }

  poll_notify(priv->fds, CONFIG_CUSTOM_HID_NPOLLWAITERS, POLLOUT);
}

/****************************************************************************
 * Name: custom_hid_bind
 ****************************************************************************/

static int custom_hid_bind(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev)
{
  FAR struct custom_hid_dev_s *priv =
    ((FAR struct custom_hid_driver_s *)driver)->dev;
  irqstate_t flags;
  int ret;
  int i;

  priv->usbdev  = dev;
  dev->ep0->priv = priv;

  /* Preallocate the control request */

  priv->ctrlreq = usbdev_allocreq(dev->ep0, CUSTOM_HID_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  priv->ctrlreq->callback = custom_hid_ep0incomplete;

  /* Preallocate the interrupt IN endpoint */

  priv->epintin = DEV_ALLOCEP(dev, CUSTOM_HID_MKEPINTIN(&priv->devinfo),
                              true, USB_EP_ATTR_XFER_INT);
  if (priv->epintin == NULL)
    {
      ret = -ENODEV;
      goto errout;
    }

  priv->epintin->priv = priv;

  /* Preallocate the interrupt OUT endpoint */

  priv->epintout = DEV_ALLOCEP(dev, CUSTOM_HID_MKEPINTOUT(&priv->devinfo),
                               false, USB_EP_ATTR_XFER_INT);
  if (priv->epintout == NULL)
    {
      ret = -ENODEV;
      goto errout;
    }

  priv->epintout->priv = priv;

  /* Preallocate read requests */

  for (i = 0; i < CONFIG_CUSTOM_HID_NRDREQS; i++)
    {
      FAR struct custom_hid_rdreq_s *rdcontainer = &priv->rdreqs[i];
      rdcontainer->req = usbdev_allocreq(priv->epintout,
                                         CONFIG_CUSTOM_HID_EPSIZE);
      if (rdcontainer->req == NULL)
        {
          ret = -ENOMEM;
          goto errout;
        }

      rdcontainer->req->priv     = rdcontainer;
      rdcontainer->req->callback = custom_hid_rdcomplete;
    }

  /* Preallocate write requests */

  for (i = 0; i < CONFIG_CUSTOM_HID_NWRREQS; i++)
    {
      FAR struct custom_hid_wrreq_s *wrcontainer = &priv->wrreqs[i];
      wrcontainer->req = usbdev_allocreq(priv->epintin,
                                         CONFIG_CUSTOM_HID_EPSIZE);
      if (wrcontainer->req == NULL)
        {
          ret = -ENOMEM;
          goto errout;
        }

      wrcontainer->req->priv     = wrcontainer;
      wrcontainer->req->callback = custom_hid_wrcomplete;

      flags = spin_lock_irqsave(&priv->spinlock);
      sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->txfree);
      priv->nwrq++;
      spin_unlock_irqrestore(&priv->spinlock, flags);
    }

  /* Report that we are self-powered */

  DEV_SETSELFPOWERED(dev);

  /* And pull up the data line for the soft connect function */

  DEV_CONNECT(dev);
  return OK;

errout:
  custom_hid_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: custom_hid_unbind
 ****************************************************************************/

static void custom_hid_unbind(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev)
{
  FAR struct custom_hid_dev_s *priv =
    ((FAR struct custom_hid_driver_s *)driver)->dev;
  int i;

  if (priv == NULL)
    {
      return;
    }

  custom_hid_resetconfig(priv);

  /* Free the interrupt IN endpoint and its write requests */

  if (priv->epintin != NULL)
    {
      for (i = 0; i < CONFIG_CUSTOM_HID_NWRREQS; i++)
        {
          if (priv->wrreqs[i].req != NULL)
            {
              usbdev_freereq(priv->epintin, priv->wrreqs[i].req);
              priv->wrreqs[i].req = NULL;
            }
        }

      DEV_FREEEP(dev, priv->epintin);
      priv->epintin = NULL;
    }

  /* Free the interrupt OUT endpoint and its read requests */

  if (priv->epintout != NULL)
    {
      for (i = 0; i < CONFIG_CUSTOM_HID_NRDREQS; i++)
        {
          if (priv->rdreqs[i].req != NULL)
            {
              usbdev_freereq(priv->epintout, priv->rdreqs[i].req);
              priv->rdreqs[i].req = NULL;
            }
        }

      DEV_FREEEP(dev, priv->epintout);
      priv->epintout = NULL;
    }

  /* Free the control request */

  if (priv->ctrlreq != NULL)
    {
      usbdev_freereq(dev->ep0, priv->ctrlreq);
      priv->ctrlreq = NULL;
    }

  sq_init(&priv->txfree);
  priv->nwrq = 0;
}

/****************************************************************************
 * Name: custom_hid_setup
 ****************************************************************************/

static int custom_hid_setup(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev,
                            FAR const struct usb_ctrlreq_s *ctrl,
                            FAR uint8_t *dataout, size_t outlen)
{
  FAR struct custom_hid_dev_s *priv =
    ((FAR struct custom_hid_driver_s *)driver)->dev;
  FAR struct usbdev_req_s *ctrlreq = priv->ctrlreq;
  uint16_t value;
  uint16_t len;
  int ret = -EOPNOTSUPP;

  value = GETUINT16(ctrl->value);
  len   = GETUINT16(ctrl->len);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      switch (ctrl->req)
        {
        case USB_REQ_GETDESCRIPTOR:
          switch (ctrl->value[1])
            {
            case USB_DESC_TYPE_DEVICE:
              ret = usbdev_copy_devdesc(ctrlreq->buf,
                                        (FAR const struct usb_devdesc_s *)
                                        &g_devdesc, dev->speed);
              break;

            case USB_DESC_TYPE_CONFIG:
              ret = custom_hid_mkcfgdesc(ctrlreq->buf, &priv->devinfo);
              break;

            case USB_DESC_TYPE_STRING:
              ret = custom_hid_mkstrdesc(ctrl->value[0],
                     (FAR struct usb_strdesc_s *)ctrlreq->buf);
              break;

            case USBHID_DESCTYPE_HID:
              custom_hid_mkhiddesc((FAR struct usbhid_descriptor_s *)
                                   ctrlreq->buf);
              ret = sizeof(struct usbhid_descriptor_s);
              break;

            case USBHID_DESCTYPE_REPORT:
              memcpy(ctrlreq->buf, g_hid_report_desc,
                     sizeof(g_hid_report_desc));
              ret = sizeof(g_hid_report_desc);
              break;

            default:
              uwarn("WARNING: Unknown descriptor: %d\n", ctrl->value[1]);
              break;
            }
          break;

        case USB_REQ_SETCONFIGURATION:
          if (ctrl->type == 0)
            {
              ret = custom_hid_setconfig(priv, value);
            }
          break;

        case USB_REQ_GETCONFIGURATION:
          if (ctrl->type == USB_DIR_IN)
            {
              ctrlreq->buf[0] = priv->config;
              ret = 1;
            }
          break;

        case USB_REQ_SETINTERFACE:
          if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE &&
              priv->config == CUSTOM_HID_CONFIGID)
            {
              ret = 0;
            }
          break;

        case USB_REQ_GETINTERFACE:
          if (ctrl->type == (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE))
            {
              ctrlreq->buf[0] = 0;
              ret = 1;
            }
          break;

        default:
          uwarn("WARNING: Unsupported std req: %d\n", ctrl->req);
          break;
        }
    }
  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      switch (ctrl->req)
        {
        case USBHID_REQUEST_GETREPORT:

          /* Return a zero-filled report of the requested length */

          if (len > CONFIG_CUSTOM_HID_EPSIZE)
            {
              len = CONFIG_CUSTOM_HID_EPSIZE;
            }

          memset(ctrlreq->buf, 0, len);
          ctrlreq->buf[0] = ctrl->value[0];  /* Echo report ID */
          ret = len;
          break;

        case USBHID_REQUEST_SETREPORT:

          /* Host is delivering an output report over EP0 */

          if (dataout != NULL && outlen > 0)
            {
              irqstate_t flags = spin_lock_irqsave(&priv->spinlock);
              custom_hid_rxpush(priv, dataout, outlen);
              spin_unlock_irqrestore(&priv->spinlock, flags);

              nxsem_post(&priv->rxsem);
              poll_notify(priv->fds, CONFIG_CUSTOM_HID_NPOLLWAITERS, POLLIN);
            }

          ret = 0;
          break;

        case USBHID_REQUEST_GETIDLE:
          ctrlreq->buf[0] = priv->idlerate;
          ret = 1;
          break;

        case USBHID_REQUEST_SETIDLE:
          priv->idlerate = ctrl->value[1];
          ret = 0;
          break;

        case USBHID_REQUEST_GETPROTOCOL:
          ctrlreq->buf[0] = priv->protocol;
          ret = 1;
          break;

        case USBHID_REQUEST_SETPROTOCOL:
          priv->protocol = ctrl->value[0];
          ret = 0;
          break;

        default:
          uwarn("WARNING: Unsupported class req: %d\n", ctrl->req);
          break;
        }
    }

  /* Respond to the setup command if data was returned */

  if (ret >= 0)
    {
      ctrlreq->len   = (len < ret) ? len : ret;
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
      ret = EP_SUBMIT(dev->ep0, ctrlreq);
      if (ret < 0)
        {
          ctrlreq->result = OK;
          custom_hid_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: custom_hid_disconnect
 ****************************************************************************/

static void custom_hid_disconnect(FAR struct usbdevclass_driver_s *driver,
                                  FAR struct usbdev_s *dev)
{
  FAR struct custom_hid_dev_s *priv =
    ((FAR struct custom_hid_driver_s *)driver)->dev;
  irqstate_t flags;

  flags = enter_critical_section();
  custom_hid_resetconfig(priv);
  leave_critical_section(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated.
   */

  DEV_CONNECT(dev);
}

/****************************************************************************
 * Char Device Operations
 ****************************************************************************/

static int custom_hid_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct custom_hid_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->crefs++;
  nxmutex_unlock(&priv->lock);
  return OK;
}

static int custom_hid_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct custom_hid_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->crefs > 0)
    {
      priv->crefs--;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

static ssize_t custom_hid_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct custom_hid_dev_s *priv = inode->i_private;
  struct custom_hid_msg_s msg;
  irqstate_t flags;
  int ret;

  if (buffer == NULL || len == 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (; ; )
    {
      flags = spin_lock_irqsave(&priv->spinlock);
      if (priv->rxcount > 0)
        {
          msg = priv->rxfifo[priv->rxtail];
          priv->rxtail = (priv->rxtail + 1) % CUSTOM_HID_RXFIFO_DEPTH;
          priv->rxcount--;
          spin_unlock_irqrestore(&priv->spinlock, flags);
          break;
        }

      spin_unlock_irqrestore(&priv->spinlock, flags);

      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          nxmutex_unlock(&priv->lock);
          return -EAGAIN;
        }

      nxmutex_unlock(&priv->lock);
      ret = nxsem_wait(&priv->rxsem);
      if (ret < 0)
        {
          return ret;
        }

      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }
    }

  nxmutex_unlock(&priv->lock);

  if (msg.len > len)
    {
      msg.len = len;
    }

  memcpy(buffer, msg.data, msg.len);
  return msg.len;
}

static ssize_t custom_hid_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct custom_hid_dev_s *priv = inode->i_private;
  FAR struct custom_hid_wrreq_s *container;
  FAR struct usbdev_req_s *req;
  irqstate_t flags;
  int ret;

  if (buffer == NULL || len == 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->connected)
    {
      nxmutex_unlock(&priv->lock);
      return -ENOTCONN;
    }

  if (len > CONFIG_CUSTOM_HID_EPSIZE)
    {
      len = CONFIG_CUSTOM_HID_EPSIZE;
    }

  flags = spin_lock_irqsave(&priv->spinlock);
  container = (FAR struct custom_hid_wrreq_s *)sq_remfirst(&priv->txfree);
  if (container != NULL)
    {
      priv->nwrq--;
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);

  if (container == NULL)
    {
      nxmutex_unlock(&priv->lock);
      return -EAGAIN;
    }

  req = container->req;
  memcpy(req->buf, buffer, len);
  req->len   = len;
  req->priv  = container;
  req->flags = USBDEV_REQFLAGS_NULLPKT;

  ret = EP_SUBMIT(priv->epintin, req);
  if (ret < 0)
    {
      flags = spin_lock_irqsave(&priv->spinlock);
      sq_addlast((FAR sq_entry_t *)container, &priv->txfree);
      priv->nwrq++;
      spin_unlock_irqrestore(&priv->spinlock, flags);
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  nxmutex_unlock(&priv->lock);
  return len;
}

static int custom_hid_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct custom_hid_dev_s *priv = inode->i_private;
  pollevent_t eventset;
  irqstate_t flags;
  int ret;
  int i;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!setup)
    {
      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      if (slot != NULL)
        {
          *slot     = NULL;
          fds->priv = NULL;
        }

      goto errout;
    }

  for (i = 0; i < CONFIG_CUSTOM_HID_NPOLLWAITERS; i++)
    {
      if (priv->fds[i] == NULL)
        {
          priv->fds[i] = fds;
          fds->priv    = &priv->fds[i];
          break;
        }
    }

  if (i >= CONFIG_CUSTOM_HID_NPOLLWAITERS)
    {
      fds->priv = NULL;
      ret = -EBUSY;
      goto errout;
    }

  eventset = 0;

  flags = spin_lock_irqsave(&priv->spinlock);
  if (priv->rxcount > 0)
    {
      eventset |= POLLIN;
    }

  if (priv->connected && priv->nwrq > 0)
    {
      eventset |= POLLOUT;
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);

  poll_notify(priv->fds, CONFIG_CUSTOM_HID_NPOLLWAITERS, eventset);

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: custom_hid_initialize
 ****************************************************************************/

int custom_hid_initialize(int minor, FAR void **handle)
{
  FAR struct custom_hid_alloc_s *alloc;
  FAR struct custom_hid_dev_s *priv;
  FAR struct custom_hid_driver_s *drvr;
  char devname[CUSTOM_HID_DEVNAME_SIZE];
  int ret;

  alloc = (FAR struct custom_hid_alloc_s *)
    kmm_zalloc(sizeof(struct custom_hid_alloc_s));
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  sq_init(&priv->txfree);
  nxmutex_init(&priv->lock);
  nxsem_init(&priv->rxsem, 0, 0);
  spin_lock_init(&priv->spinlock);

  priv->minor = minor;

  /* Initialize the device information */

  priv->devinfo.ninterfaces = CUSTOM_HID_NINTERFACES;
  priv->devinfo.ifnobase    = 0;
  priv->devinfo.nstrings    = CUSTOM_HID_NSTRIDS;
  priv->devinfo.strbase     = 0;
  priv->devinfo.nendpoints  = CUSTOM_HID_NENDPOINTS;
  priv->devinfo.epno[CUSTOM_HID_EP_INTIN_IDX]  = CONFIG_CUSTOM_HID_EPIN;
  priv->devinfo.epno[CUSTOM_HID_EP_INTOUT_IDX] = CONFIG_CUSTOM_HID_EPOUT;

  /* Initialize the USB class driver structure */

  drvr->drvr.speed = USB_SPEED_FULL;
  drvr->drvr.ops   = &g_driverops;
  drvr->dev        = priv;

  /* Register the char device */

  snprintf(devname, sizeof(devname), "/dev/hid%d", minor);
  ret = register_driver(devname, &g_custom_hid_fops, 0666, priv);
  if (ret < 0)
    {
      uerr("ERROR: register_driver failed: %d\n", ret);
      goto errout;
    }

  /* Register the USB class driver with the USB device stack */

  ret = usbdev_register(&drvr->drvr);
  if (ret < 0)
    {
      uerr("ERROR: usbdev_register failed: %d\n", ret);
      unregister_driver(devname);
      goto errout;
    }

  if (handle != NULL)
    {
      *handle = (FAR void *)drvr;
    }

  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  nxsem_destroy(&priv->rxsem);
  kmm_free(alloc);
  return ret;
}

/****************************************************************************
 * Name: custom_hid_uninitialize
 ****************************************************************************/

void custom_hid_uninitialize(FAR void *handle)
{
  FAR struct custom_hid_driver_s *drvr =
    (FAR struct custom_hid_driver_s *)handle;
  FAR struct custom_hid_dev_s *priv;
  char devname[CUSTOM_HID_DEVNAME_SIZE];

  if (drvr == NULL)
    {
      return;
    }

  priv = drvr->dev;

  usbdev_unregister(&drvr->drvr);

  snprintf(devname, sizeof(devname), "/dev/hid%d", priv->minor);
  unregister_driver(devname);

  nxmutex_destroy(&priv->lock);
  nxsem_destroy(&priv->rxsem);

  kmm_free(priv);
}
