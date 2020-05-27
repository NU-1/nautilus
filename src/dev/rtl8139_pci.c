/*
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the
 * United States National  Science Foundation and the Department of Energy.
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xstack.sandia.gov/hobbes
 *
 * Copyright (c) 2017, Panitan Wongse-ammat
 * Copyright (c) 2017, Peter Dinda
 * Copyright (c) 2017, The V3VEE Project  <http://www.v3vee.org>
 *                     The Hobbes Project <http://xstack.sandia.gov/hobbes>
 * All rights reserved.
 *
 * Authors: Panitan Wongse-ammat <Panitan.W@u.northwesttern.edu>
 *          Marc Warrior <warrior@u.northwestern.edu>
 *          Galen Lansbury <galenlansbury2017@u.northwestern.edu>
 *          Peter Dinda <pdinda@northwestern.edu>
 *
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */

#include <nautilus/nautilus.h>
#include <nautilus/netdev.h>
#include <nautilus/cpu.h>
#include <dev/pci.h>
#include <nautilus/mm.h>              // malloc, free
#include <dev/rtl8139_pci.h>
#include <nautilus/irq.h>             // interrupt register
#include <nautilus/naut_string.h>     // memset, memcpy
#include <nautilus/dev.h>             // NK_DEV_REQ_*
#include <nautilus/timer.h>           // nk_sleep(ns);
#include <nautilus/cpu.h>             // udelay

// #ifndef NAUT_CONFIG_DEBUG_RTL8139_PCI
// #undef DEBUG_PRINT
// #define DEBUG_PRINT(fmt, args...)
// #endif

#define INFO(fmt, args...)     INFO_PRINT("rtl8139_pci: " fmt, ##args)
#define DEBUG(fmt, args...)    DEBUG_PRINT("rtl8139_pci: " fmt, ##args)
#define ERROR(fmt, args...)    ERROR_PRINT("rtl8139_pci: " fmt, ##args)

#define READ_MEM8(d, o)         (*((volatile uint8_t*)(((d)->mem_start)+(o))))
#define WRITE_MEM8(d, o, v)     ((*((volatile uint8_t*)(((d)->mem_start)+(o))))=(v))

#define READ_MEM16(d, o)         (*((volatile uint16_t*)(((d)->mem_start)+(o))))
#define WRITE_MEM16(d, o, v)     ((*((volatile uint16_t*)(((d)->mem_start)+(o))))=(v))

#define READ_MEM32(d, o)         (*((volatile uint32_t*)(((d)->mem_start)+(o))))
#define WRITE_MEM32(d, o, v)     ((*((volatile uint32_t*)(((d)->mem_start)+(o))))=(v))

#define READ_MEM64(d, o)       (*((volatile uint64_t*)((d)->mem_start + (o))))
#define WRITE_MEM64(d, o, v)     ((*((volatile uint64_t*)(((d)->mem_start)+(o))))=(v))

// PCI CONFIG SPACE ************************************
#define REALTEK_VENDOR_ID               0x10ec
#define RTL8139_DEVICE_ID              	0x8139
#define PCI_CMD_OFFSET         0x4    // Device Control - RW
#define PCI_STATUS_OFFSET      0x6    // Device Status - RO

// PCI command register
#define PCI_CMD_IO_ACCESS_EN   1       // io access enable
#define PCI_CMD_MEM_ACCESS_EN  (1<<1)  // memory access enable
#define PCI_CMD_LANRW_EN       (1<<2)  // enable mastering lan r/w
#define PCI_CMD_INT_DISABLE    (1<<10) // legacy interrupt disable when set

// PCI status register
#define PCI_STATUS_INT         (1<<3)

// RTL8139 CONSTANTS

// buffer size
#define RECEIVE_BUFFER_SIZE  	(8192 + 16) 

// Transmission Sizes
#define MIN_TU 48
#define MAX_TU 1792

//IRQ
#define RTL8139_IRQ 11

struct rtl8139_state {
	// a pointer to the base class
	struct nk_net_dev *netdev;
	// pci interrupt and interupt vector
	struct pci_dev *pci_dev;
	uint8_t   pci_intr;  // IRQ number on bus
	uint8_t   intr_vec;  // IRQ we will see

	// our device list
	struct list_head node;

	// Where registers are mapped into the I/O address space
	uint16_t  ioport_start;
	uint16_t  ioport_end;

	// Where registers are mapped into the physical memory address space
	uint64_t  mem_start;
	uint64_t  mem_end;
		
	char name[DEV_NAME_LEN];
	uint8_t mac_addr[6];

	//   struct e1000e_desc_ring *tx_ring;
	//   struct e1000e_desc_ring *rxd_ring;
	//   // a circular queue mapping between callback function and tx descriptor
	//   struct e1000e_map_ring *tx_map;
	//   // a circular queue mapping between callback funtion and rx descriptor
	uint32_t 	rec_buf_addr;
	//   // the size of receive buffers
	uint64_t 	rec_buf_size;
	//   // interrupt mark set
	//   uint32_t ims_reg;

	//Transmit Pair Counter for selection of rotating transmit register pairs
	uint64_t	TRCounter;

#if TIMING
  volatile iteration_t measure;
#endif
};


// list of discovered devices
static struct list_head dev_list;


/* Symbolic offsets to registers. */
enum RTL8139_registers {
	MAC0		= 0,	 /* Ethernet hardware address. */
	MAR0		= 8,	 /* Multicast filter. */
	TxStatus0	= 0x10,	 /* Transmit status (Four 32bit registers). */
	TxAddr0		= 0x20,	 /* Tx descriptors (also four 32bit). */
	RxBuf		= 0x30,
	ChipCmd		= 0x37,
	RxBufPtr	= 0x38,
	RxBufAddr	= 0x3A,
	IntrMask	= 0x3C,
	IntrStatus	= 0x3E,
	TxConfig	= 0x40,
	RxConfig	= 0x44,
	Timer		= 0x48,	 /* A general-purpose counter. */
	RxMissed	= 0x4C,  /* 24 bits valid, write clears. */
	Cfg9346		= 0x50,
	Config0		= 0x51,
	Config1		= 0x52,
	TimerInt	= 0x54,
	MediaStatus	= 0x58,
	Config3		= 0x59,
	Config4		= 0x5A,	 /* absent on RTL-8139A */
	HltClk		= 0x5B,
	MultiIntr	= 0x5C,
	TxSummary	= 0x60,
	BasicModeCtrl	= 0x62,
	BasicModeStatus	= 0x64,
	NWayAdvert	= 0x66,
	NWayLPAR	= 0x68,
	NWayExpansion	= 0x6A,
	/* Undocumented registers, but required for proper operation. */
	FIFOTMS		= 0x70,	 /* FIFO Control and test. */
	CSCR		= 0x74,	 /* Chip Status and Configuration Register. */
	PARA78		= 0x78,
	FlashReg	= 0xD4,	/* Communication with Flash ROM, four bytes. */
	PARA7c		= 0x7c,	 /* Magic transceiver parameter register. */
	Config5		= 0xD8,	 /* absent on RTL-8139A */
};

enum ClearBitMasks {
	MultiIntrClear	= 0xF000,
	ChipCmdClear	= 0xE2,
	Config1Clear	= (1<<7)|(1<<6)|(1<<3)|(1<<2)|(1<<1),
};

enum ChipCmdBits {
	CmdReset	= 0x10,
	CmdRxEnb	= 0x08,
	CmdTxEnb	= 0x04,
	RxBufEmpty	= 0x01,
};

/* Interrupt register bits, using my own meaningful names. */
enum IntrStatusBits {
	PCIErr		= 0x8000,
	PCSTimeout	= 0x4000,
	RxFIFOOver	= 0x40,
	RxUnderrun	= 0x20,
	RxOverflow	= 0x10,
	TxErr		= 0x08,
	TxOK		= 0x04,
	RxErr		= 0x02,
	RxOK		= 0x01,

	RxAckBits	= RxFIFOOver | RxOverflow | RxOK,
};

enum TxStatusBits {
	TxHostOwns	= 0x2000,
	TxUnderrun	= 0x4000,
	TxStatOK	= 0x8000,
	TxOutOfWindow	= 0x20000000,
	TxAborted	= 0x40000000,
	TxCarrierLost	= 0x80000000,
};
enum RxStatusBits {
	RxMulticast	= 0x8000,
	RxPhysical	= 0x4000,
	RxBroadcast	= 0x2000,
	RxBadSymbol	= 0x0020,
	RxRunt		= 0x0010,
	RxTooLong	= 0x0008,
	RxCRCErr	= 0x0004,
	RxBadAlign	= 0x0002,
	RxStatusOK	= 0x0001,
};

/* Bits in RxConfig. */
enum rx_mode_bits {
	AcceptErr	= 0x20,
	AcceptRunt	= 0x10,
	AcceptBroadcast	= 0x08,
	AcceptMulticast	= 0x04,
	AcceptMyPhys	= 0x02,
	AcceptAllPhys	= 0x01,
};

/* Bits in TxConfig. */
enum tx_config_bits {
        /* Interframe Gap Time. Only TxIFG96 doesn't violate IEEE 802.3 */
        TxIFGShift	= 24,
        TxIFG84		= (0 << TxIFGShift), /* 8.4us / 840ns (10 / 100Mbps) */
        TxIFG88		= (1 << TxIFGShift), /* 8.8us / 880ns (10 / 100Mbps) */
        TxIFG92		= (2 << TxIFGShift), /* 9.2us / 920ns (10 / 100Mbps) */
        TxIFG96		= (3 << TxIFGShift), /* 9.6us / 960ns (10 / 100Mbps) */

	TxLoopBack	= (1 << 18) | (1 << 17), /* enable loopback test mode */
	TxCRC		= (1 << 16),	/* DISABLE Tx pkt CRC append */
	TxClearAbt	= (1 << 0),	/* Clear abort (WO) */
	TxDMAShift	= 8, /* DMA burst value (0-7) is shifted X many bits */
	TxRetryShift	= 4, /* TXRR value (0-15) is shifted X many bits */

	TxVersionMask	= 0x7C800000, /* mask out version bits 30-26, 23 */
};

/* Bits in Config1 */
enum Config1Bits {
	Cfg1_PM_Enable	= 0x01,
	Cfg1_VPD_Enable	= 0x02,
	Cfg1_PIO	= 0x04,
	Cfg1_MMIO	= 0x08,
	LWAKE		= 0x10,		/* not on 8139, 8139A */
	Cfg1_Driver_Load = 0x20,
	Cfg1_LED0	= 0x40,
	Cfg1_LED1	= 0x80,
	SLEEP		= (1 << 1),	/* only on 8139, 8139A */
	PWRDN		= (1 << 0),	/* only on 8139, 8139A */
};

/* Bits in Config3 */
enum Config3Bits {
	Cfg3_FBtBEn   	= (1 << 0), /* 1	= Fast Back to Back */
	Cfg3_FuncRegEn	= (1 << 1), /* 1	= enable CardBus Function registers */
	Cfg3_CLKRUN_En	= (1 << 2), /* 1	= enable CLKRUN */
	Cfg3_CardB_En 	= (1 << 3), /* 1	= enable CardBus registers */
	Cfg3_LinkUp   	= (1 << 4), /* 1	= wake up on link up */
	Cfg3_Magic    	= (1 << 5), /* 1	= wake up on Magic Packet (tm) */
	Cfg3_PARM_En  	= (1 << 6), /* 0	= software can set twister parameters */
	Cfg3_GNTSel   	= (1 << 7), /* 1	= delay 1 clock from PCI GNT signal */
};

/* Bits in Config4 */
enum Config4Bits {
	LWPTN	= (1 << 2),	/* not on 8139, 8139A */
};

/* Bits in Config5 */
enum Config5Bits {
	Cfg5_PME_STS   	= (1 << 0), /* 1	= PCI reset resets PME_Status */
	Cfg5_LANWake   	= (1 << 1), /* 1	= enable LANWake signal */
	Cfg5_LDPS      	= (1 << 2), /* 0	= save power when link is down */
	Cfg5_FIFOAddrPtr= (1 << 3), /* Realtek internal SRAM testing */
	Cfg5_UWF        = (1 << 4), /* 1 = accept unicast wakeup frame */
	Cfg5_MWF        = (1 << 5), /* 1 = accept multicast wakeup frame */
	Cfg5_BWF        = (1 << 6), /* 1 = accept broadcast wakeup frame */
};

enum RxConfigBits {
	/* rx fifo threshold */
	RxCfgFIFOShift	= 13,
	RxCfgFIFONone	= (7 << RxCfgFIFOShift),

	/* Max DMA burst */
	RxCfgDMAShift	= 8,
	RxCfgDMAUnlimited = (7 << RxCfgDMAShift),

	/* rx ring buffer length */
	RxCfgRcv8K	= 0,
	RxCfgRcv16K	= (1 << 11),
	RxCfgRcv32K	= (1 << 12),
	RxCfgRcv64K	= (1 << 11) | (1 << 12),

	/* Disable packet wrap at end of Rx buffer. (not possible with 64k) */
	RxNoWrap	= (1 << 7),
};

/* Twister tuning parameters from RealTek.
   Completely undocumented, but required to tune bad links on some boards. */
enum CSCRBits {
	CSCR_LinkOKBit		= 0x0400,
	CSCR_LinkChangeBit	= 0x0800,
	CSCR_LinkStatusBits	= 0x0f000,
	CSCR_LinkDownOffCmd	= 0x003c0,
	CSCR_LinkDownCmd	= 0x0f3c0,
};

enum Cfg9346Bits {
	Cfg9346_Lock	= 0x00,
	Cfg9346_Unlock	= 0xC0,
};

typedef enum {
	CH_8139	= 0,
	CH_8139_K,
	CH_8139A,
	CH_8139A_G,
	CH_8139B,
	CH_8130,
	CH_8139C,
	CH_8100,
	CH_8100B_8139D,
	CH_8101,
} chip_t;

enum chip_flags {
	HasHltClk	= (1 << 0),
	HasLWake	= (1 << 1),
};

uint64_t e1000rtl_packet_size_to_buffer_size(uint64_t sz) 
{
  // Round up the number to the buffer size with power of two
  // In E1000, the packet buffer is the power of two.
  // citation: https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
  sz--;
  sz |= sz >> 1;
  sz |= sz >> 2;
  sz |= sz >> 4;
  sz |= sz >> 8;
  sz |= sz >> 16;
  sz |= sz >> 32;  
  sz++;
  // if the size is larger than the maximun buffer size, return the largest size.
  if(sz >= RECEIVE_BUFFER_SIZE) {
    return RECEIVE_BUFFER_SIZE;
  } else {
    return sz;
  }
}

// netdev-specific interface - set to zero if not available
// an interface either succeeds (returns zero) or fails (returns -1)
static int rtl8139_get_characteristics(void *state, struct nk_net_dev_characteristics *c){

	if(!state){
		ERROR("Device pointer is NULL\n");
	}

	if (!c){
		ERROR("Charateristics pointer is NULL\n"); 
	}

	
	struct rtl8139_state *rtl_state = (struct rtl8139_state*) state;
	memcpy(c->mac, (void *) rtl_state->mac_addr, ETHER_MAC_LEN);
	// minimum and the maximum transmission unit
	c->min_tu = MIN_TU; 
	c->max_tu = MAX_TU;
  	c->packet_size_to_buffer_size = e1000rtl_packet_size_to_buffer_size;

	return 0;
}

// send/receive are non-blocking always.  -1 on error, otherwise return 0
// callback can be null
static int rtl8139_post_receive(void *state, uint8_t *dest, uint64_t len, void (*callback)(nk_net_dev_status_t status, void *context), void *context){

	ERROR("unimplemented\n");
	return -1;
}

static int rtl8139_send(struct rtl8139_state* state, uint8_t *packetAddr, uint64_t packetLen){
	//POTENTIAL ISSUE
	//WE WRITE TO ALL 4 REGISTER PAIRS VERY QUICKLY AND CYCLE BACK
	//BEFORE DMA IS COMPLETE
	//what do we do?

	//check for packet size
	if (packetLen > MAX_TU){
		ERROR("RTL8139 Send Packet: Packet is too large\n");
	}

	//determine which transmit register pair to use
	uint8_t entry = state->TRCounter % 4;

	//Write address of packet to address register
	WRITE_MEM32(state, TxAddr0 + entry * 4, (uint32_t) packetAddr);

	//write status register
	uint32_t TxStatusTemp = packetLen & 0x1fff;
	WRITE_MEM32(state, TxStatus0 + entry * 4, TxStatusTemp);

	return 0;
}

static int rtl8139_post_send(void *state,
		uint8_t *src,
		uint64_t len,
		void (*callback)(nk_net_dev_status_t status, void *context),
		void *context){
	DEBUG("RTL8139 Post Send\n");
	//Do something with callback
	uint8_t result;

	//if callback successful
	result = rtl8139_send((struct rtl8139_state*) state, src, len);

	return 0;
}



static struct nk_net_dev_int ops = {
  .get_characteristics = rtl8139_get_characteristics,
  .post_receive        = rtl8139_post_receive,
  .post_send           = rtl8139_post_send,
};


static int rtl8139_irq_handler(excp_entry_t * excp, excp_vec_t vec, void *s) 
{
	DEBUG("rtl8139_irq_handler fn vector: 0x%x rip: 0x%p\n", vec, excp->rip);

	struct rtl8139_state* state = (struct rtl8139_state *)s;

	uint16_t isr = READ_MEM16(state, IntrStatus);
	DEBUG("Interrupt Status: %x\n", isr);

  //e1000 code
//   uint32_t ims = READ_MEM(state, E1000_IMS_OFFSET);
//   uint32_t mask_int = icr & ims;
//   DEBUG("ICR: 0x%08x IMS: 0x%08x mask_int: 0x%08x\n", icr, ims, mask_int);
//   DEBUG("ICR: 0x%08x icr should be zero.\n",
//         READ_MEM(state, E1000_ICR_OFFSET));
  
	void (*callback)(nk_net_dev_status_t, void*) = NULL;
	void *context = NULL;
	nk_net_dev_status_t status = NK_NET_DEV_STATUS_SUCCESS;
	
	if(isr & 0xC) {
		// transmit interrupt ERROR or OK
		DEBUG("Handle Transmit Interrupt\n");

		if (isr & 0x8){
			DEBUG("RTL8139 Transmit Handler Error\n");
		}

		if (isr & 0x4){
			DEBUG("RTL8139 Transmit Handler OK\n");
		}

		// e1000_unmap_callback(state->tx_map, (uint64_t **)&callback, (void **)&context);
		// // if there is an error while sending a packet, set the error status
		// if(TXD_STATUS(TXD_PREV_HEAD).ec || TXD_STATUS(TXD_PREV_HEAD).lc) {
		//   ERROR("transmit errors\n");
		//   status = NK_NET_DEV_STATUS_ERROR;
		// }

		// // update the head of the ring buffer
		// TXD_PREV_HEAD = TXD_INC(1, TXD_PREV_HEAD);
		// DEBUG("total packet transmitted = %d\n",
		//       READ_MEM(state, E1000_TPT_OFFSET));    
	}
  
	if (isr & 0x3){
		// receive interrupt ERROR or OK
		DEBUG("Handle Receive Interrupt\n");


		if(isr & 0x2){
			DEBUG("RTL8139 Receive Handler Error\n");
		}

		if(isr & 0x1){
			DEBUG("RTL8139 Receive Handler OK\n");
		}

		// e1000_unmap_callback(state->rx_map, (uint64_t **)&callback, (void **)&context);
		// // checking errors
		// if(RXD_ERRORS(RXD_PREV_HEAD)) {
		//   ERROR("receive an error packet\n");
		//   status = NK_NET_DEV_STATUS_ERROR;
		// }
		// DEBUG("RDLEN=0x%08x, RDH=0x%08x, RDT=0x%08x, RCTL=0x%08x\n",
		// 	    READ_MEM(state, RDLEN_OFFSET),
		// 	    READ_MEM(state, RDH_OFFSET),
		// 	    READ_MEM(state, RDT_OFFSET),
		// 	    READ_MEM(state, RCTL_OFFSET));
		
		// // in the irq, update only the head of the buffer
		// RXD_PREV_HEAD = RXD_INC(1, RXD_PREV_HEAD);    
		// DEBUG("total packet received = %d\n",
		//       READ_MEM(state, E1000_TPR_OFFSET));
	}

//   if(callback) {
//     DEBUG("invoke callback function callback: 0x%p\n", callback);
//     callback(status, context);
//   }
	//reset ISR to 0 for QEMU
	// isr &= 0xfff0; // clear T/R ERR/OK bits
	// WRITE_MEM16(state, IntrStatus, isr);

	DEBUG("end irq\n\n\n");
	// must have this line at the end of the handler
	IRQ_HANDLER_END();
	return 0;
}



int rtl8139_pci_init(struct naut_info * naut)
{
  struct pci_info *pci = naut->sys.pci;
  struct list_head *curbus, *curdev;
  uint16_t num = 0;

  INFO("init\n");

  // if (!pci) {
  //   ERROR("No PCI info\n");
  //   return -1;
  // }
 
  // INIT_LIST_HEAD(&dev_list);
  
  DEBUG("Finding rtl8139 devices\n");



	struct rtl8139_state *s = malloc(sizeof(*s));
	if (!s){
		ERROR("Can't alloc\n");
		return -1;
	}

	memset(s, 0, sizeof(*s));

	nk_net_dev_register("TheFakeRTL8139", 0, &ops, (void *)s);


  list_for_each(curbus,&(pci->bus_list)) {
    struct pci_bus *bus = list_entry(curbus,struct pci_bus,bus_node);

    DEBUG("Searching PCI bus %u for RTL8139 devices\n", bus->num);

    list_for_each(curdev, &(bus->dev_list)) {
      struct pci_dev *pdev = list_entry(curdev,struct pci_dev,dev_node);
      struct pci_cfg_space *cfg = &pdev->cfg;

      DEBUG("Device %u is a 0x%x:0x%x\n", pdev->num, cfg->vendor_id, cfg->device_id);
      // intel vendor id and e1000e device id
      if (cfg->vendor_id==REALTEK_VENDOR_ID && cfg->device_id==RTL8139_DEVICE_ID) {
		int foundio=0, foundmem=0;
        DEBUG("Found RTL8139 Device :)\n");
		struct rtl8139_state *state = malloc(sizeof(struct rtl8139_state));
	
        if (!state) {
          ERROR("Cannot allocate device\n");
          return -1;
        }

        memset(state,0,sizeof(*state));
	
	// We will only support MSI for now

        // find out the bar for e1000e
        for (int i=0;i<6;i++) {
          uint32_t bar = pci_cfg_readl(bus->num, pdev->num, 0, 0x10 + i*4);
          uint32_t size;
          DEBUG("bar %d: 0x%0x\n",i, bar);
          // go through until the last one, and get out of the loop
          if (bar==0) {
            break;
          }
          // get the last bit and if it is zero, it is the memory
          // " -------------------------"  one, it is the io
          if (!(bar & 0x1)) {
            uint8_t mem_bar_type = (bar & 0x6) >> 1;
            if (mem_bar_type != 0) { // 64 bit address that we do not handle it
              ERROR("Cannot handle memory bar type 0x%x\n", mem_bar_type);
              return -1;
            }
          }

          // determine size
          // write all 1s, get back the size mask
          pci_cfg_writel(bus->num, pdev->num, 0, 0x10 + i*4, 0xffffffff);
          // size mask comes back + info bits
          // write all ones and read back. if we get 00 (negative size), size = 4.
          size = pci_cfg_readl(bus->num, pdev->num, 0, 0x10 + i*4);

          // mask all but size mask
          if (bar & 0x1) { // I/O
            size &= 0xfffffffc;
          } else { // memory
            size &= 0xfffffff0;
          }
          // two complement, get back the positive size
          size = ~size;
          size++;

          // now we have to put back the original bar
          pci_cfg_writel(bus->num, pdev->num, 0, 0x10 + i*4, bar);

          if (!size) { // size = 0 -> non-existent bar, skip to next one
            continue;
          }

          uint32_t start = 0;
          if (bar & 0x1) { // IO
            start = state->ioport_start = bar & 0xffffffc0;
            state->ioport_end = state->ioport_start + size;
	    	foundio=1;
          } else { // mem
            start = state->mem_start = bar & 0xfffffff0;
            state->mem_end = state->mem_start + size;
	    	foundmem=1;
		  }

          

          DEBUG("bar %d is %s address=0x%x size=0x%x\n", i,
                bar & 0x1 ? "io port":"memory", start, size);
        }

        INFO("Adding rtl8139 device: bus=%u dev=%u func=%u: ioport_start=%p ioport_end=%p mem_start=%p mem_end=%p\n",
             bus->num, pdev->num, 0,
             state->ioport_start, state->ioport_end,
             state->mem_start, state->mem_end);

		uint16_t pci_cmd = PCI_CMD_MEM_ACCESS_EN | PCI_CMD_IO_ACCESS_EN | PCI_CMD_LANRW_EN; // | E1000E_PCI_CMD_INT_DISABLE;
        DEBUG("init fn: new pci cmd: 0x%04x\n", pci_cmd);
        pci_cfg_writew(bus->num,pdev->num,0, PCI_CMD_OFFSET, pci_cmd);
        DEBUG("init fn: pci_cmd 0x%04x expects 0x%04x\n",
              pci_cfg_readw(bus->num,pdev->num, 0, PCI_CMD_OFFSET),
              pci_cmd);
        DEBUG("init fn: pci status 0x%04x\n",
              pci_cfg_readw(bus->num,pdev->num, 0, PCI_STATUS_OFFSET));

        list_add(&dev_list, &state->node);
        sprintf(state->name, "rtl8139-%d", num);
        num++;
        
        state->pci_dev = pdev;
	
        state->netdev = nk_net_dev_register(state->name, 0, &ops, (void *)state);
	
        if (!state->netdev) {
          ERROR("init fn: Cannot register the e1000e device \"%s\"", state->name);
          return -1;
        }

		if (!foundmem) {
			ERROR("init fn: ignoring device %s as it has no memory access method\n",state->name);
			continue;
		}



		// disable interrupts? (both 3c and 5c)
		// disable interrupts
		uint32_t mac_address0 = READ_MEM32(state, MAC0);
		DEBUG("higher address of our card thingy; %x\n", mac_address0);

		uint32_t mac_address1 = READ_MEM32(state, MAC0 + 4);
		DEBUG("lower address of our card thingy; %x\n", mac_address1);

		uint64_t mac_address = (((uint64_t)mac_address0)+(((uint64_t)mac_address1)<<32));
		DEBUG("mac address of our card thingy; 0x%lX\n", mac_address);

		DEBUG("init fn: device reset\n");
		WRITE_MEM8(state, Config1, 0);
		udelay(10);
		WRITE_MEM8(state, ChipCmd, CmdReset);		
		while((READ_MEM8(state, ChipCmd) & 0x10) != 0);

		// set up receive buffer; get its address and save it
  		state->rec_buf_addr = (uint32_t) malloc(state->rec_buf_size);
		state->rec_buf_size = RECEIVE_BUFFER_SIZE;

		// write write the receive buffer address
		WRITE_MEM32(state, RxBuf, state->rec_buf_addr);
		
		// set up the interrupt mask
		DEBUG("init fn: interrupts disables after reset\n");
		WRITE_MEM16(state, IntrMask, 0x0005);


		// set up whatever packet types we want to receive
		// this is a "run mode"
		// we choose: "accept physical match," which are packets sent to our own mac address
		// we also tell it the buffer size
		// we do this in the rcr register, by unsetting bits 11 and 12 and 7 and setting bit 1
		uint32_t rcr_val = READ_MEM32(state, RxConfig);
		// setting bit 1 to accept physical match packets to mac address
		rcr_val |= 0x2;
		// unsetting bits 11 and 12 to tell it that the page size is 8192 + 16
		rcr_val &= ~(0x3 << 11);
		// unsetting bit 7 to tell it to wrap around the ring buffere (not assume that there is extra space after)
		rcr_val &= ~(0x1 << 7);
		WRITE_MEM32(state, RxConfig, rcr_val);

		// enable receive and transmit
		WRITE_MEM8(state, ChipCmd, 0xc);
	
		// done initializing device?  woop-dee-doo

		//FINDING THE IRQ AND SETTING UP HANDLER

			//enable all interrupts on the device
			WRITE_MEM16(state, IntrMask, ~0);
			
			//enable interrupts to flow off the device
			uint16_t old_cmd = pci_cfg_readw(bus->num,pdev->num,0,0x4);
			DEBUG("Old PCI CMD: 0x%04x\n",old_cmd);

			old_cmd |= 0x7;  // make sure bus master is enabled
			old_cmd &= ~0x40;

			DEBUG("New PCI CMD: 0x%04x\n",old_cmd);

			pci_cfg_writew(bus->num,pdev->num,0,0x4,old_cmd);

			// PCI Interrupt (A..D)
			state->pci_intr = cfg->dev_cfg.intr_pin;

			// GRUESOME HACK
			state->intr_vec = RTL8139_IRQ;

			// uint64_t num_vecs = pdev->msi.num_vectors_needed;
			// uint64_t base_vec = 0;

			// if (idt_find_and_reserve_range(num_vecs,1,&base_vec)) {
			// 	ERROR("Cannot find %d vectors for %s - skipping\n",num_vecs,state->name);
			// 	continue;
			// }

			// DEBUG("%s vectors are %d..%d\n",state->name,base_vec,base_vec+num_vecs-1);

			// if (pci_dev_enable_msi(pdev, base_vec, num_vecs, 0)) {
			// 	ERROR("Failed to enable MSI for device %s - skipping\n", state->name);
			// 	continue;
			// }

			// int i;
			// int failed=0;

			// for (i=base_vec;i<(base_vec+num_vecs);i++) {
			// 	if (register_int_handler(i, rtl8139_irq_handler, state)) {
			// 		ERROR("Failed to register handler for vector %d on device %s - skipping\n",i,state->name);
			// 		failed=1;
			// 		break;
			// 	}
			// }

			// register the interrupt handler
			// if (!failed) { 
			// 	for (i=base_vec; i<(base_vec+num_vecs);i++) {
			// 		if (pci_dev_unmask_msi(pdev, i)) {
			// 			ERROR("Failed to unmask interrupt %d for device %s\n",i,state->name);
			// 			failed = 1;
			// 			break;
			// 		}
			// 	}
			// }

			if (register_irq_handler(state->intr_vec, rtl8139_irq_handler, state)){
				ERROR("RTL8139 IRQ Handler failed registration\n");
			}
			
			for (int i = 0; i < 256; i++){
				nk_unmask_irq(i);		
			}
			DEBUG("Finished initing rtl8139\n");
			
			//Create Ethernet Packet for testing
			uint8_t p[64];
			memset(p,0,64);

			p[0]=p[1]=p[2]=p[3]=p[4]=p[5]=0xff;  // destination is broadcast address ff:ff:ff:ff:ff:ff
			p[6]=1; p[7]=2; p[8]=3; p[9]=4; p[10]=5; p[11]=6; // src is 01:02:03:04:05:06
			p[12]= 8; p[13]=0;  // type is IP (see  https://en.wikipedia.org/wiki/EtherType)
			strcpy(&p[14],"Hello World");

			//Write address of packet to address register
			uint32_t t_p = (uint64_t) (void *) p & 0x00000000ffffffff;
			WRITE_MEM32(state, TxAddr0, t_p);

			//write status register
			uint32_t TxStatusTemp = 0;//READ_MEM32(state, TxStatus0);
			TxStatusTemp = 64;
			WRITE_MEM32(state, TxStatus0, TxStatusTemp);
	  }
	}
  }


  return 0;

}

int rtl8139_pci_deinit() {
  INFO("deinited and leaking\n");
  return 0;
}