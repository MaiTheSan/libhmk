
#define BOARD_USB_HS
#define CFG_TUD_MCU OPT_MCU_AT32F4032_405

#define TUP_DCD_ENDPOINT_MAX 8

#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_DEBUG 0
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))


// Driver configuration
// 2 HID interfaces (keyboard and generic), 1 vendor interface
#define CFG_TUD_HID 2
#define CFG_TUD_VENDOR 1

#define CFG_TUD_ENABLED 1
#define CFG_TUD_ENDPOINT0_SIZE 64
// HID buffer size. Must be strictly large than all report sizes
#define CFG_TUD_HID_EP_BUFSIZE 64
#define CFG_TUD_VENDOR_EPSIZE 64

#if defined(BOARD_USB_FS)
#define BOARD_TUD_RHPORT 0
#elif defined(BOARD_USB_HS)
#define BOARD_TUD_RHPORT 1
#else
#error "USB peripheral not defined"
#endif

