/*
 * flash_parms.h
 * Robust Flash persistence for user settings (STM32F1xx).
 */

#ifndef INC_FLASH_PARMS_H_
#define INC_FLASH_PARMS_H_

#include <stdint.h>

/* =============================== Flash layout =============================== */
/* Adjust these if your linker / memory map changes. Defaults are for STM32F103
 * 128 KiB devices with 1 KiB page size. The user area spans two pages.
 */
#ifndef FLASH_BASE
#define FLASH_BASE           0x08000000UL
#endif

#ifndef FLASH_PAGE_BYTES
#ifdef FLASH_PAGE_SIZE
#define FLASH_PAGE_BYTES     ((uint32_t)FLASH_PAGE_SIZE)
#else
#define FLASH_PAGE_BYTES     1024UL              /* bytes (STM32F1: 1 KiB) */
#endif
#endif

/* Helpers to convert between page <-> address */
#define FLASH_PAGE_TO_ADDR(page)   ( (uint32_t)(FLASH_BASE + ((uint32_t)(page) * FLASH_PAGE_BYTES)) )
#define ADDR_TO_FLASH_PAGE(addr)   ( (uint32_t)(((uint32_t)(addr) - FLASH_BASE) / FLASH_PAGE_BYTES) )

/* Pick two pages near the end of Flash for parameters (customize as needed). */
#ifndef FLASH_USER_START_ADDR
#define FLASH_USER_START_ADDR      FLASH_PAGE_TO_ADDR(61U)    /* inclusive */
#endif
#ifndef FLASH_USER_END_ADDR
#define FLASH_USER_END_ADDR        FLASH_PAGE_TO_ADDR(63U)    /* exclusive; spans pages 61 & 62 */
#endif
/* Compatibility aliases used by page helpers below */
#define PAGE_TO_ADDR(p)            FLASH_PAGE_TO_ADDR(p)
#define ADDR_TO_PAGE(a)            ADDR_TO_FLASH_PAGE(a)

/* =============================== Error codes ================================ */
typedef enum
{
    FLASH_SUCCESS = 0,
    FLASH_PARAM_ERROR,
    FLASH_ADDR_ERROR,
    FLASH_WRITE_WORD_ERROR,
    FLASH_WRITE_HALF_WORD_ERROR,
    FLASH_WRITE_BYTE_ERROR,
    FLASH_READ_ERROR,
    FLASH_ERASE_ERROR,
} FLASH_ERROR_CODE_E;

/* ============================ Persisted structure =========================== */
/* This struct is what user_settings.c reads/writes. Keep size multiple of 4.  */
#define PARAMS_SAVED_FLAG  0xAA

typedef struct
{
    /* User selections (persisted) */
    uint8_t  saved_freeze_mode;           /* 0..4 */
    uint8_t  saved_led_ring_brightness;   /* 0..100 */
    uint8_t  saved_rgb_led_default;       /* 0=OFF, 1=WHITE */
    uint8_t  param_flag;                  /* signature = PARAMS_SAVED_FLAG */

    /* Housekeeping / telemetry */
    uint32_t saved_motor_hours;           /* rounded hours (exact value maintained elsewhere) */
    uint8_t  saved_live_fault_condition;  /* latest live fault snapshot */
    uint8_t  _rsv1;
    uint8_t  _rsv2;
    uint8_t  _rsv3;

    /* Room for future use while keeping alignment */
    uint32_t _padding0;
} params_settings_t;

/* Statically allocated instance lives in user_settings.c */
extern params_settings_t usr_params_settings;

/* ============================ High-level interface ========================== */
/* Load/save full struct from/to Flash. Return 1 on success, 0 on fail. */
int  flash_params_load (params_settings_t *out);
int  flash_params_save (const params_settings_t *in);

/* ============================ Low-level primitives ========================== */
uint32_t          flash_read (uint32_t address, uint8_t *pdata, uint32_t size);
FLASH_ERROR_CODE_E flash_write(uint32_t address, const uint8_t *pdata, uint32_t size);
FLASH_ERROR_CODE_E flash_erase(uint32_t start_addr, uint32_t end_addr);

/* Page helpers retained for compatibility with older code */
uint32_t          flash_read_page (uint8_t page_no, uint32_t offset, uint8_t *pdata, uint32_t size);
uint32_t          flash_write_page(uint8_t page_no, uint32_t offset, const uint8_t *pdata, uint32_t size);
FLASH_ERROR_CODE_E flash_erase_page(uint32_t start_page, uint16_t page_cnt);

/* Utility */
void print_flash_info(void);

/* ----------------------- Legacy API compatibility --------------------------- */
/* If any legacy code still calls these, keep them mapped to new names. */
static inline void params_load_settings(params_settings_t *p)  { (void)flash_params_load(p); }
static inline void params_save_settings(params_settings_t *p)  { (void)flash_params_save(p); }

#endif /* INC_FLASH_PARMS_H_ */
