/*
 * flash_parms.c
 * Robust Flash persistence for user settings (STM32F1xx).
 */

#include "flash_parms.h"
#include "main.h"

#include <string.h>
#include <stdio.h>

/* ============================ High-level parameter API ===================== */

int flash_params_load(params_settings_t *out)
{
    uint32_t n;

    if (!out) return 0;

    memset(out, 0, sizeof(*out));
    n = flash_read(FLASH_USER_START_ADDR, (uint8_t *)out, sizeof(*out));
    if (n != sizeof(*out))
    {
        /* Not programmed or read error → present as blank */
        memset(out, 0, sizeof(*out));
        return 0;
    }

    if (out->param_flag != PARAMS_SAVED_FLAG)
    {
        /* Invalid signature → treat as blank/defaults */
        memset(out, 0, sizeof(*out));
        return 0;
    }

    return 1;
}

int flash_params_save(const params_settings_t *in)
{
    params_settings_t tmp;

    if (!in) return 0;

    /* Copy and enforce signature */
    memcpy(&tmp, in, sizeof(tmp));
    tmp.param_flag = PARAMS_SAVED_FLAG;

    if (flash_erase(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR) != FLASH_SUCCESS)
        return 0;

    if (flash_write(FLASH_USER_START_ADDR, (const uint8_t *)&tmp, sizeof(tmp)) != FLASH_SUCCESS)
        return 0;

    return 1;
}

/* =============================== Low-level Flash I/O ======================= */

uint32_t flash_read(uint32_t address, uint8_t *pdata, uint32_t size)
{
    const uint8_t *src;
    uint32_t i;
    uint32_t end_addr;

    if (!pdata || size == 0U)
        return 0U;

    end_addr = address + size;
    if (address < FLASH_USER_START_ADDR || end_addr > FLASH_USER_END_ADDR)
        return 0U;

    /* Flash is memory-mapped – regular reads are fine. */
    src = (const uint8_t *)address;
    for (i = 0U; i < size; i++)
    {
        pdata[i] = src[i];
    }

    return size;
}

/* STM32F1 flash programming granularity is 16-bit halfword. */
FLASH_ERROR_CODE_E flash_write(uint32_t address, const uint8_t *pdata, uint32_t size)
{
    FLASH_ERROR_CODE_E rc;
    uint32_t i;

    if (!pdata || size == 0U)
        return FLASH_PARAM_ERROR;

    if (address < FLASH_USER_START_ADDR || (address + size) > FLASH_USER_END_ADDR)
        return FLASH_ADDR_ERROR;

    /* Halfword alignment required */
    if ((address & 0x1U) != 0U)
        return FLASH_ADDR_ERROR;

    HAL_FLASH_Unlock();
    rc = FLASH_SUCCESS;

    /* Program in halfwords; pad final odd byte with 0xFF. */
    i = 0U;
    while (i < size)
    {
        uint16_t hw = pdata[i];
        if ((i + 1U) < size)
        {
            hw |= ((uint16_t)pdata[i + 1U]) << 8;
        }
        else
        {
            hw |= 0xFF00U; /* pad */
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, hw) != HAL_OK)
        {
            rc = FLASH_WRITE_HALF_WORD_ERROR;
            break;
        }

        i += 2U;
    }

    HAL_FLASH_Lock();
    return rc;
}

FLASH_ERROR_CODE_E flash_erase(uint32_t start_addr, uint32_t end_addr)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_err;
    uint32_t first_page_addr;
    uint32_t last_addr;
    uint32_t last_page_addr;
    uint32_t nb_pages;
    FLASH_ERROR_CODE_E rc;

    if (start_addr < FLASH_USER_START_ADDR || end_addr > FLASH_USER_END_ADDR || start_addr >= end_addr)
        return FLASH_ADDR_ERROR;

    /* Align to page boundaries (inclusive start, exclusive end) */
    first_page_addr = (start_addr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    last_addr       = end_addr - 1U;
    last_page_addr  = (last_addr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    nb_pages        = ((last_page_addr - first_page_addr) / FLASH_PAGE_SIZE) + 1U;

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = first_page_addr;
    erase.NbPages     = nb_pages;

    HAL_FLASH_Unlock();
    page_err = 0U;
    rc = (HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK) ? FLASH_SUCCESS : FLASH_ERASE_ERROR;
    HAL_FLASH_Lock();

    return rc;
}

/* =========================== Page-based compatibility API ================== */

uint32_t flash_read_page(uint8_t page_no, uint32_t offset, uint8_t *pdata, uint32_t size)
{
    uint32_t addr = FLASH_PAGE_TO_ADDR(page_no) + offset;
    return flash_read(addr, pdata, size);
}

uint32_t flash_write_page(uint8_t page_no, uint32_t offset, const uint8_t *pdata, uint32_t size)
{
    uint32_t addr = FLASH_PAGE_TO_ADDR(page_no) + offset;
    return (flash_write(addr, pdata, size) == FLASH_SUCCESS) ? size : 0U;
}

FLASH_ERROR_CODE_E flash_erase_page(uint32_t start_page, uint16_t page_cnt)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_err;
    FLASH_ERROR_CODE_E rc;

    if (page_cnt == 0U) return FLASH_PARAM_ERROR;

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_PAGE_TO_ADDR(start_page);
    erase.NbPages     = page_cnt;

    HAL_FLASH_Unlock();
    page_err = 0U;
    rc = (HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK) ? FLASH_SUCCESS : FLASH_ERASE_ERROR;
    HAL_FLASH_Lock();

    return rc;
}

/* ================================= Utilities =============================== */

void print_flash_info(void)
{
    printf("Flash info:\r\n");
    printf("  User start : 0x%08lX\r\n", (unsigned long)FLASH_USER_START_ADDR);
    printf("  User end   : 0x%08lX (exclusive)\r\n", (unsigned long)FLASH_USER_END_ADDR);
    printf("  Page size  : %lu bytes\r\n", (unsigned long)FLASH_PAGE_SIZE);
    printf("  Struct size: %lu bytes\r\n", (unsigned long)sizeof(params_settings_t));
}
