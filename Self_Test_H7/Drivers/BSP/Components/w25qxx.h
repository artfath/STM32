/*
 * w25qxx.h
 *
 *  Created on: May 16, 2025
 *      Author: Muhammad Fatahila
 */

#ifndef W25QXX_H_
#define W25QXX_H_

/* W25Q32 */
#define W25X_FLASH_SIZE                  0x0400000 /* 32 MBits => 4MBytes */
#define W25X_SECTOR_COUNT                0x40      /* 64 sectors of 64KBytes */
#define W25X_SECTOR_SIZE                 0x10000   /* 64 sectors of 64KBytes */
#define W25X_SUBSECTOR_COUNT             0x0400    /* 1024 subsectors of 4kBytes */
#define W25X_SUBSECTOR_SIZE              0x1000    /* 1024 subsectors of 4kBytes */
#define W25X_PAGE_SIZE                   0x100     /* 16384 pages of 256 bytes */

#define W25Q80  0XEF13
#define W25Q16  0XEF14
#define W25Q32  0XEF15
#define W25Q64  0XEF16
#define W25Q128 0XEF17

#define W25X_DUMMY_CYCLES_READ           4
#define W25X_DUMMY_CYCLES_READ_QUAD      10

#define W25X_BULK_ERASE_MAX_TIME         250000
#define W25X_SECTOR_ERASE_MAX_TIME       3000
#define W25X_SUBSECTOR_ERASE_MAX_TIME    800
#define W25X_TIMEOUT_VALUE 1000

/**
  * @brief  W25QXX Commands
  */
/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

#define ENTER_QPI_MODE_CMD                   0x38
#define EXIT_QPI_MODE_CMD                    0xFF

/* Identification Operations */
#define READ_ID_CMD                          0x90
#define DUAL_READ_ID_CMD                     0x92
#define QUAD_READ_ID_CMD                     0x94
#define READ_JEDEC_ID_CMD                    0x9F

/* Read Operations */
#define FLASH_READ                            0x03
#define FLASH_FAST_READ                        0x0B
#define FLASH_DUAL_OUT_FAST_READ               0x3B
#define FLASH_DUAL_INOUT_FAST_READ             0xBB
#define FLASH_QUAD_OUT_FAST_READ               0x6B
#define FLASH_QUAD_INOUT_FAST_READ             0xEB

/* Write Operations */
#define FLASH_WRITE_ENABLE                    0x06
#define FLASH_WRITE_DISABLE                   0x04

/* Register Operations */
#define FLASH_READ_STATUS_REG1                 0x05
#define FLASH_READ_STATUS_REG2                  0x35
#define FLASH_READ_STATUS_REG3                  0x15

#define FLASH_WRITE_STATUS_REG1                 0x01
#define FLASH_WRITE_STATUS_REG2                 0x31
#define FLASH_WRITE_STATUS_REG3                 0x11


/* Program Operations */
#define FLASH_PAGE_PROG                      0x02
#define FLASH_QUAD_INPUT_PAGE_PROG            0x32


/* Erase Operations */
#define FLASH_SECTOR_ERASE                     0x20
#define FLASH_CHIP_ERASE                       0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* Flag Status Register */
#define FLASH_FSR_BUSY                    0x01    /*!< busy */
#define FLASH_FSR_WREN                    0x02    /*!< write enable */
#define FLASH_FSR_QE                      0x02    /*!< quad enable */

#define FLASH_OK            ((uint8_t)0x00)
#define FLASH_ERROR         ((uint8_t)0x01)
#define FLASH_BUSY          ((uint8_t)0x02)
#define FLASH_TIMEOUT		((uint8_t)0x03)
#endif /* BSP_COMPONENTS_W25QXX_H_ */
