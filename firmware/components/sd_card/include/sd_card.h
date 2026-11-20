#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#define SDCARD_MOUNT_POINT "/sdcard"

/*
 * Mounts the card (4-bit SDMMC) on SDCARD_MOUNT_POINT.
 * Returns false if the FAT filesystem cannot be mounted (no card, or not
 * FAT-formatted: the card is never formatted automatically).
 */
bool sdcard_mount(uint8_t sd_pin_clk, uint8_t sd_pin_cmd, uint8_t sd_pin_d0, uint8_t sd_pin_d1, uint8_t sd_pin_d2, uint8_t sd_pin_d3, uint8_t sd_pin_cd);

/*
 * Creates the next free file EEG_0001.BIN ... EEG_9999.BIN and opens it for
 * binary writing. The chosen path is copied to `path` (if not NULL).
 * Returns NULL if the card is not mounted or no name is free.
 */
FILE* sdcard_new_file(char *path, size_t path_len);

/* Flushes the stdio buffer and the FAT cache of `f` to the card. */
void sdcard_sync(FILE *f);

void sdcard_unmount(void);
