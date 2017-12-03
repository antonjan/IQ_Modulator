/*
 * This file is part of RPIO.
 *
 * Copyright
 *
 *     Copyright (C) 2013 Chris Hager <chris@linuxuser.at>
 *
 * License
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU Lesser General Public License as published
 *     by the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU Lesser General Public License for more details at
 *     <http://www.gnu.org/licenses/lgpl-3.0-standalone.html>
 *
 * Documentation
 *
 *     http://pythonhosted.org/RPIO
 *
 * Based on the excellent ServoBlaster by Richard Hirst.
 *
 * Documentation
 * =============
 *
 * A server is controlled via the pulse-width within a fixed period (the
 * period is defined by the servo-maker; look it up in the datasheet).
 *
 *      |<--- Period Width (20ms) ------>|
 *
 *      +--------+                       +--------+
 *      |        |                       |        |
 * -----+        +-----------------------+        +------
 *
 *   -->|        |<-- Pulse Width (1..2ms)
 *
 *
 * This documentation is work in progress. Look here for more information:
 * - https://github.com/metachris/raspberrypi-pwm
 * - https://github.com/richardghirst/PiBits/blob/master/ServoBlaster
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define out32(a,v) (*(volatile uint32_t *)(a) = (v))
#define in32(a) (*(volatile uint32_t *)(a))

#if 0
static uint8_t gpio_list[] = {
    4,    // P1-7
    17,    // P1-11
    18,    // P1-12
    21,    // P1-13
    22,    // P1-15
    23,    // P1-16
    24,    // P1-18
    25,    // P1-22
};
#endif
#define GPIO_FREQ_NUM 4
#define GPIO_POS_NUM 17
#define GPIO_NEG_NUM 18

#define DEVFILE_SEND "/dev/psk31.data"
#define DEVFILE_CTRL "/dev/psk31.ctrl"
#define DEVFILE_STAT "/dev/psk31.stat"

enum {
	SYM_L,
	SYM_H,
	SYM_LH,
	SYM_HL,

	SYM_COUNT
};

// PULSE_WIDTH_INCR_US is the pulse width increment granularity, again in microseconds.
// Setting it too low will likely cause problems as the DMA controller will use too much
//memory bandwidth. 10us is a good value, though you might be ok setting it as low as 2us.
#define PULSE_WIDTH_INCR_US  10

#define BS_US                32000
#define BS_SAMPLES           (BS_US / PULSE_WIDTH_INCR_US)
#define TS_SHIFT             4
#define TS_COUNT             (1 << TS_SHIFT)
#define TS_US                BS_US

// Various
#define NUM_SAMPLES          (BS_SAMPLES * SYM_COUNT * TS_COUNT)
#define NUM_CBS              (NUM_SAMPLES * 3)

#define PAGE_SIZE            4096
#define PAGE_SHIFT           12
#define NUM_PAGES_CBS        ((NUM_CBS * 32 + PAGE_SIZE - 1) >> PAGE_SHIFT)
#define NUM_PAGES_SAMPLES    ((2 * 4 + PAGE_SIZE - 1) >> PAGE_SHIFT)
#define NUM_PAGES            (NUM_PAGES_CBS + NUM_PAGES_SAMPLES)



// Memory Addresses
#define DMA_BASE        0x20007000
#define DMA_LEN         0x24
#define PWM_BASE        0x2020C000
#define PWM_LEN         0x28
#define CLK_BASE        0x20101000
#define CLK_LEN         0xA8
#define GPIO_BASE       0x20200000
#define GPIO_LEN        0x100
#define PCM_BASE        0x20203000
#define PCM_LEN         0x24

#define DMA_NO_WIDE_BURSTS  (1<<26)
#define DMA_WAIT_RESP   (1<<3)
#define DMA_D_DREQ      (1<<6)
#define DMA_PER_MAP(x)  ((x)<<16)
#define DMA_END         (1<<1)
#define DMA_RESET       (1<<31)
#define DMA_INT         (1<<2)

#define DMA_CS          (0x00/4)
#define DMA_CONBLK_AD   (0x04/4)
#define DMA_DEBUG       (0x20/4)

// GPIO Memory Addresses
#define GPIO_FSEL0      (0x00/4)
#define GPIO_SET0       (0x1c/4)
#define GPIO_CLR0       (0x28/4)
#define GPIO_LEV0       (0x34/4)
#define GPIO_PULLEN     (0x94/4)
#define GPIO_PULLCLK    (0x98/4)

// GPIO Modes (IN=0, OUT=1)
#define GPIO_MODE_IN    0
#define GPIO_MODE_OUT   1
#define GPIO_MODE_ALT0  4
#define GPIO_MODE_ALT1  5
#define GPIO_MODE_ALT2  6
#define GPIO_MODE_ALT3  7
#define GPIO_MODE_ALT4  3
#define GPIO_MODE_ALT5  2

// PWM Memory Addresses
#define PWM_CTL         (0x00/4)
#define PWM_DMAC        (0x08/4)
#define PWM_RNG1        (0x10/4)
#define PWM_FIFO        (0x18/4)

#define PWMCLK_CNTL     40
#define PWMCLK_DIV      41

#define PWMCTL_MODE1    (1<<1)
#define PWMCTL_PWEN1    (1<<0)
#define PWMCTL_CLRF     (1<<6)
#define PWMCTL_USEF1    (1<<5)

#define PWMDMAC_ENAB    (1<<31)
#define PWMDMAC_THRSHLD ((15<<8) | (15<<0))

#define PCM_CS_A        (0x00/4)
#define PCM_FIFO_A      (0x04/4)
#define PCM_MODE_A      (0x08/4)
#define PCM_RXC_A       (0x0c/4)
#define PCM_TXC_A       (0x10/4)
#define PCM_DREQ_A      (0x14/4)
#define PCM_INTEN_A     (0x18/4)
#define PCM_INT_STC_A   (0x1c/4)
#define PCM_GRAY        (0x20/4)

#define PCMCLK_CNTL     38
#define PCMCLK_DIV      39

#define DELAY_VIA_PWM   0
#define DELAY_VIA_PCM   1

/* General Purpose Clock */
#define CM_GP0CTL       (0x70 / 4)
#define CM_GP1CTL       (0x78 / 4)
#define CM_GP2CTL       (0x80 / 4)
#define CM_GP0DIV       (0x74 / 4)
#define CM_GP1DIV       (0x7c / 4)
#define CM_GP2DIV       (0x84 / 4)

typedef struct {
	uint32_t info;
	uint32_t src;
	uint32_t dst;
	uint32_t length;
	uint32_t stride;
	uint32_t next;
	uint32_t pad_1;
	uint32_t pad_2;
} dma_cb_t;

struct ctl {
	union {
		dma_cb_t cb[NUM_CBS];
		char cb_pages[NUM_PAGES_CBS][PAGE_SIZE];
	};
	union {
		uint32_t samples[2];
		char samples_pages[NUM_PAGES_SAMPLES][PAGE_SIZE];
	};
};

typedef struct {
	uint32_t physaddr;   /* Starting address */
	dma_cb_t *cb_last;   /* Last CB */
} bs_info_t;

typedef struct {
	bs_info_t bs[SYM_COUNT];
	uint32_t physaddr;
} ts_info_t;

ts_info_t ts_info[TS_COUNT];
int ts_last;
dma_cb_t *ts_last_cbp;
int ts_last_sym;

static const int ts_next[SYM_COUNT][2] = {
	[SYM_L] = {SYM_LH, SYM_L},
	[SYM_H] = {SYM_HL, SYM_H},
	[SYM_LH] = {SYM_HL, SYM_H},
	[SYM_HL] = {SYM_LH, SYM_L},
};

typedef struct {
    uint8_t *virtaddr;
    uint32_t physaddr;
} page_map_t;

page_map_t *page_map;
page_map_t *phys_info;

static uint8_t *virtbase;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

static int delay_hw = DELAY_VIA_PWM;

static double pi;
static double option_amplitude = 0.9;
static double option_frequency = 0;
static int option_div = 0;
static int option_mash = 3;
static double option_rc = 4700.0 * 0.000001;
static int option_timeout = -1;
static double level_error_max;

typedef struct {
	int b_len;
	int b_val;
} burst_t;

static const burst_t starting_burst = {20, 0};
static const burst_t ending_burst = {20, 0x000fffff};
static const burst_t fill_burst = {1, 0};
static const burst_t idle_burst = {1, 1};

/* Each character is separated by last two zeros. The bits are sent lsbit first. */
static const burst_t varicode_table[] = {
	{12, 0x0355}, /* ASCII =   0 101010101100 */
	{12, 0x036d}, /* ASCII =   1 101101101100 */
	{12, 0x02dd}, /* ASCII =   2 101110110100 */
	{12, 0x03bb}, /* ASCII =   3 110111011100 */
	{12, 0x035d}, /* ASCII =   4 101110101100 */
	{12, 0x03eb}, /* ASCII =   5 110101111100 */
	{12, 0x03dd}, /* ASCII =   6 101110111100 */
	{12, 0x02fd}, /* ASCII =   7 101111110100 */
	{12, 0x03fd}, /* ASCII =   8 101111111100 */
	{10, 0x00f7}, /* ASCII =   9 1110111100 */
	{ 7, 0x0017}, /* ASCII =  10 1110100 */
	{12, 0x03db}, /* ASCII =  11 110110111100 */
	{12, 0x02ed}, /* ASCII =  12 101101110100 */
	{ 7, 0x001f}, /* ASCII =  13 1111100 */
	{12, 0x02bb}, /* ASCII =  14 110111010100 */
	{12, 0x0357}, /* ASCII =  15 111010101100 */
	{12, 0x03bd}, /* ASCII =  16 101111011100 */
	{12, 0x02bd}, /* ASCII =  17 101111010100 */
	{12, 0x02d7}, /* ASCII =  18 111010110100 */
	{12, 0x03d7}, /* ASCII =  19 111010111100 */
	{12, 0x036b}, /* ASCII =  20 110101101100 */
	{12, 0x035b}, /* ASCII =  21 110110101100 */
	{12, 0x02db}, /* ASCII =  22 110110110100 */
	{12, 0x03ab}, /* ASCII =  23 110101011100 */
	{12, 0x037b}, /* ASCII =  24 110111101100 */
	{12, 0x02fb}, /* ASCII =  25 110111110100 */
	{12, 0x03b7}, /* ASCII =  26 111011011100 */
	{12, 0x02ab}, /* ASCII =  27 110101010100 */
	{12, 0x02eb}, /* ASCII =  28 110101110100 */
	{12, 0x0377}, /* ASCII =  29 111011101100 */
	{12, 0x037d}, /* ASCII =  30 101111101100 */
	{12, 0x03fb}, /* ASCII =  31 110111111100 */
	{ 3, 0x0001}, /* ASCII = ' ' 100 */
	{11, 0x01ff}, /* ASCII = '!' 11111111100 */
	{11, 0x01f5}, /* ASCII = '"' 10101111100 */
	{11, 0x015f}, /* ASCII = '#' 11111010100 */
	{11, 0x01b7}, /* ASCII = '$' 11101101100 */
	{12, 0x02ad}, /* ASCII = '%' 101101010100 */
	{12, 0x0375}, /* ASCII = '&' 101011101100 */
	{11, 0x01fd}, /* ASCII = ''' 10111111100 */
	{10, 0x00df}, /* ASCII = '(' 1111101100 */
	{10, 0x00ef}, /* ASCII = ')' 1111011100 */
	{11, 0x01ed}, /* ASCII = '*' 10110111100 */
	{11, 0x01f7}, /* ASCII = '+' 11101111100 */
	{ 9, 0x0057}, /* ASCII = ',' 111010100 */
	{ 8, 0x002b}, /* ASCII = '-' 11010100 */
	{ 9, 0x0075}, /* ASCII = '.' 101011100 */
	{11, 0x01eb}, /* ASCII = '/' 11010111100 */
	{10, 0x00ed}, /* ASCII = '0' 1011011100 */
	{10, 0x00bd}, /* ASCII = '1' 1011110100 */
	{10, 0x00b7}, /* ASCII = '2' 1110110100 */
	{10, 0x00ff}, /* ASCII = '3' 1111111100 */
	{11, 0x01dd}, /* ASCII = '4' 10111011100 */
	{11, 0x01b5}, /* ASCII = '5' 10101101100 */
	{11, 0x01ad}, /* ASCII = '6' 10110101100 */
	{11, 0x016b}, /* ASCII = '7' 11010110100 */
	{11, 0x01ab}, /* ASCII = '8' 11010101100 */
	{11, 0x01db}, /* ASCII = '9' 11011011100 */
	{10, 0x00af}, /* ASCII = ':' 1111010100 */
	{11, 0x017b}, /* ASCII = ';' 11011110100 */
	{11, 0x016f}, /* ASCII = '<' 11110110100 */
	{ 9, 0x0055}, /* ASCII = '=' 101010100 */
	{11, 0x01d7}, /* ASCII = '>' 11101011100 */
	{12, 0x03d5}, /* ASCII = '?' 101010111100 */
	{12, 0x02f5}, /* ASCII = '@' 101011110100 */
	{ 9, 0x005f}, /* ASCII = 'A' 111110100 */
	{10, 0x00d7}, /* ASCII = 'B' 1110101100 */
	{10, 0x00b5}, /* ASCII = 'C' 1010110100 */
	{10, 0x00ad}, /* ASCII = 'D' 1011010100 */
	{ 9, 0x0077}, /* ASCII = 'E' 111011100 */
	{10, 0x00db}, /* ASCII = 'F' 1101101100 */
	{10, 0x00bf}, /* ASCII = 'G' 1111110100 */
	{11, 0x0155}, /* ASCII = 'H' 10101010100 */
	{ 9, 0x007f}, /* ASCII = 'I' 111111100 */
	{11, 0x017f}, /* ASCII = 'J' 11111110100 */
	{11, 0x017d}, /* ASCII = 'K' 10111110100 */
	{10, 0x00eb}, /* ASCII = 'L' 1101011100 */
	{10, 0x00dd}, /* ASCII = 'M' 1011101100 */
	{10, 0x00bb}, /* ASCII = 'N' 1101110100 */
	{10, 0x00d5}, /* ASCII = 'O' 1010101100 */
	{10, 0x00ab}, /* ASCII = 'P' 1101010100 */
	{11, 0x0177}, /* ASCII = 'Q' 11101110100 */
	{10, 0x00f5}, /* ASCII = 'R' 1010111100 */
	{ 9, 0x007b}, /* ASCII = 'S' 110111100 */
	{ 9, 0x005b}, /* ASCII = 'T' 110110100 */
	{11, 0x01d5}, /* ASCII = 'U' 10101011100 */
	{11, 0x015b}, /* ASCII = 'V' 11011010100 */
	{11, 0x0175}, /* ASCII = 'W' 10101110100 */
	{11, 0x015d}, /* ASCII = 'X' 10111010100 */
	{11, 0x01bd}, /* ASCII = 'Y' 10111101100 */
	{12, 0x02d5}, /* ASCII = 'Z' 101010110100 */
	{11, 0x01df}, /* ASCII = '[' 11111011100 */
	{11, 0x01ef}, /* ASCII = '\' 11110111100 */
	{11, 0x01bf}, /* ASCII = ']' 11111101100 */
	{12, 0x03f5}, /* ASCII = '^' 101011111100 */
	{11, 0x016d}, /* ASCII = '_' 10110110100 */
	{12, 0x03ed}, /* ASCII = '`' 101101111100 */
	{ 6, 0x000d}, /* ASCII = 'a' 101100 */
	{ 9, 0x007d}, /* ASCII = 'b' 101111100 */
	{ 8, 0x003d}, /* ASCII = 'c' 10111100 */
	{ 8, 0x002d}, /* ASCII = 'd' 10110100 */
	{ 4, 0x0003}, /* ASCII = 'e' 1100 */
	{ 8, 0x002f}, /* ASCII = 'f' 11110100 */
	{ 9, 0x006d}, /* ASCII = 'g' 101101100 */
	{ 8, 0x0035}, /* ASCII = 'h' 10101100 */
	{ 6, 0x000b}, /* ASCII = 'i' 110100 */
	{11, 0x01af}, /* ASCII = 'j' 11110101100 */
	{10, 0x00fd}, /* ASCII = 'k' 1011111100 */
	{ 7, 0x001b}, /* ASCII = 'l' 1101100 */
	{ 8, 0x0037}, /* ASCII = 'm' 11101100 */
	{ 6, 0x000f}, /* ASCII = 'n' 111100 */
	{ 5, 0x0007}, /* ASCII = 'o' 11100 */
	{ 8, 0x003f}, /* ASCII = 'p' 11111100 */
	{11, 0x01fb}, /* ASCII = 'q' 11011111100 */
	{ 7, 0x0015}, /* ASCII = 'r' 1010100 */
	{ 7, 0x001d}, /* ASCII = 's' 1011100 */
	{ 5, 0x0005}, /* ASCII = 't' 10100 */
	{ 8, 0x003b}, /* ASCII = 'u' 11011100 */
	{ 9, 0x006f}, /* ASCII = 'v' 111101100 */
	{ 9, 0x006b}, /* ASCII = 'w' 110101100 */
	{10, 0x00fb}, /* ASCII = 'x' 1101111100 */
	{ 9, 0x005d}, /* ASCII = 'y' 101110100 */
	{11, 0x0157}, /* ASCII = 'z' 11101010100 */
	{12, 0x03b5}, /* ASCII = '{' 101011011100 */
	{11, 0x01bb}, /* ASCII = '|' 11011101100 */
	{12, 0x02b5}, /* ASCII = '}' 101011010100 */
	{12, 0x03ad}, /* ASCII = '~' 101101011100 */
	{12, 0x02b7}, /* ASCII = 127 111011010100 */
	{12, 0x02f7}, /* ASCII = 128 111011110100 */
	{12, 0x03f7}, /* ASCII = 129 111011111100 */
	{12, 0x02af}, /* ASCII = 130 111101010100 */
	{12, 0x03af}, /* ASCII = 131 111101011100 */
	{12, 0x036f}, /* ASCII = 132 111101101100 */
	{12, 0x02ef}, /* ASCII = 133 111101110100 */
	{12, 0x03ef}, /* ASCII = 134 111101111100 */
	{12, 0x035f}, /* ASCII = 135 111110101100 */
	{12, 0x02df}, /* ASCII = 136 111110110100 */
	{12, 0x03df}, /* ASCII = 137 111110111100 */
	{12, 0x02bf}, /* ASCII = 138 111111010100 */
	{12, 0x03bf}, /* ASCII = 139 111111011100 */
	{12, 0x037f}, /* ASCII = 140 111111101100 */
	{12, 0x02ff}, /* ASCII = 141 111111110100 */
	{12, 0x03ff}, /* ASCII = 142 111111111100 */
	{13, 0x0555}, /* ASCII = 143 1010101010100 */
	{13, 0x0755}, /* ASCII = 144 1010101011100 */
	{13, 0x06d5}, /* ASCII = 145 1010101101100 */
	{13, 0x05d5}, /* ASCII = 146 1010101110100 */
	{13, 0x07d5}, /* ASCII = 147 1010101111100 */
	{13, 0x06b5}, /* ASCII = 148 1010110101100 */
	{13, 0x05b5}, /* ASCII = 149 1010110110100 */
	{13, 0x07b5}, /* ASCII = 150 1010110111100 */
	{13, 0x0575}, /* ASCII = 151 1010111010100 */
	{13, 0x0775}, /* ASCII = 152 1010111011100 */
	{13, 0x06f5}, /* ASCII = 153 1010111101100 */
	{13, 0x05f5}, /* ASCII = 154 1010111110100 */
	{13, 0x07f5}, /* ASCII = 155 1010111111100 */
	{13, 0x06ad}, /* ASCII = 156 1011010101100 */
	{13, 0x05ad}, /* ASCII = 157 1011010110100 */
	{13, 0x07ad}, /* ASCII = 158 1011010111100 */
	{13, 0x056d}, /* ASCII = 159 1011011010100 */
	{13, 0x076d}, /* ASCII = 160 1011011011100 */
	{13, 0x06ed}, /* ASCII = 161 1011011101100 */
	{13, 0x05ed}, /* ASCII = 162 1011011110100 */
	{13, 0x07ed}, /* ASCII = 163 1011011111100 */
	{13, 0x055d}, /* ASCII = 164 1011101010100 */
	{13, 0x075d}, /* ASCII = 165 1011101011100 */
	{13, 0x06dd}, /* ASCII = 166 1011101101100 */
	{13, 0x05dd}, /* ASCII = 167 1011101110100 */
	{13, 0x07dd}, /* ASCII = 168 1011101111100 */
	{13, 0x06bd}, /* ASCII = 169 1011110101100 */
	{13, 0x05bd}, /* ASCII = 170 1011110110100 */
	{13, 0x07bd}, /* ASCII = 171 1011110111100 */
	{13, 0x057d}, /* ASCII = 172 1011111010100 */
	{13, 0x077d}, /* ASCII = 173 1011111011100 */
	{13, 0x06fd}, /* ASCII = 174 1011111101100 */
	{13, 0x05fd}, /* ASCII = 175 1011111110100 */
	{13, 0x07fd}, /* ASCII = 176 1011111111100 */
	{13, 0x06ab}, /* ASCII = 177 1101010101100 */
	{13, 0x05ab}, /* ASCII = 178 1101010110100 */
	{13, 0x07ab}, /* ASCII = 179 1101010111100 */
	{13, 0x056b}, /* ASCII = 180 1101011010100 */
	{13, 0x076b}, /* ASCII = 181 1101011011100 */
	{13, 0x06eb}, /* ASCII = 182 1101011101100 */
	{13, 0x05eb}, /* ASCII = 183 1101011110100 */
	{13, 0x07eb}, /* ASCII = 184 1101011111100 */
	{13, 0x055b}, /* ASCII = 185 1101101010100 */
	{13, 0x075b}, /* ASCII = 186 1101101011100 */
	{13, 0x06db}, /* ASCII = 187 1101101101100 */
	{13, 0x05db}, /* ASCII = 188 1101101110100 */
	{13, 0x07db}, /* ASCII = 189 1101101111100 */
	{13, 0x06bb}, /* ASCII = 190 1101110101100 */
	{13, 0x05bb}, /* ASCII = 191 1101110110100 */
	{13, 0x07bb}, /* ASCII = 192 1101110111100 */
	{13, 0x057b}, /* ASCII = 193 1101111010100 */
	{13, 0x077b}, /* ASCII = 194 1101111011100 */
	{13, 0x06fb}, /* ASCII = 195 1101111101100 */
	{13, 0x05fb}, /* ASCII = 196 1101111110100 */
	{13, 0x07fb}, /* ASCII = 197 1101111111100 */
	{13, 0x0557}, /* ASCII = 198 1110101010100 */
	{13, 0x0757}, /* ASCII = 199 1110101011100 */
	{13, 0x06d7}, /* ASCII = 200 1110101101100 */
	{13, 0x05d7}, /* ASCII = 201 1110101110100 */
	{13, 0x07d7}, /* ASCII = 202 1110101111100 */
	{13, 0x06b7}, /* ASCII = 203 1110110101100 */
	{13, 0x05b7}, /* ASCII = 204 1110110110100 */
	{13, 0x07b7}, /* ASCII = 205 1110110111100 */
	{13, 0x0577}, /* ASCII = 206 1110111010100 */
	{13, 0x0777}, /* ASCII = 207 1110111011100 */
	{13, 0x06f7}, /* ASCII = 208 1110111101100 */
	{13, 0x05f7}, /* ASCII = 209 1110111110100 */
	{13, 0x07f7}, /* ASCII = 210 1110111111100 */
	{13, 0x06af}, /* ASCII = 211 1111010101100 */
	{13, 0x05af}, /* ASCII = 212 1111010110100 */
	{13, 0x07af}, /* ASCII = 213 1111010111100 */
	{13, 0x056f}, /* ASCII = 214 1111011010100 */
	{13, 0x076f}, /* ASCII = 215 1111011011100 */
	{13, 0x06ef}, /* ASCII = 216 1111011101100 */
	{13, 0x05ef}, /* ASCII = 217 1111011110100 */
	{13, 0x07ef}, /* ASCII = 218 1111011111100 */
	{13, 0x055f}, /* ASCII = 219 1111101010100 */
	{13, 0x075f}, /* ASCII = 220 1111101011100 */
	{13, 0x06df}, /* ASCII = 221 1111101101100 */
	{13, 0x05df}, /* ASCII = 222 1111101110100 */
	{13, 0x07df}, /* ASCII = 223 1111101111100 */
	{13, 0x06bf}, /* ASCII = 224 1111110101100 */
	{13, 0x05bf}, /* ASCII = 225 1111110110100 */
	{13, 0x07bf}, /* ASCII = 226 1111110111100 */
	{13, 0x057f}, /* ASCII = 227 1111111010100 */
	{13, 0x077f}, /* ASCII = 228 1111111011100 */
	{13, 0x06ff}, /* ASCII = 229 1111111101100 */
	{13, 0x05ff}, /* ASCII = 230 1111111110100 */
	{13, 0x07ff}, /* ASCII = 231 1111111111100 */
	{14, 0x0d55}, /* ASCII = 232 10101010101100 */
	{14, 0x0b55}, /* ASCII = 233 10101010110100 */
	{14, 0x0f55}, /* ASCII = 234 10101010111100 */
	{14, 0x0ad5}, /* ASCII = 235 10101011010100 */
	{14, 0x0ed5}, /* ASCII = 236 10101011011100 */
	{14, 0x0dd5}, /* ASCII = 237 10101011101100 */
	{14, 0x0bd5}, /* ASCII = 238 10101011110100 */
	{14, 0x0fd5}, /* ASCII = 239 10101011111100 */
	{14, 0x0ab5}, /* ASCII = 240 10101101010100 */
	{14, 0x0eb5}, /* ASCII = 241 10101101011100 */
	{14, 0x0db5}, /* ASCII = 242 10101101101100 */
	{14, 0x0bb5}, /* ASCII = 243 10101101110100 */
	{14, 0x0fb5}, /* ASCII = 244 10101101111100 */
	{14, 0x0d75}, /* ASCII = 245 10101110101100 */
	{14, 0x0b75}, /* ASCII = 246 10101110110100 */
	{14, 0x0f75}, /* ASCII = 247 10101110111100 */
	{14, 0x0af5}, /* ASCII = 248 10101111010100 */
	{14, 0x0ef5}, /* ASCII = 249 10101111011100 */
	{14, 0x0df5}, /* ASCII = 250 10101111101100 */
	{14, 0x0bf5}, /* ASCII = 251 10101111110100 */
	{14, 0x0ff5}, /* ASCII = 252 10101111111100 */
	{14, 0x0aad}, /* ASCII = 253 10110101010100 */
	{14, 0x0ead}, /* ASCII = 254 10110101011100 */
	{14, 0x0dad}, /* ASCII = 255 10110101101100 */
};

typedef struct {
	uint32_t c_div;
	int c_mash;
} clock_cb_t;

clock_cb_t clock_cb = {
	.c_div = 0,
	.c_mash = 0,
};

// Sets a GPIO to either GPIO_MODE_IN(=0) or GPIO_MODE_OUT(=1)
static void gpio_set_mode(uint32_t pin, uint32_t mode) {
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + pin/10];

	fsel &= ~(7 << ((pin % 10) * 3));
	fsel |= mode << ((pin % 10) * 3);
	gpio_reg[GPIO_FSEL0 + pin/10] = fsel;
}

// Sets the gpio to input (level=1) or output (level=0)
static void
gpio_set(int pin, int level)
{
    if (level)
        gpio_reg[GPIO_SET0] = 1 << pin;
    else
        gpio_reg[GPIO_CLR0] = 1 << pin;
}

// Very short delay
static void
udelay(int us)
{
    struct timespec ts = { 0, us * 1000 };

    nanosleep(&ts, NULL);
}

static void devfiles_unlink(void) {
	unlink(DEVFILE_SEND);
	unlink(DEVFILE_CTRL);
	unlink(DEVFILE_STAT);
}

static void clock_stop(void) {
	clk_reg[CM_GP0CTL] = 0x5a000000 | (clk_reg[CM_GP0CTL] & 0x0000070f);
	while ((clk_reg[CM_GP0CTL] & 0x00000080) != 0)
		/*usleep(1000)*/;
	clock_cb.c_div = 0;
}

static void clock_start(void) {
	uint32_t div, divi;
	uint32_t mash;
	uint32_t ctl;
	const struct {
		uint32_t divi_min;
		uint32_t divi_dec;
		uint32_t divi_inc;
	} dt[] = {{2, 0, 1}, {3, 1, 2}, {5, 3, 4}};

	/* Stop the clock */
	clock_stop();
	if (option_div > 0 && option_div <= 0x00fff000)
		div = option_div;
	else if (option_frequency >= 500.0 * (double)(1 << 12) / (double)0x00fff000)
		div = (uint32_t)((500.0 / option_frequency) * (double)(1 << 12) + 0.5);
	else
		return;
	gpio_set_mode(GPIO_FREQ_NUM, GPIO_MODE_ALT0);
	/* Setup new frequency */
	divi = div >> 12;
	if (divi < 1 || div > 0x00fff000)
		return;
	clk_reg[CM_GP0DIV] = 0x5a000000 | div;
	if (option_mash >= -3 && option_mash <= 0) {
		mash = -option_mash;
	} else {
		if (option_mash >= 3)
			mash = 3;
		else
			mash = option_mash;
		for (; mash; mash--) {
			if (divi < dt[mash - 1].divi_min)
				continue;
			if (divi < 500 / 25 + dt[mash - 1].divi_dec)
				continue;
			/* This might not be a restriction, but this way it is safer. */
			if (divi > 4095 - dt[mash - 1].divi_inc)
				continue;
			break;
		}
	}
	ctl = 0x5a000006 | (mash << 9);
	clk_reg[CM_GP0CTL] = ctl;
	clk_reg[CM_GP0CTL] = ctl | 0x00000010;
	clock_cb.c_div = div;
	clock_cb.c_mash = mash;
}

// Shutdown -- its super important to reset the DMA before quitting
static void terminate(int dummy) {
	if (dma_reg && virtbase) {
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
	}
	clock_stop();
	devfiles_unlink();
	exit(1);
}

// Shutdown with an error
static void fatal(char *fmt, ...) {
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	terminate(0);
}

static void devfile_create(const char *devfile_name, mode_t mode) {
	if (mkfifo(devfile_name, mode) < 0)
		fatal("rpio-pwm: Failed to create %s: %m\n", devfile_name);
	if (chmod(devfile_name, mode) < 0)
		fatal("rpio-pwm: Failed to set permissions on %s: %m\n", devfile_name);
}

static void devfiles_create(void) {
	devfile_create(DEVFILE_SEND, 0622);
	devfile_create(DEVFILE_CTRL, 0622);
}

// Catch all signals possible - it is vital we kill the DMA engine
// on process exit!
static void setup_sighandlers(void) {
	int i;
	for (i = 0; i < 64; i++) {
		struct sigaction sa;
		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}
}

// Memory mapping
static uint32_t mem_virt_to_phys(void *virt) {
	uint32_t offset = (uint8_t *)virt - virtbase;

	return page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
}

#if 0
static void *mem_phys_to_virt(uint32_t phys) {
	int l, u, m;

	l = 0;
	u = NUM_PAGES_CBS;
	while (u > l + 1) {
		m = (l + u) / 2;
		if (phys >= phys_info[m].physaddr)
			l = m;
		else
			u = m;
	}
	if ((phys >> PAGE_SHIFT) != (phys_info[l].physaddr >> PAGE_SHIFT))
		fatal("rpio-pwm: invalid phys addr\n");
	return phys_info[l].virtaddr + (phys % PAGE_SIZE);
}
#endif

static int tx_sym_pending(void) {
	uint32_t phys;
	int l, m, u;

	/* Retrieve current TS */
	phys = dma_reg[DMA_CONBLK_AD];
	if (phys == 0)
		fatal("rpio-pwm: DMA stopped\n");
	l = 0;
	u = TS_COUNT;
	while (u > l + 1) {
		m = (l + u) / 2;
		if (phys >= ts_info[m].physaddr)
			l = m;
		else
			u = m;
	}
	return (ts_last - l) & (TS_COUNT - 1);
}

static void tx_sym_enqueue(int s) {
	ts_info_t *ti;
	bs_info_t *bs;

#if 0
{
	static int sent = 0;
	static const char s_char[][2] = {
		[SYM_L] = {' ', '_'},
		[SYM_H] = {'_', ' '},
		[SYM_LH] = {' ', '/'},
		[SYM_HL] = {' ', '\\'},
	};
	static char buf[2][51];

	buf[0][sent] = s_char[s][0];
	buf[1][sent] = s_char[s][1];
	if (++sent == 50) {
		buf[0][sent] = 0;
		buf[1][sent] = 0;
		sent = 0;
		printf(" %s\n[%s]\n", buf[0], buf[1]);
	}
}
#endif
	if (!ts_last_cbp)
		ts_last = 0;
	else
		ts_last = (ts_last + 1) % TS_COUNT;
	ti = &ts_info[ts_last];
	bs = &ti->bs[s];
	out32(&bs->cb_last->next, 0);
	if (ts_last_cbp)
		out32(&ts_last_cbp->next, bs->physaddr);
	ts_last_cbp = bs->cb_last;
	ts_last_sym = s;
}

// More memory mapping
static void *map_peripheral(uint32_t base, uint32_t len) {
	int fd = open("/dev/mem", O_RDWR);
	void * vaddr;

	if (fd < 0)
		fatal("rpio-pwm: Failed to open /dev/mem: %m\n");
	vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
	if (vaddr == MAP_FAILED)
		fatal("rpio-pwm: Failed to map peripheral at 0x%08x: %m\n", base);
	close(fd);

	return vaddr;
}

// Initialize the memory pagemap
static void make_pagemap(void) {
	int i, fd, memfd, pid;
	char pagemap_fn[64];

	page_map = malloc(NUM_PAGES * sizeof(*page_map));
	if (page_map == 0)
		fatal("rpio-pwm: Failed to malloc page_map: %m\n");
	memfd = open("/dev/mem", O_RDWR);
	if (memfd < 0)
		fatal("rpio-pwm: Failed to open /dev/mem: %m\n");
	pid = getpid();
	sprintf(pagemap_fn, "/proc/%d/pagemap", pid);
	fd = open(pagemap_fn, O_RDONLY);
	if (fd < 0)
		fatal("rpio-pwm: Failed to open %s: %m\n", pagemap_fn);
	if (lseek(fd, (uint32_t)virtbase >> 9, SEEK_SET) != (uint32_t)virtbase >> 9)
		fatal("rpio-pwm: Failed to seek on %s: %m\n", pagemap_fn);
	for (i = 0; i < NUM_PAGES; i++) {
		uint64_t pfn;
		page_map[i].virtaddr = virtbase + i * PAGE_SIZE;
		// Following line forces page to be allocated
		page_map[i].virtaddr[0] = 0;
		if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn))
			fatal("rpio-pwm: Failed to read %s: %m\n", pagemap_fn);
		if (((pfn >> 55) & 0x1bf) != 0x10c)
			fatal("rpio-pwm: Page %d not present (pfn 0x%016llx)\n", i, pfn);
		page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
	}
	close(fd);
	close(memfd);
}

static int make_physinfo_cmp(const void *v1, const void *v2) {
	const page_map_t *pi1 = (const page_map_t *)v1;
	const page_map_t *pi2 = (const page_map_t *)v2;

	if (pi1->physaddr < pi2->physaddr)
		return -1;
	if (pi1->physaddr > pi2->physaddr)
		return 1;
	return 0;
}

static void make_physinfo(void) {
	if (!(phys_info = (typeof(phys_info))malloc(NUM_PAGES_CBS * sizeof(*phys_info))))
		fatal("rpio-pwm: Failed to malloc phys_info: %m\n");
	memcpy(phys_info, page_map, NUM_PAGES_CBS * sizeof(*phys_info));
	qsort(phys_info, NUM_PAGES_CBS, sizeof(*phys_info), make_physinfo_cmp);
}

static uint32_t cb_offset_to_phys(uint32_t cb_offset) {
	return phys_info[cb_offset >> PAGE_SHIFT].physaddr + (cb_offset % PAGE_SIZE);
}

static void *cb_offset_to_virt(uint32_t cb_offset) {
	return phys_info[cb_offset >> PAGE_SHIFT].virtaddr + (cb_offset % PAGE_SIZE);
}

#define LEVEL_MIN (0.5 - option_amplitude / 2)
#define LEVEL_MAX (0.5 + option_amplitude / 2)
#define LEVEL_MED (0.5)

static double sym_l_fn(double t) {
	return LEVEL_MIN;
}

static double sym_h_fn(double t) {
	return LEVEL_MAX;
}

static double sym_lh_fn(double t) {
	return LEVEL_MED - cos(pi * t) * (LEVEL_MAX - LEVEL_MED);
}

static double sym_hl_fn(double t) {
	return LEVEL_MED + cos(pi * t) * (LEVEL_MAX - LEVEL_MED);
}

typedef struct {
	double (*sd_fn)(double t);
} sd_t;

static const sd_t sym_def[SYM_COUNT] = {
	[SYM_L] = {.sd_fn = sym_l_fn},
	[SYM_H] = {.sd_fn = sym_h_fn},
	[SYM_LH] = {.sd_fn = sym_lh_fn},
	[SYM_HL] = {.sd_fn = sym_hl_fn},
};

static uint32_t init_bs(bs_info_t *bs, const sd_t *sd, uint32_t cb_offset, uint32_t phys_sample_pos, uint32_t phys_sample_neg) {
	dma_cb_t *cbp;
	uint32_t cb_phys;
	int i;
	uint32_t cbp_info;
	uint32_t phys_fifo_addr;
	uint32_t phys_gpclr0 = 0x7e200000 + 0x28;
	uint32_t phys_gpset0 = 0x7e200000 + 0x1c;
	double v_old, v_new, v, v_error;
	int up, up_old;
	double mean_decay;
	double mean_weight;

	mean_decay = exp(-((double)PULSE_WIDTH_INCR_US) / (1000000.0 * option_rc));
	mean_weight = 1.0 - mean_decay;

	if (delay_hw == DELAY_VIA_PWM) {
		cbp_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
		phys_fifo_addr = (PWM_BASE | 0x7e000000) + 0x18;
	} else {
		cbp_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
		phys_fifo_addr = (PCM_BASE | 0x7e000000) + 0x04;
	}

	cb_phys = cb_offset_to_phys(cb_offset);
	bs->physaddr = cb_phys;
	cbp = NULL;
	v_old = sd->sd_fn(0);
	up_old = 0; /* To avoid warnings */
	for (i = 0; i < BS_SAMPLES; i++) {
		/* Get new cb physical address */
		cb_phys = cb_offset_to_phys(cb_offset);
		/* Link previous cb to new cb */
		if (cbp)
			cbp->next = cb_phys;
		/* Get new target value */
		v = sd->sd_fn((i + 1) / (double)BS_SAMPLES);
		up = (v > v_old);
		v_new = v_old * mean_decay;
		if (up)
			v_new += mean_weight;
		/* Compute error statistics */
		v_error = fabs(v - v_new);
		if (v_error > level_error_max)
			level_error_max = v_error;
		/* Write cb */
		if (i == 0 || up_old != up) {
			/* Positive pad */
			cbp = (dma_cb_t *)cb_offset_to_virt(cb_offset);
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
			cbp->src = phys_sample_pos;
			cbp->dst = up ? phys_gpset0 : phys_gpclr0;
			cbp->length = 4;
			cbp->stride = 0;
			cb_offset += 32;
			cb_phys = cb_offset_to_phys(cb_offset);
			cbp->next = cb_phys;
			/* Negative pad */
			cbp = (dma_cb_t *)cb_offset_to_virt(cb_offset);
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
			cbp->src = phys_sample_neg;
			cbp->dst = up ? phys_gpclr0 : phys_gpset0;
			cbp->length = 4;
			cbp->stride = 0;
			cb_offset += 32;
			cb_phys = cb_offset_to_phys(cb_offset);
			cbp->next = cb_phys;
		}
		// Delay
		cbp = (dma_cb_t *)cb_offset_to_virt(cb_offset);
		cbp->info = cbp_info;
		cbp->src = phys_sample_pos;    // Any data will do
		cbp->dst = phys_fifo_addr;
		cbp->length = 4;
		cbp->stride = 0;
		cb_offset += 32;

		up_old = up;
		v_old = v_new;
	}
	bs->cb_last = cbp;
	return cb_offset;
}

static void init_ctrl_data(void) {
	struct ctl *ctl;
	int ts;
	uint32_t cb_offset;
	void *cb_virt;
	ts_info_t *ti;
	uint32_t phys_sample_pos;
	uint32_t phys_sample_neg;
	int s;

	/* Generate waveforms */
	level_error_max = 0;
	ctl = (struct ctl *)virtbase;
	memset(ctl, 0, sizeof(*ctl));
	ctl->samples[0] = (1 << GPIO_POS_NUM);
	ctl->samples[1] = (1 << GPIO_NEG_NUM);
	phys_sample_pos = mem_virt_to_phys(&ctl->samples[0]);
	phys_sample_neg = mem_virt_to_phys(&ctl->samples[1]);
	ti = ts_info;
	cb_offset = 0;
	for (ti = ts_info, ts = 0; ts < TS_COUNT; ti++, ts++) {
		ti->physaddr = cb_offset_to_phys(cb_offset);
		for (s = 0; s < SYM_COUNT; s++)
			cb_offset = init_bs(&ti->bs[s], &sym_def[s], cb_offset, phys_sample_pos, phys_sample_neg);
	}
	/* Free unused memory */
	cb_offset = (cb_offset + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
	while (cb_offset < sizeof(ctl->cb_pages)) {
		cb_virt = cb_offset_to_virt(cb_offset);
		if (munmap(cb_virt, PAGE_SIZE) != 0)
			fatal("psk31: munmap failed: %m\n");
		cb_offset += PAGE_SIZE;
	}
}

// Initialize PWM (or PCM) and DMA
static void init_hardware(void) {
	int i;
	uint32_t phys;

	/* Setup idle burst */
	ts_last_cbp = NULL;
	for (i = 0; i < TS_COUNT; i++)
		tx_sym_enqueue(SYM_H);
	phys = ts_info[0].bs[SYM_H].physaddr;

	if (delay_hw == DELAY_VIA_PWM) {
		// Initialise PWM
		pwm_reg[PWM_CTL] = 0;
		udelay(10);
		clk_reg[PWMCLK_CNTL] = 0x5A000006;        // Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PWMCLK_DIV] = 0x5A000000 | (50<<12);    // set pwm div to 50, giving 10MHz
		udelay(100);
		clk_reg[PWMCLK_CNTL] = 0x5A000016;        // Source=PLLD and enable
		udelay(100);
		pwm_reg[PWM_RNG1] = PULSE_WIDTH_INCR_US * 10;
		udelay(10);
		pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
		udelay(10);
		pwm_reg[PWM_CTL] = PWMCTL_CLRF;
		udelay(10);
		pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
		udelay(10);
	} else {
		// Initialise PCM
		pcm_reg[PCM_CS_A] = 1;                // Disable Rx+Tx, Enable PCM block
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000006;        // Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PCMCLK_DIV] = 0x5A000000 | (50<<12);    // Set pcm div to 50, giving 10MHz
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000016;        // Source=PLLD and enable
		udelay(100);
		pcm_reg[PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
		udelay(100);
		pcm_reg[PCM_MODE_A] = (PULSE_WIDTH_INCR_US * 10 - 1) << 10;
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;        // Clear FIFOs
		udelay(100);
		pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;        // DMA Req when one slot is free?
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<9;            // Enable DMA
		udelay(100);
	}

	// Initialise the DMA
	dma_reg[DMA_CS] = DMA_RESET;
	udelay(10);
	dma_reg[DMA_CS] = DMA_INT | DMA_END;
	dma_reg[DMA_CONBLK_AD] = phys;
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = 0x10880001;    // go, mid priority, wait for outstanding writes

	if (delay_hw == DELAY_VIA_PCM) {
		pcm_reg[PCM_CS_A] |= 1<<2;            // Enable Tx
	}
}

static void term_hardware(void) {
	dma_reg[DMA_CS] = DMA_RESET;
	udelay(10);
}

#define max(a, b) ((a) > (b) ? (a) : (b))

typedef struct stat_s {
	struct stat_s *s_next;
	int s_fd;
	int s_read;
	int s_count;
	char *s_buf;
} stat_t;

const struct sockaddr_un stat_addr = {
	.sun_family = AF_UNIX,
	.sun_path = DEVFILE_STAT,
};

static int stat_fd_set(int fd_max, int fd_stat, stat_t *stat_head, fd_set *readfs, fd_set *writefs) {
	stat_t *s;

	for (s = stat_head; s; s = s->s_next) {
		fd_max = max(fd_max, s->s_fd);
		FD_SET(s->s_fd, writefs);
	}
	fd_max = max(fd_max, fd_stat);
	FD_SET(fd_stat, readfs);
	return fd_max;
}

static void stat_accept(int fd_stat, stat_t **stat_head, fd_set *readfs, int sendcount) {
	int fd;
	stat_t *s;

	if (!FD_ISSET(fd_stat, readfs))
		return;
	for (;;) {
		if ((fd = accept(fd_stat, NULL, 0)) == -1) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				return;
			fatal("psk31: accept error: %m\n");
		}
//		printf("psk31: accept %d\n", fd);
		if (!(s = (stat_t *)malloc(sizeof(*s))))
			fatal("psk31: accept oom\n");
		s->s_fd = fd;
		s->s_read = 0;
		s->s_count = asprintf(&s->s_buf,
			"amplitude %f\n"
			"rc %f\n"
			"clock_div %u\n"
			"clock_mash %d\n"
			"clock_freq %f\n"
			"timeout %d\n"
			"pending_char %d\n",
			option_amplitude,
			option_rc,
			(unsigned)clock_cb.c_div,
			clock_cb.c_mash,
			clock_cb.c_div ? 500.0 * (double)(1 << 12) / (double)clock_cb.c_div : 0,
			option_timeout,
			sendcount);
		if (s->s_count == -1)
			fatal("psk31: asprintf oom\n");
		s->s_next = *stat_head;
		*stat_head = s;
	}
}

static void stat_write(stat_t **stat_head, fd_set *writefs) {
	stat_t **sp;
	stat_t *s;
	ssize_t ss;

	for (sp = stat_head; (s = *sp) != NULL; ) {
		if (!FD_ISSET(s->s_fd, writefs)) {
			sp = &s->s_next;
			continue;
		}
//		printf("psk31: write %d\n", s->s_fd);
		ss = send(s->s_fd, s->s_buf + s->s_read, s->s_count - s->s_read, MSG_NOSIGNAL);
		if (ss == -1) {
			if (errno != EPIPE)
				fatal("psk31: stat write error: %m\n");
		} else {
			s->s_read += ss;
			s->s_count -= ss;
			if (s->s_count != 0) {
				sp = &s->s_next;
				continue;
			}
		}
//		printf("psk31: close %d\n", s->s_fd);
		if (close(s->s_fd) == -1)
			fatal("psk31: stat close error: %m\n");
		*sp = s->s_next;
		free(s->s_buf);
		free(s);
	}
}

// Endless loop to read the FIFO DEVFILE_SEND and set the servos according
// to the values in the FIFO
static void go_go_go(void) {
	int fd_send;
	int fd_stat;
	stat_t *stat_head;
	int fd_max;
#define SENDSIZE 128
	unsigned char sendbuf[SENDSIZE];
	int sendread, sendwrite, sendcount;
	fd_set readfs;
	fd_set writefs;
	struct timeval tv;
	burst_t curburst;
	int n;
	enum {
		STATE_START,
		STATE_SEND,
		STATE_FILL,
		STATE_STOP,
		STATE_IDLE,
	} state;
	int fill_timeout = 0;

	/* Files for communication */
	fd_send = -1;
	stat_head = NULL;
	if ((fd_stat = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1)
		fatal("psk31: socket error: %m\n");
	if (bind(fd_stat, (struct sockaddr *)&stat_addr, sizeof(struct sockaddr_un)) == -1)
		fatal("psk31: bind error: %m\n");
	if (chmod(DEVFILE_STAT, 0666) < 0)
		fatal("psk31: failed to set permissions on %s: %m\n", DEVFILE_STAT);
	if (listen(fd_stat, 5) == -1)
		fatal("psk31: listen error: %m\n");
	sendread = sendwrite = 0;
	sendcount = 0;
	curburst.b_len = 0;
	curburst.b_val = 0; /* To avoid compiler warnings */
	state = STATE_IDLE;
	for (;;) {
		if (fd_send == -1 && ((fd_send = open(DEVFILE_SEND, O_RDONLY | O_NONBLOCK)) == -1))
			fatal("psk31: Failed to open %s: %m\n", DEVFILE_SEND);
#if 0
		if (fd_ctrl == -1 && ((fd_ctrl = open(DEVFILE_CTRL, O_RDONLY | O_NONBLOCK)) == -1))
			fatal("rpio-pwm: Failed to open %s: %m\n", DEVFILE_CTRL);
		if (fd_stat == -1 && ((fd_stat = open(DEVFILE_STAT, O_WRONLY | O_NONBLOCK)) == -1))
			fatal("rpio-pwm: Failed to open %s: %m\n", DEVFILE_STAT);
#endif
		FD_ZERO(&readfs);
		FD_ZERO(&writefs);
		fd_max = 0;
		if (sendcount < SENDSIZE) {
			FD_SET(fd_send, &readfs);
			fd_max = max(fd_max, fd_send);
		}
		fd_max = stat_fd_set(fd_max, fd_stat, stat_head, &readfs, &writefs);
		tv.tv_sec = 0;
		tv.tv_usec = TS_US * TS_COUNT / 4;
		n = select(fd_max + 1, &readfs, &writefs, NULL, &tv);
		if (n < 0)
			fatal("psk31: select error: %m\n");

		/* Status */
		stat_accept(fd_stat, &stat_head, &readfs, sendcount);
		stat_write(&stat_head, &writefs);

		/* Fill in the buffer */
		if (FD_ISSET(fd_send, &readfs))
			while (sendcount < SENDSIZE) {
				ssize_t ss;
				int n;

				n = SENDSIZE - sendcount;
				if (n > SENDSIZE - sendwrite)
					n = SENDSIZE - sendwrite;
				ss = read(fd_send, &sendbuf[sendwrite], n);
				if (ss == -1) {
					if (errno == EAGAIN || errno == EWOULDBLOCK)
						break;
					fatal("rpio-pwm: %s read error: %m\n", DEVFILE_SEND);
				} else if (ss <= 0) {
					close(fd_send);
					fd_send = -1;
					break;
				} else {
					sendcount += ss;
					if ((sendwrite += ss) == SENDSIZE)
						sendwrite = 0;
				}
			}

		/* Feed the hw */
		for (n = TS_COUNT - 1 - tx_sym_pending(); n > 0; n--) {
			/* Get burst of bits to be sent */
			while (curburst.b_len == 0) {
				switch (state) {
					case STATE_START:
						state = STATE_SEND;
//						printf("state start->send\n");
						break;
					case STATE_SEND:
						if (sendcount) {
							curburst = varicode_table[sendbuf[sendread]];
							sendcount--;
							if (++sendread == SENDSIZE)
								sendread = 0;
//							printf("state send: load 0x%x(%d)\n", curburst.b_val, curburst.b_len);
						} else {
							fill_timeout = option_timeout;
							state = STATE_FILL;
//							printf("state send->fill %d\n", fill_timeout);
						}
						break;
					case STATE_FILL:
						if (sendcount) {
							state = STATE_SEND;
//							printf("state fill->send\n");
						} else if (fill_timeout != 0) {
							curburst = fill_burst;
							if (fill_timeout > 0)
								fill_timeout--;
//							printf("state fill %d\n", fill_timeout);
						} else {
							state = STATE_STOP;
							curburst = ending_burst;
//							printf("state fill->stop\n");
						}
						break;
					case STATE_STOP:
						state = STATE_IDLE;
//						printf("state stop->idle\n");
						break;
					case STATE_IDLE:
						if (option_timeout < 0 || sendcount) {
							state = STATE_START;
							curburst = starting_burst;
//							printf("state idle->start\n");
						} else {
							curburst = idle_burst;
//							printf("state idle\n");
						}
						break;
				}
			}

			/* Send one bit from burst */
			tx_sym_enqueue(ts_next[ts_last_sym][curburst.b_val & 1]);
			curburst.b_val >>= 1;
			curburst.b_len--;
		}
	}
#if 0
finish:
	if (fd_send != -1)
		close(fd_send);
#endif
}

static const struct option long_options[] = {
	{"amplitude", required_argument, NULL, 'a'},
	{"clock-div", required_argument, NULL, 'd'},
	{"frequency", required_argument, NULL, 'f'},
	{"help", no_argument, NULL, 'h'},
	{"mash", required_argument, NULL, 'm'},
	{"pcm", no_argument, NULL, 'p'},
	{"rc", required_argument, NULL, 'r'},
	{"timeout", required_argument, NULL, 't'},
	{NULL, 0, NULL, 0}
};

int main(int argc, char **argv) {
	pi = atan(1) * 4;

	while (1) {
		int opt;
		int opt_index;

		opt_index = 0;
		opt = getopt_long(argc, argv, "a:pt:", long_options, &opt_index);
		if (opt == -1)
			break;
		switch (opt) {
			case 'a':
				option_amplitude = atof(optarg);
				break;
			case 'd':
				option_div = atoi(optarg);
				break;
			case 'f':
				option_frequency = atof(optarg);
				break;
			case 'h':
				fprintf(stderr,
					"Options:\n"
					"  --amplitude=<n>     Signal amplitude (0 .. 1]\n"
					"  --clock-div=<n>     Fractional divisor for carrier [4096 .. 16773120]\n"
					"                      Note: frequency = 500 MHz / (clock-div / 4096)\n"
					"  --frequency=<f>     Carrier frequency, in MHz [0.125 .. 500]\n"
					"                      Note: this is overridden by clock-div\n"
					"  --help              Show this help\n"
					"  --mash=<n>          Set number of MASH stages [0 .. 3]\n"
					"  --pcm               Use PCM clock instead of PWM clock for signal generation\n"
					"  --rc=<f>            Set signal filter RC value (s)\n"
					"  --timeout=<n>       Number of zeros before switching off. 0 for infinite.\n");
				return 0;
			case 'm':
				option_mash = atoi(optarg);
				break;
			case 'p':
				delay_hw = DELAY_VIA_PCM;
				break;
			case 'r':
				option_rc = atof(optarg);
				break;
			case 't':
				option_timeout = atoi(optarg);
				break;
			default:
				fatal("psk31: invalid options\n");
		}
	}

	printf("Using hardware:       %s\n", delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	printf("RC:                   %fs\n", option_rc);
	printf("Amplitude:            %f\n", option_amplitude);
	printf("Timeout:              %d\n", option_timeout);
	printf("Symbol time:          %dus\n", BS_US);
	printf("Buffer time:          %dus\n", TS_COUNT * TS_US);
	printf("Clock div:            %d\n", option_div);
	printf("Mash:                 %d\n", option_mash);
	printf("Frequency:            %f\n", option_frequency);

	setup_sighandlers();

	dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
	pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
	pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
	clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

	/* TODO: retrieve PAGE_SIZE from system */
	virtbase = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ|PROT_WRITE,
	        MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
	        -1, 0);
	if (virtbase == MAP_FAILED)
		fatal("rpio-pwm: Failed to mmap physical pages: %m\n");
	if ((unsigned long)virtbase & (PAGE_SIZE-1))
		fatal("rpio-pwm: Virtual address is not page aligned\n");

	make_pagemap();
	make_physinfo();

	gpio_set(GPIO_POS_NUM, 1);
	gpio_set(GPIO_NEG_NUM, 0);
	gpio_set_mode(GPIO_POS_NUM, GPIO_MODE_OUT);
	gpio_set_mode(GPIO_NEG_NUM, GPIO_MODE_OUT);

	clock_start();

	init_ctrl_data();
	printf("Max. error:           %fmV\n", level_error_max * 3300);
	init_hardware();

	devfiles_unlink();
	devfiles_create();

	if (daemon(0,1) < 0)
		fatal("rpio-pwm: Failed to daemonize process: %m\n");

	go_go_go();

	term_hardware();
	clock_stop();
	devfiles_unlink();

	return 0;
}

