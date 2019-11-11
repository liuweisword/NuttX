/************************************************************************************
 * arch/arm/src/artosyn/chip/ar8020_pinmap.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            liuwei  <wei.liu@cecooleye.cn>
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARTOSYN_CHIP_AR8020_PINMAP_H
#define __ARCH_ARM_SRC_ARTOSYN_CHIP_AR8020_PINMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
#define GPIO_MODE_SHIFT               (10)                        /* Bits 10-11: GPIO port mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_DEFAULT                (0 << GPIO_MODE_SHIFT)     /* Default mode */
#  define GPIO_TEST                   (1 << GPIO_MODE_SHIFT)     /* Test mode */
#  define GPIO_INPUT                  (2 << GPIO_MODE_SHIFT)     /* Input mode */
#  define GPIO_OUTPUT                 (3 << GPIO_MODE_SHIFT)     /* Output mode */


#define GPIO_EXTI_SHIFT               (9)                        /* Bits 9: Output-reset/Output-set */
#define GPIO_EXTI_MASK                (1 << GPIO_EXTI_SHIFT)
#  define GPIO_EXTI                   (1 << GPIO_EXTI_SHIFT)     


#define GPIO_OUTPUT_SHIFT             (8)                        /* Bits 8: Output-reset/Output-set */
#define GPIO_OUTPUT_MASK              (1 << GPIO_OUTPUT_SHIFT)
#  define GPIO_OUTRESET                  (0 << GPIO_OUTPUT_SHIFT)     /* Output reset */
#  define GPIO_OUTSET                    (1 << GPIO_OUTPUT_SHIFT)     /* Output-set */


#define GPIO_PIN_SHIFT                (0)                        /* Bits 0-7: GPIO number: 0-117 */
#define GPIO_PIN_MASK                 (255 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                   (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                   (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN16                  (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17                  (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18                  (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19                  (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20                  (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21                  (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22                  (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23                  (23 << GPIO_PIN_SHIFT)
#  define GPIO_PIN24                  (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25                  (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26                  (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27                  (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28                  (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29                  (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30                  (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31                  (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN32                  (32 << GPIO_PIN_SHIFT)
#  define GPIO_PIN33                  (33 << GPIO_PIN_SHIFT)
#  define GPIO_PIN34                  (34 << GPIO_PIN_SHIFT)
#  define GPIO_PIN35                  (35 << GPIO_PIN_SHIFT)
#  define GPIO_PIN36                  (36 << GPIO_PIN_SHIFT)
#  define GPIO_PIN37                  (37 << GPIO_PIN_SHIFT)
#  define GPIO_PIN38                  (38 << GPIO_PIN_SHIFT)
#  define GPIO_PIN39                  (39 << GPIO_PIN_SHIFT)
#  define GPIO_PIN40                  (40 << GPIO_PIN_SHIFT)
#  define GPIO_PIN41                  (41 << GPIO_PIN_SHIFT)
#  define GPIO_PIN42                  (42 << GPIO_PIN_SHIFT)
#  define GPIO_PIN43                  (43 << GPIO_PIN_SHIFT)
#  define GPIO_PIN44                  (44 << GPIO_PIN_SHIFT)
#  define GPIO_PIN45                  (45 << GPIO_PIN_SHIFT)
#  define GPIO_PIN46                  (46 << GPIO_PIN_SHIFT)
#  define GPIO_PIN47                  (47 << GPIO_PIN_SHIFT)
#  define GPIO_PIN48                  (48 << GPIO_PIN_SHIFT)
#  define GPIO_PIN49                  (49 << GPIO_PIN_SHIFT)
#  define GPIO_PIN50                  (50 << GPIO_PIN_SHIFT)
#  define GPIO_PIN51                  (51 << GPIO_PIN_SHIFT)
#  define GPIO_PIN52                  (52 << GPIO_PIN_SHIFT)
#  define GPIO_PIN53                  (53 << GPIO_PIN_SHIFT)
#  define GPIO_PIN54                  (54 << GPIO_PIN_SHIFT)
#  define GPIO_PIN55                  (55 << GPIO_PIN_SHIFT)
#  define GPIO_PIN56                  (56 << GPIO_PIN_SHIFT)
#  define GPIO_PIN57                  (57 << GPIO_PIN_SHIFT)
#  define GPIO_PIN58                  (58 << GPIO_PIN_SHIFT)
#  define GPIO_PIN59                  (59 << GPIO_PIN_SHIFT)
#  define GPIO_PIN60                  (60 << GPIO_PIN_SHIFT)
#  define GPIO_PIN61                  (61 << GPIO_PIN_SHIFT)
#  define GPIO_PIN62                  (62 << GPIO_PIN_SHIFT)
#  define GPIO_PIN63                  (63 << GPIO_PIN_SHIFT)
#  define GPIO_PIN64                  (64 << GPIO_PIN_SHIFT)
#  define GPIO_PIN65                  (65 << GPIO_PIN_SHIFT)
#  define GPIO_PIN66                  (66 << GPIO_PIN_SHIFT)
#  define GPIO_PIN67                  (67 << GPIO_PIN_SHIFT)
#  define GPIO_PIN68                  (68 << GPIO_PIN_SHIFT)
#  define GPIO_PIN69                  (69 << GPIO_PIN_SHIFT)
#  define GPIO_PIN70                  (70 << GPIO_PIN_SHIFT)
#  define GPIO_PIN71                  (71 << GPIO_PIN_SHIFT)
#  define GPIO_PIN72                  (72 << GPIO_PIN_SHIFT)
#  define GPIO_PIN73                  (73 << GPIO_PIN_SHIFT)
#  define GPIO_PIN74                  (74 << GPIO_PIN_SHIFT)
#  define GPIO_PIN75                  (75 << GPIO_PIN_SHIFT)
#  define GPIO_PIN76                  (76 << GPIO_PIN_SHIFT)
#  define GPIO_PIN77                  (77 << GPIO_PIN_SHIFT)
#  define GPIO_PIN78                  (78 << GPIO_PIN_SHIFT)
#  define GPIO_PIN79                  (79 << GPIO_PIN_SHIFT)
#  define GPIO_PIN80                  (80 << GPIO_PIN_SHIFT)
#  define GPIO_PIN81                  (81 << GPIO_PIN_SHIFT)
#  define GPIO_PIN82                  (82 << GPIO_PIN_SHIFT)
#  define GPIO_PIN83                  (83 << GPIO_PIN_SHIFT)
#  define GPIO_PIN84                  (84 << GPIO_PIN_SHIFT)
#  define GPIO_PIN85                  (85 << GPIO_PIN_SHIFT)
#  define GPIO_PIN86                  (86 << GPIO_PIN_SHIFT)
#  define GPIO_PIN87                  (87 << GPIO_PIN_SHIFT)
#  define GPIO_PIN88                  (88 << GPIO_PIN_SHIFT)
#  define GPIO_PIN89                  (89 << GPIO_PIN_SHIFT)
#  define GPIO_PIN90                  (90 << GPIO_PIN_SHIFT)
#  define GPIO_PIN91                  (91 << GPIO_PIN_SHIFT)
#  define GPIO_PIN92                  (92 << GPIO_PIN_SHIFT)
#  define GPIO_PIN93                  (93 << GPIO_PIN_SHIFT)
#  define GPIO_PIN94                  (94 << GPIO_PIN_SHIFT)
#  define GPIO_PIN95                  (95 << GPIO_PIN_SHIFT)
#  define GPIO_PIN96                  (96 << GPIO_PIN_SHIFT)
#  define GPIO_PIN97                  (97 << GPIO_PIN_SHIFT)
#  define GPIO_PIN98                  (98 << GPIO_PIN_SHIFT)
#  define GPIO_PIN99                  (99 << GPIO_PIN_SHIFT)
#  define GPIO_PIN100                 (100 << GPIO_PIN_SHIFT)
#  define GPIO_PIN101                 (101 << GPIO_PIN_SHIFT)
#  define GPIO_PIN102                 (102 << GPIO_PIN_SHIFT)
#  define GPIO_PIN103                 (103 << GPIO_PIN_SHIFT)
#  define GPIO_PIN104                 (104 << GPIO_PIN_SHIFT)
#  define GPIO_PIN105                 (105 << GPIO_PIN_SHIFT)
#  define GPIO_PIN106                 (106 << GPIO_PIN_SHIFT)
#  define GPIO_PIN107                 (107 << GPIO_PIN_SHIFT)
#  define GPIO_PIN108                 (108 << GPIO_PIN_SHIFT)
#  define GPIO_PIN109                 (109 << GPIO_PIN_SHIFT)
#  define GPIO_PIN110                 (110 << GPIO_PIN_SHIFT)
#  define GPIO_PIN111                 (111 << GPIO_PIN_SHIFT)
#  define GPIO_PIN112                 (112 << GPIO_PIN_SHIFT)
#  define GPIO_PIN113                 (113 << GPIO_PIN_SHIFT)
#  define GPIO_PIN114                 (114 << GPIO_PIN_SHIFT)
#  define GPIO_PIN115                 (115 << GPIO_PIN_SHIFT)
#  define GPIO_PIN116                 (116 << GPIO_PIN_SHIFT)
#  define GPIO_PIN117                 (117 << GPIO_PIN_SHIFT)

/* JUST FOR SOME FUNCTION : NEVER USED THIS AS GPIO*/
#  define GPIO_PIN118                 (118 << GPIO_PIN_SHIFT)
#  define GPIO_PIN119                 (119 << GPIO_PIN_SHIFT)
#  define GPIO_PIN120                 (120 << GPIO_PIN_SHIFT)
#  define GPIO_PIN121                 (121 << GPIO_PIN_SHIFT)
#  define GPIO_PIN122                 (122 << GPIO_PIN_SHIFT)
#  define GPIO_PIN123                 (123 << GPIO_PIN_SHIFT)
#  define GPIO_PIN124                 (124 << GPIO_PIN_SHIFT)
#  define GPIO_PIN125                 (125 << GPIO_PIN_SHIFT)
#  define GPIO_PIN126                 (126 << GPIO_PIN_SHIFT)
#  define GPIO_PIN127                 (127 << GPIO_PIN_SHIFT)

#define GPIO_SPI6_NSS           (GPIO_DEFAULT|GPIO_PIN46)
#define GPIO_SPI6_SCK           (GPIO_DEFAULT|GPIO_PIN47)
#define GPIO_SPI6_MOSI          (GPIO_DEFAULT|GPIO_PIN48)
#define GPIO_SPI6_MISO          (GPIO_DEFAULT|GPIO_PIN49)
    
#define GPIO_SPI4_NSS           (GPIO_DEFAULT|GPIO_PIN54)
#define GPIO_SPI4_SCK           (GPIO_DEFAULT|GPIO_PIN55)
#define GPIO_SPI4_MOSI          (GPIO_DEFAULT|GPIO_PIN56)
#define GPIO_SPI4_MISO          (GPIO_DEFAULT|GPIO_PIN57)


#define GPIO_SPI0_NSS           (GPIO_DEFAULT|GPIO_PIN70)
#define GPIO_SPI0_SCK           (GPIO_DEFAULT|GPIO_PIN66)
#define GPIO_SPI0_MOSI          (GPIO_DEFAULT|GPIO_PIN62)
#define GPIO_SPI0_MISO          (GPIO_DEFAULT|GPIO_PIN58)

#define GPIO_TIM0_7_CH0PWM      (GPIO_DEFAULT|GPIO_PIN80)
#define GPIO_TIM0_7_CH0IN       (GPIO_INPUT  |GPIO_PIN80)
#define GPIO_TIM0_7_CH1PWM      (GPIO_DEFAULT|GPIO_PIN81)
#define GPIO_TIM0_7_CH1IN       (GPIO_INPUT  |GPIO_PIN81)
#define GPIO_TIM0_7_CH2PWM      (GPIO_DEFAULT|GPIO_PIN82)
#define GPIO_TIM0_7_CH2IN       (GPIO_INPUT  |GPIO_PIN82)
#define GPIO_TIM0_7_CH3PWM      (GPIO_DEFAULT|GPIO_PIN83)
#define GPIO_TIM0_7_CH3IN       (GPIO_INPUT  |GPIO_PIN83)
#define GPIO_TIM0_7_CH4PWM      (GPIO_DEFAULT|GPIO_PIN84)
#define GPIO_TIM0_7_CH4IN       (GPIO_INPUT  |GPIO_PIN84)
#define GPIO_TIM0_7_CH5PWM      (GPIO_DEFAULT|GPIO_PIN85)
#define GPIO_TIM0_7_CH5IN       (GPIO_INPUT  |GPIO_PIN85)
#define GPIO_TIM0_7_CH6PWM      (GPIO_DEFAULT|GPIO_PIN86)
#define GPIO_TIM0_7_CH6IN       (GPIO_INPUT  |GPIO_PIN86)
#define GPIO_TIM0_7_CH7PWM      (GPIO_DEFAULT|GPIO_PIN87)
#define GPIO_TIM0_7_CH7IN       (GPIO_INPUT  |GPIO_PIN87)




#endif /* __ARCH_ARM_SRC_ARTOSYN_CHIP_AR8020_PINMAP_H */
