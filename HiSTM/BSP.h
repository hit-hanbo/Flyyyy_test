#ifndef BSP_H_
#define BSP_H_

#define  SYS_LD_ON      GPIOB->ODR &= ~(1 << 14)
#define  SYS_LD_OFF     GPIOB->ODR |=  (1 << 14)
#define  SYS_LD_TOGGLE  GPIOB->ODR ^=  (1 << 14)
#define  STA_LD_ON      GPIOB->ODR &= ~(1 << 15)
#define  STA_LD_OFF     GPIOB->ODR |=  (1 << 15)
#define  STA_LD_TOGGLE  GPIOB->ODR ^=  (1 << 15)

#endif
