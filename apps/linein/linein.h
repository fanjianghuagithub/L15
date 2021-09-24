
#ifndef __LINEIN_H__
#define __LINEIN_H__

#ifdef __cplusplus
extern "C" {
#endif

void linein_gpio_init(void);

void linein_irq_en(void);
void linein_Send_Cmd(void);



#ifdef __cplusplus
}
#endif

#endif

