#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif

void Delay_Init(void);
void Delay_Us (unsigned int n);
void Delay_Ms (unsigned int n);

#ifdef __cplusplus
}
#endif

#endif 
