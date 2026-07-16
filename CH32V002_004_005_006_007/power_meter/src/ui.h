#ifndef POWER_METER_UI_H
#define POWER_METER_UI_H

void UI_Init(void);
void Process_Timer_Event(unsigned int keyboard_status);
int StopLogging(void);
int StartLogging(void);
int StartRecording(void);
int StopRecording(void);

#endif
