/*
 * Splash.h
 *
 *  Created on: Oct 21, 2017
 *      Author: bbagbyd
 */

#ifndef SPLASH_H_
#define SPLASH_H_

#define DISPLAY_TIME 2 //Splash screen display time
#define RED     0x001F
#define GREEN   0x07E0
#define BLACK   0x0000
#define BLUE    0xF800
#define CYAN    0xFFE0
#define MAGENTA 0xF81F
#define YELLOW  0x07FF
#define WHITE   0xFFFF

int splash(void);
int unlock(void);
int lock(void);

#endif /* SPLASH_H_ */
