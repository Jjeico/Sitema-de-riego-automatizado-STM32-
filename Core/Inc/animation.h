#include "fonts.h"
#include "ssd1306.h"
#include "stdio.h"
#include "planta.h"

#ifndef INC_ANIMATION_H_
#define INC_ANIMATION_H_

//BEGIN PLANT ANIMATION
void animation1(){
   	  SSD1306_DrawBitmap(45,0,p1,51,64,1); //IMAGEN, 1 PARA AZUL, 0 PARA NEGRO
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p1,51,64,0); //REEMPLAZA EL CLEAR, POR ALGUNA RAZON EL CLEAR TILDEA LA PANTALLA, ADEMAS, CON ESTO NO SE NECESITA EL DELAY. ESTO FUNCIONA BORRANDO LOS PIXELES DEL BITMAP SELECCIONADO, HACIENDO UNA ANIMACION MÁS SUAVE, CON DIFERENTES CONFIGURACIONES SE PUEDEN HACER DIFERENTES TIPOS DE ANIMACIONES (POR EJEMPLO, QUE EL AGUA VAYA HACIA ARRIBA Y HACIA ABAJO). INTERESANTE!!
   	  SSD1306_DrawBitmap(45,0,p2,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p2,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p3,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p3,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p4,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p4,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p5,51,64,1);
   	  SSD1306_UpdateScreen();;
   	  SSD1306_DrawBitmap(45,0,p5,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p6,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p6,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p7,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p7,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p8,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p8,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p9,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p9,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p10,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p10,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p11,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p11,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p12,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p12,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p13,51,64,1);
   	  SSD1306_UpdateScreen();;
   	  SSD1306_DrawBitmap(45,0,p13,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p14,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p14,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p15,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p15,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p16,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p16,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p17,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p17,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p18,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p18,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p19,51,64,1);
   	  SSD1306_UpdateScreen();
   	  SSD1306_DrawBitmap(45,0,p19,51,64,0);
   	  SSD1306_DrawBitmap(45,0,p20,51,64,1);
   	  SSD1306_UpdateScreen();
}
//END PLANT ANIMATION

#endif /* INC_ANIMATION_H_ */
