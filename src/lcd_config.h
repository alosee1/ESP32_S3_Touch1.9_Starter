#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H

#define LCD_HOST    SPI3_HOST

#define Rotate 1    //Rotation 90
#define Normal 0    //Normal
#define Direction   Normal


#if (Direction == Normal) 
  #define EXAMPLE_LCD_H_RES 170   //宽度 水平分辨率
  #define EXAMPLE_LCD_V_RES 320   //高度 竖直分辨率
#elif (Direction == Rotate)
  #define EXAMPLE_LCD_H_RES 320   //宽度 水平分辨率
  #define EXAMPLE_LCD_V_RES 170   //高度 竖直分辨率
#endif

#define EXAMPLE_LCD_DMA_Line (EXAMPLE_LCD_V_RES / 2)

#define EXAMPLE_USE_Disp       1
#define EXAMPLE_USE_TOUCH      1

#define PIN_NUM_RST  9
#define PIN_NUM_CLK  10
#define PIN_NUM_DC   11
#define PIN_NUM_CS   12
#define PIN_NUM_DIN  13
#define PIN_NUM_BL   14

#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 4)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2                          //Timer time
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500                        //LVGL Indicates the maximum time for a task to run
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1                          //LVGL Minimum time to run a task
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)                 //LVGL runs the task stack
#define EXAMPLE_LVGL_TASK_PRIORITY     2                          //LVGL Running task priority



#define I2C_Touch_ADDR 0x15
#define EXAMPLE_PIN_NUM_TOUCH_SCL 48
#define EXAMPLE_PIN_NUM_TOUCH_SDA 47

#endif