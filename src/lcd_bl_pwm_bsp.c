#include <stdio.h>
#include "lcd_bl_pwm_bsp.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "lcd_config.h"
static void GPIO_SET(uint8_t pin,uint8_t mode)
{
  gpio_set_level(pin,mode);
}
static uint8_t GPIO_GET(uint8_t pin)
{
  return gpio_get_level(pin);
}
static void bl_gpio_init(void)
{
  gpio_config_t gpio_conf = {};
  gpio_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_conf.mode = GPIO_MODE_OUTPUT;
  gpio_conf.pin_bit_mask = ((uint64_t)0X01<<PIN_NUM_BL);
  gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;

  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_conf));
}
static void lcd_bl_pwm_bsp_init(uint16_t duty)
{ 
  ledc_timer_config_t timer_conf = 
  {
    .speed_mode =  LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT, //256
    .timer_num =  LEDC_TIMER_3,
    .freq_hz = 50 * 1000,
    .clk_cfg = LEDC_USE_RTC8M_CLK,
  };
  ledc_channel_config_t ledc_conf = 
  {
    .gpio_num = PIN_NUM_BL,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel =  LEDC_CHANNEL_1,
    .intr_type =  LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_3,
    .duty = duty,   //占空比
    .hpoint = 0,    //相位
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_timer_config(&timer_conf));
  ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_channel_config(&ledc_conf));
}

static void setUpduty(uint16_t duty)
{
  ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty));
  ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}
void setUpdutySubdivide(uint8_t mode , uint16_t duty) //0: GPIO模式
{
  static uint8_t _mode = 0;
  if(mode != _mode)
  {
    if(mode == 0)  //GPIO 模式
    {
      gpio_reset_pin(PIN_NUM_BL);
      bl_gpio_init();
    }
    else
    {
      gpio_reset_pin(PIN_NUM_BL);
      lcd_bl_pwm_bsp_init(duty); //PWM模式
    }
    _mode = mode;
  }
  if(mode == 0)
  {
    if(duty > 0)
    {
      GPIO_SET(PIN_NUM_BL,1);
    }
    else
    {
      GPIO_SET(PIN_NUM_BL,0);
    }
  }
  else
  {
    setUpduty(duty);
  }
}


