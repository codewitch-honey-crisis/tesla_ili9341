#include <Arduino.h>
#include <tft_io.hpp>
#include <ili9341.hpp>
#include <gfx_cpp14.hpp>
#include <SPIFFS.h>
using namespace arduino;
using namespace gfx;

#define LCD_HOST    VSPI
// change to your setup
#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22
#define LCD_WRITE_SPEED_PERCENT 400 // 40MHz
#define LCD_READ_SPEED_PERCENT 200 // 20MHz
#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BKL 5
#define LCD_BKL_HIGH false
#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_ROTATION 1

using bus_type = tft_spi_ex<LCD_HOST,
                            PIN_NUM_CS,
                            PIN_NUM_MOSI,
                            PIN_NUM_MISO,
                            PIN_NUM_CLK,
                            SPI_MODE0,
                            PIN_NUM_MISO<0
#ifdef OPTIMIZE_DMA
                            ,(LCD_WIDTH*LCD_HEIGHT)*2+8
#endif
>;

using lcd_type = ili9341<PIN_NUM_DC,
                        PIN_NUM_RST,
                        PIN_NUM_BKL,
                        bus_type,
                        LCD_ROTATION,
                        LCD_BKL_HIGH,
                        LCD_WRITE_SPEED_PERCENT,
                        LCD_READ_SPEED_PERCENT>;

using lcd_color = color<typename lcd_type::pixel_type>;
lcd_type lcd;
uint8_t* bungee_otf_buffer;
size_t bungee_otf_buffer_len;
void draw_text(spoint16 loc, const open_font& fnt,const char* text, float scale, rgb_pixel<16> fg,rgb_pixel<16> bg) {
  size16 text_size = (size16)fnt.measure_text((ssize16)lcd.dimensions(),{0,0},text,scale);
  // just a little padding
  text_size.width+=2;
  text_size.height+=2;
  using bt = bitmap<rgb_pixel<16>>;
  uint8_t* buf = (uint8_t*)malloc(bt::sizeof_buffer(text_size));
  if(nullptr==buf) {
    return;
  }
  bt tmp(text_size,buf);
  tmp.fill(tmp.bounds(),bg);
  draw::text(tmp,(srect16)tmp.bounds(),{0,0},text,fnt,scale,fg);
  draw::bitmap(lcd,srect16(loc,(spoint16)lcd.bounds().bottom_right()),tmp,tmp.bounds());
  free(buf);
}

void setup() {
  Serial.begin(115200);
  SPIFFS.begin(false);
  File file = SPIFFS.open("/Bungee.otf","rb");
  if(!file)  {
    Serial.println("Asset Bungee.otf not found. Halting.");
    while(true) delay(1000);
  }
  file.seek(0,fs::SeekMode::SeekEnd);
  size_t len = file.position();
  file.seek(0);
  bungee_otf_buffer = (uint8_t*)ps_malloc(len);
  if(!bungee_otf_buffer)  {
    Serial.println("Unable to allocate PSRAM for asset Bungee.otf. Halting.");
    while(true) delay(1000);
  }
  file.readBytes((char*)bungee_otf_buffer,len);
  file.close();
  bungee_otf_buffer_len = len;
  // setup is complete

  // fill the screen
  lcd.fill(lcd.bounds(),lcd_color::white);
  // reconstitute the font from the buffer
  const_buffer_stream cbs(bungee_otf_buffer,bungee_otf_buffer_len);
  open_font fnt;
  gfx_result r=open_font::open(&cbs,&fnt);
  if(r!=gfx_result::success) {
    Serial.println("Unable to load asset Bungee.otf. Halting.");
    while(true) delay(1000);
  }

  draw_text({10,10},fnt,"Hello world!",fnt.scale(40),lcd_color::indian_red,lcd_color::white);
}

void loop() {
  // put your main code here, to run repeatedly:
}