#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <tft_io.hpp>
#include <ili9341.hpp>
#include "tft_touch.hpp"
#include <gfx_cpp14.hpp>

using namespace arduino;
using namespace gfx;

#define LCD_HOST    VSPI
// change to your setup
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define LCD_WRITE_SPEED_PERCENT 400 // 40MHz
#define LCD_READ_SPEED_PERCENT 200 // 20MHz
#define PIN_NUM_DC   2
#define PIN_NUM_RST  4
#define PIN_NUM_BKL 22
#define LCD_BKL_HIGH true
#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_ROTATION 3

#define TOUCH_HOST VSPI
#define PIN_NUM_T_CLK 18
#define PIN_NUM_T_CS 15
#define PIN_NUM_T_MISO 19
#define PIN_NUM_T_MOSI 23

using bus_type = tft_spi<LCD_HOST,
                            PIN_NUM_CS,
                            SPI_MODE0
#ifdef OPTIMIZE_DMA
                            ,(LCD_WIDTH*LCD_HEIGHT)*2+8
#endif
>;
//using touch_bus_type = tft_spi<TOUCH_HOST,PIN_NUM_T_CS,SPI_MODE0>;

using lcd_type = ili9341<PIN_NUM_DC,
                        PIN_NUM_RST,
                        PIN_NUM_BKL,
                        bus_type,
                        LCD_ROTATION,
                        LCD_BKL_HIGH,
                        LCD_WRITE_SPEED_PERCENT,
                        LCD_READ_SPEED_PERCENT>;

//using touch_type = tft_touch<touch_bus_type>;
using touch_type = tft_touch<TOUCH_HOST,PIN_NUM_T_CS>;

using lcd_color = color<typename lcd_type::pixel_type>;
lcd_type lcd;
touch_type touch(spi_container<TOUCH_HOST>::instance());

// if you change the font, you'll have to tweak
// the code in draw_speed() to place it properly.
const char* speed_font_path = "/Telegrama.otf"; //"/Bungee.otf"; // "/Ubuntu.otf";
const char* speed_font_name = speed_font_path+1;
uint8_t* speed_font_buffer;
size_t speed_font_buffer_len;
ssize16 speed_size;
char speed_buf[16];
int speed_threshold = 70;
// copies a speed into an ascii
// buffer, returning it
char* speed_text(int speed) {
  gfx::helpers::clamp(speed,0,999);
  itoa(speed,speed_buf,10);
  return speed_buf;
}

// draws the speed on the screen
void draw_speed(int speed,const char* units) {
  using pt = typename lcd_type::pixel_type;
  static const pt speed_col = color<pt>::black;
  static const pt speeding_col = color<pt>::indian_red;
  static const pt unit_col = color<pt>::blue;
  static const pt bg_col = color<pt>::white;
  // reconstitute the font from the buffer
  const_buffer_stream cbs(speed_font_buffer,speed_font_buffer_len);
  open_font fnt;
  open_font::open(&cbs,&fnt);
  // refresh the speed buffer
  speed_text(speed);
  // get the scale for the speed part
  float spd_scale = fnt.scale(80);
  // measure it
  ssize16 spd_size = fnt.measure_text({32767,32767},{0,0},speed_buf,spd_scale);
  // pad it a little
  spd_size = spd_size.inflate(2,2);
  // get the scale for the units part
  float uni_scale = fnt.scale(40);
  // measure it
  ssize16 uni_size = fnt.measure_text({32767,32767},{0,0},units,uni_scale);
  // pad it
  uni_size=uni_size.inflate(2,2);
  // our bitmap is the size of the *maximum* width of the speed, plus the units width
  // and the height of the (maximum implied) the speed text.
  size16 bmp_size = size16(speed_size.width+uni_size.width,speed_size.height);
  using bmp_type = bitmap<typename lcd_type::pixel_type>;
  // allocate a buffer. malloc is faster than ps_malloc here
  // note that we could be recycling a buffer, but we don't because KISS
  uint8_t* buf = (uint8_t*)malloc(
        bmp_type::sizeof_buffer(bmp_size));
  if(buf==nullptr) return;
  bmp_type tmp(bmp_size,buf);
  // fill the background
  tmp.fill(tmp.bounds(),bg_col);
  // draw the speed text
  draw::text(tmp,srect16(speed_size.width-spd_size.width,0,tmp.bounds().x2,tmp.bounds().y2),{0,0},speed_buf,fnt,spd_scale,speed>speed_threshold?speeding_col:speed_col);
  // draw the units text
  draw::text(tmp,srect16(bmp_size.width-uni_size.width,bmp_size.height-uni_size.height-8,tmp.bounds().x2,tmp.bounds().y2),{0,0},units,fnt,uni_scale,unit_col);
  // draw the bitmap to the screen
  draw::bitmap(lcd,(srect16)lcd.bounds().offset(20,70).crop(lcd.bounds()),tmp,tmp.bounds());
  // we're done with the buffer.
  free(buf);
    
}
// computes the *maximum* size for the speed.
// the speed text ends up right aligned to the region
// indicated by this size
ssize16 compute_speed_size() {
  const_buffer_stream cbs(speed_font_buffer,speed_font_buffer_len);
  open_font fnt;
  open_font::open(&cbs,&fnt);
  float scale = fnt.scale(80);
  char sz[5];
  strcpy(sz,speed_text(888));
  sz[3]=' ';
  sz[4]=0;
  ssize16 result= fnt.measure_text({32767,32767},{0,0},sz,scale);
  return result.inflate(2,2);
}
void calibrate(bool write=true) {
  touch.initialize();
  File file;
  if(write) {
    file = SPIFFS.open("/calibration","wb");
  }
  
  int16_t values[8];
  uint16_t x,y;
  srect16 sr(0,0,15,15);
  ssize16 ssr(8,8);
  // top left
  lcd.fill(lcd.bounds(),lcd_color::white);
  draw::filled_rectangle(lcd,ssr.bounds().offset(sr.top_left()),lcd_color::sky_blue);
  draw::filled_ellipse(lcd,sr,lcd_color::sky_blue);
  while(!touch.calibrate_touch(&x,&y)) delay(1);
  values[0]=x;values[1]=y;
  if(write) {
    file.write((uint8_t*)&x,2);
    file.write((uint8_t*)&y,2);
  }
  lcd.fill(lcd.bounds(),lcd_color::white);
  delay(1000); // debounce

  // bottom left
  sr.offset_inplace(0,lcd.dimensions().height-sr.height());
  lcd.fill(lcd.bounds(),lcd_color::white);
  draw::filled_rectangle(lcd,ssr.bounds().offset(sr.x1,sr.y1+sr.height()/2),lcd_color::sky_blue);
  draw::filled_ellipse(lcd,sr,lcd_color::sky_blue);
  while(!touch.calibrate_touch(&x,&y)) delay(1);
  values[2]=x;values[3]=y;
  if(write) {
    file.write((uint8_t*)&x,2);
    file.write((uint8_t*)&y,2);
  }
  lcd.fill(lcd.bounds(),lcd_color::white);
  delay(1000); // debounce
  
  sr=srect16(0,0,15,15);
  // top right
  sr.offset_inplace(lcd.dimensions().width-sr.width(),0);
  draw::filled_rectangle(lcd,ssr.bounds().offset(sr.x1+sr.width()/2,sr.y1),lcd_color::sky_blue);
  draw::filled_ellipse(lcd,sr,lcd_color::sky_blue);
  while(!touch.calibrate_touch(&x,&y)) delay(1);
  values[4]=x;values[5]=y;
  if(write) {
    file.write((uint8_t*)&x,2);
    file.write((uint8_t*)&y,2);
  }
  lcd.fill(lcd.bounds(),lcd_color::white);
  delay(1000); // debounce
  
  // bottom right
  sr.offset_inplace(0,lcd.dimensions().height-sr.height());
  draw::filled_rectangle(lcd,ssr.bounds().offset(sr.x1+sr.width()/2,sr.y1+sr.height()/2),lcd_color::sky_blue);
  draw::filled_ellipse(lcd,sr,lcd_color::sky_blue);
  while(!touch.calibrate_touch(&x,&y)) delay(1);
  values[6]=x;values[7]=y;
  if(write) {
    file.write((uint8_t*)&x,2);
    file.write((uint8_t*)&y,2);
  }
  lcd.fill(lcd.bounds(),lcd_color::white);
  delay(1000); // debounce
  touch.calibrate(lcd.dimensions().width,lcd.dimensions().height,values);
  if(write) {
    file.close();
  }
}
bool read_calibration() {
  if(SPIFFS.exists("/calibration")) {
    File file = SPIFFS.open("/calibration","rb");
    int16_t values[8];
    uint16_t x,y;
    for(int i = 0;i<8;i+=2) {
      if(2!=file.readBytes((char*)&x,2)) { file.close(); return false; }
      if(2!=file.readBytes((char*)&y,2)) { file.close(); return false; }
      values[i]=x;
      values[i+1]=y;
    }
    file.close();
    return touch.calibrate(lcd.dimensions().width,lcd.dimensions().height,values);
  }
  return false;
}
// variables for demo
int speed = 0;
int speed_delta = 1;
int speed_delay = 250;
uint32_t speed_ts = 0;
void setup() {
  Serial.begin(115200);
  SPIFFS.begin(false);
  // fill the screen
  lcd.fill(lcd.bounds(),lcd_color::white);

  File file = SPIFFS.open(speed_font_path,"rb");
  if(!file)  {
    Serial.printf("Asset %s not found. Halting.",speed_font_name);
    while(true) delay(1000);
  }
  // get the file length
  file.seek(0,fs::SeekMode::SeekEnd);
  size_t len = file.position();
  file.seek(0);
  if(len==0) {
    Serial.printf("Asset %s not found. Halting.",speed_font_name);
    while(true) delay(1000);
  }
  // allocate the buffer
  speed_font_buffer = (uint8_t*)ps_malloc(len);
  if(!speed_font_buffer)  {
    Serial.printf("Unable to allocate PSRAM for asset %s. Halting.",speed_font_name);
    while(true) delay(1000);
  }
  // copy the file into the buffer
  file.readBytes((char*)speed_font_buffer,len);
  // don't need the file anymore
  file.close();
  speed_font_buffer_len = len;
  // test the font to make sure it's good (avoiding checks later)
  // first wrap the buffer w/ a stream
  const_buffer_stream cbs(speed_font_buffer,speed_font_buffer_len);
  open_font fnt;
  // attempt to open the font
  gfx_result r=open_font::open(&cbs,&fnt);
  if(r!=gfx_result::success) {
    Serial.printf("Unable to load asset %s. Halting.",speed_font_name);
    while(true) delay(1000);
  }
  // get the maximum size of the speed in the current font
  speed_size = compute_speed_size();
  if(!read_calibration() || !touch.calibrated()) {
    calibrate(true);
  }
    
  // setup is complete

  
 
}

void loop() {
  if(millis()-speed_ts>=speed_delay) {
    draw_speed(speed,"mph");
    speed+=speed_delta;
    if(speed<0) {
      speed_delta=-speed_delta;
      speed=speed_delta;
    } else if(speed>200) {
      speed_delta=-speed_delta;
      speed=200-speed_delta;
    }
  }
  uint16_t x,y;
  if(touch.calibrated_xy(&x,&y)) {
    draw::filled_ellipse(lcd,srect16(spoint16(x,y),8),lcd_color::gray);
  }  
}