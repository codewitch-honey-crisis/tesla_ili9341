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
// if you change the font, you'll have to tweak
// the code in draw_speed() to place it properly.
const char* speed_font_path = "/Telegrama.otf";//"/Bungee.otf"; // "/Ubuntu.otf"
const char* speed_font_name = speed_font_path+1;
uint8_t* speed_font_buffer;
size_t speed_font_buffer_len;
ssize16 speed_size;
char speed_buf[16];

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
  static const pt unit_col = color<pt>::red;
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
  draw::text(tmp,srect16(speed_size.width-spd_size.width,0,tmp.bounds().x2,tmp.bounds().y2),{0,0},speed_buf,fnt,spd_scale,speed_col);
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

// variables for demo
int speed = 0;
int speed_delta = 1;
int speed_delay = 25;

void setup() {
  Serial.begin(115200);
  SPIFFS.begin(false);
  File file = SPIFFS.open(speed_font_path,"rb");
  if(!file)  {
    Serial.printf("Asset %s not found. Halting.",speed_font_name);
    while(true) delay(1000);
  }
  // get the file length
  file.seek(0,fs::SeekMode::SeekEnd);
  size_t len = file.position();
  file.seek(0);
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
  // setup is complete

  // fill the screen
  lcd.fill(lcd.bounds(),lcd_color::white);
  
}

void loop() {
  draw_speed(speed,"mph");
  speed+=speed_delta;
  if(speed<0) {
    speed_delta=-speed_delta;
    speed=speed_delta;
  } else if(speed>200) {
    speed_delta=-speed_delta;
    speed=200-speed_delta;
  }
  delay(speed_delay);
}