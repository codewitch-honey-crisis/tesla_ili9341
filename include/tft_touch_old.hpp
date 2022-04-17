#pragma once
#include <SPI.h>

#include <io_stream.hpp>
namespace arduino {
template <typename Bus, unsigned int TouchSpeedPercent = 20>
struct tft_touch {
    using bus_type = Bus;
    constexpr static const float touch_speed_multiplier = TouchSpeedPercent / 100.0;

   private:
    struct calibration {
        uint16_t x0;
        uint16_t x1;
        uint16_t y0;
        uint16_t y1;
        uint8_t rotate;
        uint8_t invert_x;
        uint8_t invert_y;
    };
    constexpr static const int z_threshold = 350;
    bool m_initialized;
    SPIClass m_spi;
    SPISettings m_spi_settings;
    calibration m_calibration;
    bool m_calibrated;
    uint32_t m_press_time;
    static void init_calibration(calibration* out_cal) {
        out_cal->x0 = 300;
        out_cal->x1 = 3600;
        out_cal->y0 = 300;
        out_cal->y1 = 3600;
        out_cal->rotate = 1;
        out_cal->invert_x = 2;
        out_cal->invert_y = 0;
    }

   public:
    struct point {
        uint16_t x;
        uint16_t y;
    };
    tft_touch() : m_initialized(false),m_spi(bus_type::spi_host), m_calibrated(false), m_press_time(0) {
    }
    inline bool initialized() const {
        return m_initialized;
    }
    bool initialize() {
        if (!m_initialized) {
            m_spi_settings._bitOrder = MSBFIRST;
            m_spi_settings._clock = uint32_t((touch_speed_multiplier * 10.0) * 1000 * 1000);
            m_spi_settings._dataMode = SPI_MODE0;
            pinMode(bus_type::pin_cs,OUTPUT);
            digitalWrite(bus_type::pin_cs,HIGH);
            
            if (!bus_type::initialize()) {
                return false;
            }
            m_spi.begin();
            m_calibrated = false;
            init_calibration(&m_calibration);
            m_press_time = 0;
            m_initialized = true;
        }
        return true;
    }
    void deinitialize() {
        if (m_initialized) {
            bus_type::deinitialize();
            m_initialized = false;
        }
    }
    inline bool calibrated() const {
        return m_calibrated;
    }

   private:
    inline void begin_touch() {
        // bus_type::dma_wait();
        // bus_type::set_speed_multiplier(touch_speed_multiplier);
        //bus_type::cs_high();
        // bus_type::begin_write();
        
        
        m_spi.beginTransaction(m_spi_settings);
        digitalWrite(bus_type::pin_cs,LOW);
    }
    inline void end_touch() {
        m_spi.endTransaction();
        digitalWrite(bus_type::pin_cs,HIGH);
        // bus_type::end_write();
        
    }
    bool touch_point(uint16_t* out_x, uint16_t* out_y) {
        begin_touch();
        /*bus_type::write_raw8(0xD0);
        bus_type::write_raw8(0);
        bus_type::write_raw8(0xD0);
        bus_type::write_raw8(0);
        bus_type::write_raw8(0xD0);
        bus_type::write_raw8(0);
        bus_type::write_raw8(0xD0);
        */
        m_spi.transfer(0xD0);
        m_spi.transfer(0);
        m_spi.transfer(0xD0);
        m_spi.transfer(0);
        m_spi.transfer(0xD0);
        m_spi.transfer(0);
        m_spi.transfer(0xD0);
        // uint8_t tmp = bus_type::read_raw8();
        uint8_t tmp = m_spi.transfer(0);
        tmp=tmp << 5;
        // tmp|=0x1F & (bus_type::read_write_raw8(0x90)>>3);
        tmp |= 0x1F & (m_spi.transfer(0x90) >> 3);
        *out_x = tmp;
        /*bus_type::write_raw8(0);
        bus_type::write_raw8(0x90);
        bus_type::write_raw8(0);
        bus_type::write_raw8(0x90);
        bus_type::write_raw8(0);
        bus_type::write_raw8(0x90);
        */
        m_spi.transfer(0);
        m_spi.transfer(0x90);
        m_spi.transfer(0);
        m_spi.transfer(0x90);
        m_spi.transfer(0);
        m_spi.transfer(0x90);
        // tmp = bus_type::read_raw8();
        tmp = m_spi.transfer(0);
        tmp =tmp<< 5;
        tmp |= 0x1F & (m_spi.transfer(0) >> 3);
        *out_y = tmp;
        end_touch();
        return true;
    }
    bool touch_pressure(uint16_t* out_z) {
        begin_touch();
        int16_t tmp = 0xFFF;
        // bus_type::write_raw8(0xB0);
        m_spi.transfer(0xB0);
        // tmp+=int16_t(bus_type::read_write_raw16(0xC0))>>3;
        tmp += m_spi.transfer16(0xC0) >> 3;
        // tmp-=int16_t(bus_type::read_write_raw16(0))>>3;
        tmp -= m_spi.transfer16(0) >> 3;
        end_touch();
        *out_z = (uint16_t)tmp;
        return true;
    }
    bool touch_internal(uint16_t* x, uint16_t* y, uint16_t threshold) {
        initialize();
        constexpr static const int _RAWERR = 20;  // Deadband error allowed in successive position samples
        uint16_t x_tmp, y_tmp, x_tmp2, y_tmp2;

        // Wait until pressure stops increasing to debounce pressure
        uint16_t z1 = 1;
        uint16_t z2 = 0;
        while (z1 > z2) {
            z2 = z1;
            touch_pressure(&z1);
            delay(1);
        }

        if (z1 <= threshold) return false;

        touch_point(&x_tmp, &y_tmp);

        delay(1);  // Small delay to the next sample
        uint16_t tmp;
        touch_pressure(&tmp);
        if (tmp <= threshold) return false;

        delay(2);  // Small delay to the next sample
        touch_point(&x_tmp2, &y_tmp2);

        if (abs(x_tmp - x_tmp2) > _RAWERR) return false;
        if (abs(y_tmp - y_tmp2) > _RAWERR) return false;

        *x = x_tmp;
        *y = y_tmp;

        return true;
    }
    void translate_calibrated(uint16_t screen_width, uint16_t screen_height, uint16_t* out_x, uint16_t* out_y) {
        if (!m_calibrated) return;
        uint16_t x_tmp = *out_x, y_tmp = *out_y, xx, yy;
        if (!m_calibration.rotate) {
            xx = (x_tmp - m_calibration.x0) * screen_width / m_calibration.x1;
            yy = (y_tmp - m_calibration.y0) * screen_height / m_calibration.y1;
            if (m_calibration.invert_x)
                xx = screen_width - xx;
            if (m_calibration.invert_y)
                yy = screen_height - yy;
        } else {
            xx = (y_tmp - m_calibration.x0) * screen_width / m_calibration.x1;
            yy = (x_tmp - m_calibration.y0) * screen_height / m_calibration.y1;
            if (m_calibration.invert_x)
                xx = screen_width - xx;
            if (m_calibration.invert_y)
                yy = screen_height - yy;
        }
        *out_x = xx;
        *out_y = yy;
    }

   public:
    bool read_calibration(io::stream* calibration_stream) {
        if (calibration_stream->caps().read && sizeof(calibration) == calibration_stream->read((uint8_t*)m_calibration, sizeof(calibration))) {
            m_calibrated = true;
            return true;
        }
        // error
        init_calibration(m_calibration);
        m_calibrated = false;
        return false;
    }
    bool write_calibration(io::stream* calibration_stream) {
        if (m_calibrated && calibration_stream->caps().write && sizeof(calibration) == calibration_stream->write((uint8_t*)m_calibration, sizeof(calibration))) {
            return true;
        }
        return false;
    }
    bool calibrate_touch(uint16_t* out_x, uint16_t* out_y) {
        uint16_t x_tmp, y_tmp;
        // average 8 samples
        for (uint8_t j = 0; j < 8; j++) {
            // Use a lower detect threshold as corners tend to be less sensitive
            while (!touch_internal(&x_tmp, &y_tmp, z_threshold / 2))
                ;
            x_tmp += x_tmp;
            y_tmp += y_tmp;
        }
        *out_x = (int)(x_tmp / 8.0 + .5);
        *out_y = (int)(y_tmp / 8.0 + .5);
    }
    // values are x,y|x,y|x,y|x,y top-left, bottom-left, top-right, bottom-right
    // TODO: should be clockwise
    bool calibrate(uint16_t* values) {
        constexpr static const size_t top_left_x = 0;
        constexpr static const size_t top_left_y = 1;
        constexpr static const size_t bottom_left_x = 2;
        constexpr static const size_t bottom_left_y = 3;
        constexpr static const size_t top_right_x = 4;
        constexpr static const size_t top_right_y = 5;
        constexpr static const size_t bottom_right_x = 6;
        constexpr static const size_t bottom_right_y = 7;

        if (values == nullptr) {
            return false;
        }
        if(!initialize()) {
            return false;
        }
          m_calibration.rotate = false;
  if(abs(values[0]-values[2]) > abs(values[1]-values[3])){
    m_calibration.rotate = true;
    m_calibration.x0 = (values[1] + values[3])/2; // calc min x
    m_calibration.x1 = (values[5] + values[7])/2; // calc max x
    m_calibration.y0 = (values[0] + values[4])/2; // calc min y
    m_calibration.y1 = (values[2] + values[6])/2; // calc max y
  } else {
    m_calibration.x0 = (values[0] + values[2])/2; // calc min x
    m_calibration.x1 = (values[4] + values[6])/2; // calc max x
    m_calibration.y0 = (values[1] + values[5])/2; // calc min y
    m_calibration.y1 = (values[3] + values[7])/2; // calc max y
  }

  // in addition, the touch screen axis could be in the opposite direction of the TFT axis
  m_calibration.invert_x = false;
  if(m_calibration.x0 > m_calibration.x1){
    values[0]=m_calibration.x0;
    m_calibration.x0 = m_calibration.x1;
    m_calibration.x1 = values[0];
    m_calibration.invert_x = true;
  }
  m_calibration.invert_y = false;
  if(m_calibration.y0 > m_calibration.y1){
    values[0]=m_calibration.y0;
    m_calibration.y0 = m_calibration.y1;
    m_calibration.y1 = values[0];
    m_calibration.invert_y = true;
  }

  // pre calculate
  m_calibration.x1 -= m_calibration.x0;
  m_calibration.y1 -= m_calibration.y0;

  if(m_calibration.x0 == 0) m_calibration.x0 = 1;
  if(m_calibration.x1 == 0) m_calibration.x1 = 1;
  if(m_calibration.y0 == 0) m_calibration.y0 = 1;
  if(m_calibration.y1 == 0) m_calibration.y1 = 1;

        m_calibrated = true;
        return true;
    }
    bool raw_touch(uint16_t* out_x, uint16_t* out_y, uint16_t* out_z) {
        if(!initialize()) 
            return false;
        
        if (!touch_point(out_x, out_y)) {
             return false;
        }
        if (!touch_pressure(out_z))
            return false;

        return true;
    }
    uint8_t touch(uint16_t screen_width,uint16_t screen_height, uint16_t* out_x, uint16_t* out_y, uint16_t threshold=z_threshold) {
        uint16_t x_tmp, y_tmp;

        if (threshold < 20) threshold = 20;
        if (m_press_time > millis()) threshold = 20;

        uint8_t n = 5;
        uint8_t valid = 0;
        while (n--) {
            if (touch_internal(&x_tmp, &y_tmp, threshold)) valid++;
            ;
        }

        if (valid < 1) {
            m_press_time = 0;
            return false;
        }

        m_press_time = millis() + 50;

        //translate_calibrated(screen_width,screen_height, &x_tmp, &y_tmp);

        if (x_tmp >= screen_width || y_tmp >= screen_height) {
            Serial.printf("Out of bounds: x = %d, y = %d\n",(int)x_tmp,(int)y_tmp);
            return false;
        }

        *out_x = x_tmp;
        *out_y = y_tmp;
        return valid;
    }
};
}  // namespace arduino