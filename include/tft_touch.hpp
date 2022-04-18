#pragma once
#include <Arduino.h>
#include <SPI.h>
namespace arduino {
template <int8_t PinCS>
class tft_touch {
    constexpr const static int8_t pin_cs = PinCS;
    struct calibration {
        uint16_t width;
        uint16_t height;
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
    bool m_calibrated;
    calibration m_calibration;
    uint32_t m_press_time;
    SPIClass& m_spi;
    SPISettings m_spi_settings;
    static void init_calibration(calibration* out_cal) {
        // just give it some values so it doesn't crash
        out_cal->width = 320;
        out_cal->height = 200;
        out_cal->x0 = 300;
        out_cal->x1 = 3600;
        out_cal->y0 = 300;
        out_cal->y1 = 3600;
        out_cal->rotate = 1;
        out_cal->invert_x = 2;
        out_cal->invert_y = 0;
    }
    void translate_calibrated(uint16_t* out_x, uint16_t* out_y) {
        if (!m_calibrated) return;
        uint16_t x_tmp = *out_x, y_tmp = *out_y, xx, yy;
        if (!m_calibration.rotate) {
            xx = (x_tmp - m_calibration.x0) * m_calibration.width / m_calibration.x1;
            yy = (y_tmp - m_calibration.y0) * m_calibration.height / m_calibration.y1;
            if (m_calibration.invert_x)
                xx = m_calibration.width - xx;
            if (m_calibration.invert_y)
                yy = m_calibration.height - yy;
        } else {
            xx = (y_tmp - m_calibration.x0) * m_calibration.width / m_calibration.x1;
            yy = (x_tmp - m_calibration.y0) * m_calibration.height / m_calibration.y1;
            if (m_calibration.invert_x)
                xx = m_calibration.width - xx;
            if (m_calibration.invert_y)
                yy = m_calibration.height - yy;
        }
        *out_x = xx;
        *out_y = yy;
    }
    bool touch_internal(uint16_t* x, uint16_t* y, uint16_t threshold) {
        if (!initialize()) {
            return false;
        }
        constexpr static const int _RAWERR = 20;  // Deadband error allowed in successive position samples
        uint16_t x_tmp, y_tmp, x_tmp2, y_tmp2;

        // Wait until pressure stops increasing to debounce pressure
        uint16_t z1 = 1;
        uint16_t z2 = 0;
        while (z1 > z2) {
            z2 = z1;
            raw_z(&z1);
            delay(1);
        }

        if (z1 <= threshold) return false;

        raw_xy(&x_tmp, &y_tmp);

        delay(1);  // Small delay to the next sample
        uint16_t tmp;
        raw_z(&tmp);
        if (tmp <= threshold) return false;

        delay(2);  // Small delay to the next sample
        raw_xy(&x_tmp2, &y_tmp2);

        if (abs(x_tmp - x_tmp2) > _RAWERR) return false;
        if (abs(y_tmp - y_tmp2) > _RAWERR) return false;

        *x = x_tmp;
        *y = y_tmp;

        return true;
    }
   public:
    tft_touch(SPIClass& spi) : m_initialized(false), m_calibrated(false), m_press_time(0), m_spi(spi) {
        m_spi_settings._bitOrder = MSBFIRST;
        m_spi_settings._clock = uint32_t(2.5 * 1000 * 1000);
        m_spi_settings._dataMode = SPI_MODE0;
        init_calibration(&m_calibration);
    }
    // intiailizes the touch controller - will be called automatically
    bool initialize() {
        if (!m_initialized) {
            pinMode(pin_cs, OUTPUT);
            digitalWrite(pin_cs, HIGH);
            m_spi.begin();
            m_calibrated = false;
            init_calibration(&m_calibration);
            m_press_time = 0;
            m_initialized = true;
        }
        return m_initialized;
    }
    // indicates whether or not the controller has been initialized
    inline bool initialized() const {
        return m_initialized;
    }
    // indicates whether or not the controller has been calibrated
    inline bool calibrated() const {
        return m_calibrated;
    }
    // retrieves the raw x and y valuea from the controller
    // NOTE: this always returns values, regardless of whether or not the screen is being touched
    bool raw_xy(uint16_t* out_x, uint16_t* out_y) {
        if (!initialize()) {
            return false;
        }
        uint16_t tmp;
        m_spi.beginTransaction(m_spi_settings);
        digitalWrite(pin_cs, LOW);
        m_spi.transfer(0xd0);  // Start new YP conversion
        m_spi.transfer(0);     // Read first 8 bits
        m_spi.transfer(0xd0);  // Read last 8 bits and start new YP conversion
        m_spi.transfer(0);     // Read first 8 bits
        m_spi.transfer(0xd0);  // Read last 8 bits and start new YP conversion
        m_spi.transfer(0);     // Read first 8 bits
        m_spi.transfer(0xd0);  // Read last 8 bits and start new YP conversion

        tmp = m_spi.transfer(0);  // Read first 8 bits
        tmp = tmp << 5;
        tmp |= 0x1f & (m_spi.transfer(0x90) >> 3);  // Read last 8 bits and start new XP conversion
        *out_x = tmp;
        m_spi.transfer(0);     // Read first 8 bits
        m_spi.transfer(0x90);  // Read last 8 bits and start new XP conversion
        m_spi.transfer(0);     // Read first 8 bits
        m_spi.transfer(0x90);  // Read last 8 bits and start new XP conversion
        m_spi.transfer(0);     // Read first 8 bits
        m_spi.transfer(0x90);  // Read last 8 bits and start new XP conversion

        tmp = m_spi.transfer(0);  // Read first 8 bits
        tmp = tmp << 5;
        tmp |= 0x1f & (m_spi.transfer(0) >> 3);  // Read last 8 bits
        *out_y = tmp;

        m_spi.endTransaction();
        digitalWrite(pin_cs, HIGH);
        return true;
    }
    // retrieves the raw z value from the controller
    bool raw_z(uint16_t* out_z) {
        if (!initialize()) {
            return false;
        }
        m_spi.beginTransaction(m_spi_settings);
        digitalWrite(pin_cs, LOW);
        int16_t tmp = 0xFFF;
        m_spi.transfer(0xB0);
        tmp += m_spi.transfer16(0xC0) >> 3;
        tmp -= m_spi.transfer16(0) >> 3;
        m_spi.endTransaction();
        digitalWrite(pin_cs, HIGH);
        *out_z = (uint16_t)tmp;
        return true;
    }
    // retrieves the calibrated x and y values if the screen was depressed by the specified threshold
    // returns true if the screen was touched, or false if it wasn't
    bool calibrated_xy(uint16_t* out_x, uint16_t* out_y, uint16_t threshold=z_threshold) {
        if(!initialize()) {
            return false;
        }
        if(!m_calibrated) {
            return false;
        }
        
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

        translate_calibrated(&x_tmp, &y_tmp);

        if (x_tmp >= m_calibration.width || y_tmp >= m_calibration.height) {
            return false;
        }

        *out_x = x_tmp;
        *out_y = y_tmp;
        return valid;
    }
    // retrieves a semi-debounced sampled average of the touch coordinates, uncalibrated
    // used for touch calibration
    bool calibrate_touch(uint16_t* out_x, uint16_t* out_y,size_t attempts = 8, uint16_t threshold = z_threshold / 2) {
        if(!initialize()) {
            return false;
        }
        uint16_t x_tmp, y_tmp;
        uint16_t xa=0,ya=0;
        // average 8 samples
        for (uint8_t j = 0; j < attempts; j++) {
            // Use a lower detect threshold as corners tend to be less sensitive
            while (!touch_internal(&x_tmp, &y_tmp, threshold))
                ;
            xa += x_tmp;
            ya += y_tmp;
        }
        *out_x = (int)(xa / attempts);
        *out_y = (int)(ya / attempts);
        return true;
    }
    // Calibrates the screen using the indicated touch values
    // Values are x,y|x,y|x,y|x,y top-left, top-right, bottom-right, bottom-left
    // (clockwise from top-left). Touch values retrieved from using calibrate_touch()
    // to sample the points at the corners of the screeen
    bool calibrate(uint16_t screen_width,uint16_t screen_height,int16_t* values) {
        
        /* TFT_eSPI order
        constexpr static const size_t top_left_x = 0;
        constexpr static const size_t top_left_y = 1;
        constexpr static const size_t bottom_left_x = 2;
        constexpr static const size_t bottom_left_y = 3;
        constexpr static const size_t top_right_x = 4;
        constexpr static const size_t top_right_y = 5;
        constexpr static const size_t bottom_right_x = 6;
        constexpr static const size_t bottom_right_y = 7;
        */
        // clockwise from top-left order
        constexpr static const size_t top_left_x = 0;
        constexpr static const size_t top_left_y = 1;
        constexpr static const size_t top_right_x = 2;
        constexpr static const size_t top_right_y = 3;
        constexpr static const size_t bottom_right_x = 4;
        constexpr static const size_t bottom_right_y = 5;
        constexpr static const size_t bottom_left_x = 6;
        constexpr static const size_t bottom_left_y = 7;
        
        if (values == nullptr || screen_width == 0 || screen_height==0) {
            return false;
        }
        if (!initialize()) {
            return false;
        }
        m_calibration.width = screen_width;
        m_calibration.height = screen_height;
        m_calibration.rotate = false;
        if (abs(values[top_left_x] - values[bottom_left_x]) > abs(values[top_left_y] - values[bottom_left_y])) {
            m_calibration.rotate = true;
            m_calibration.x0 = (values[top_left_y] + values[bottom_left_y]) / 2;  // calc min x
            m_calibration.x1 = (values[top_right_y] + values[bottom_right_y]) / 2;  // calc max x
            m_calibration.y0 = (values[top_left_x] + values[top_right_x]) / 2;  // calc min y
            m_calibration.y1 = (values[bottom_left_x] + values[bottom_right_x]) / 2;  // calc max y
        } else {
            m_calibration.x0 = (values[top_left_x] + values[bottom_left_x]) / 2;  // calc min x
            m_calibration.x1 = (values[top_right_x] + values[bottom_right_x]) / 2;  // calc max x
            m_calibration.y0 = (values[top_left_y] + values[top_right_y]) / 2;  // calc min y
            m_calibration.y1 = (values[bottom_left_y] + values[bottom_right_y]) / 2;  // calc max y
        }

        // in addition, the touch screen axis could be in the opposite direction of the TFT axis
        m_calibration.invert_x = false;
        if (m_calibration.x0 > m_calibration.x1) {
            values[top_left_x] = m_calibration.x0;
            m_calibration.x0 = m_calibration.x1;
            m_calibration.x1 = values[top_left_x];
            m_calibration.invert_x = true;
        }
        m_calibration.invert_y = false;
        if (m_calibration.y0 > m_calibration.y1) {
            values[top_left_x] = m_calibration.y0;
            m_calibration.y0 = m_calibration.y1;
            m_calibration.y1 = values[top_left_x];
            m_calibration.invert_y = true;
        }

        // pre calculate
        m_calibration.x1 -= m_calibration.x0;
        m_calibration.y1 -= m_calibration.y0;

        if (m_calibration.x0 == 0) m_calibration.x0 = 1;
        if (m_calibration.x1 == 0) m_calibration.x1 = 1;
        if (m_calibration.y0 == 0) m_calibration.y0 = 1;
        if (m_calibration.y1 == 0) m_calibration.y1 = 1;

        m_calibrated = true;
        return true;
    }
};
}  // namespace arduino