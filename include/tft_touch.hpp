#pragma once
namespace arduino {
    template<typename Bus,unsigned int TouchSpeedPercent=100>
    struct tft_touch {
        using bus_type = Bus;
        constexpr static const float touch_speed_multiplier = TouchSpeedPercent/100.0;
    private:
        bool m_initialized;
    public:
        inline bool initialized() const {
            return m_initialized;
        }
        bool initialize() {
            if(!m_initialized) {
                if(!bus_type::initialize()) {
                    return false;
                }
                m_initialized = true;
            }
            return true;
        }
        void deinitialize() {
            if(m_initialized) {
                bus_type::deinitialize();
            }
        }
    private:
        static inline void begin_touch() {
            bus_type::dma_wait();
            bus_type::set_speed_multiplier(touch_speed_multiplier);
            bus_type::cs_low();
            bus_type::begin_write();
        }
        static inline void end_touch() {
            bus_type::end_write();
            bus_type::cs_high();
        }
        static bool touch_point(uint16_t* out_x, uint16_t* out_y) {
            begin_touch();
            bus_type::write_raw8(0xD0);
            bus_type::write_raw8(0);
            bus_type::write_raw8(0xD0);
            bus_type::write_raw8(0);
            bus_type::write_raw8(0xD0);
            bus_type::write_raw8(0);
            bus_type::write_raw8(0xD0);
            uint8_t tmp = bus_type::read_raw8();
            tmp<<=5;
            tmp|=0x1F & (bus_type::read_write_raw8(0x90)>>3);
            *out_x=tmp;
            bus_type::write_raw8(0);
            bus_type::write_raw8(0x90);
            bus_type::write_raw8(0);
            bus_type::write_raw8(0x90);
            bus_type::write_raw8(0);
            bus_type::write_raw8(0x90);
            tmp = bus_type::read_raw8();
            tmp<<=5;
            tmp|=0x1F & (bus_type::read_raw8()>>3);
            *out_y=tmp;
            end_touch();
            return true;
        }
        static bool touch_pressure(uint16_t* out_z) {
            begin_touch();
            int16_t tmp = 0xFFF;
            bus_type::write_raw8(0xB0);
            tmp+=int16_t(bus_type::read_write_raw16(0xC0))>>3;
            tmp-=int16_t(bus_type::read_write_raw16(0))>>3;
            end_touch();
            *out_z = (uint16_t)tmp;
            return true;
        }
    public:
        bool raw_touch(uint16_t* out_x, uint16_t* out_y, uint16_t *out_z) {
            initialize();
            if(touch_pressure(out_z)) {
                if(touch_point(out_x,out_y)) {
                    return true;
                }
            }
            return false;
        }
    };
}