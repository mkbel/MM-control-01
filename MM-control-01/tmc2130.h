//tmc2130.h - Trinamic stepper driver
#ifndef _TMC2130_H
#define _TMC2130_H

#include <inttypes.h>

#define TMC2130_SG_THR         4       // SG_THR default
#define TMC2130_TCOOLTHRS      450     // TCOOLTHRS default

#define TMC2130_CHECK_SPI 0x01
#define TMC2130_CHECK_MSC 0x02
#define TMC2130_CHECK_MOC 0x04
#define TMC2130_CHECK_STP 0x08
#define TMC2130_CHECK_DIR 0x10
#define TMC2130_CHECK_ENA 0x20
#define TMC2130_CHECK_OK  0x3f

enum class Mode : uint8_t
{
    Homing,
    Normal,
    Stealth,
};

struct Modes
{
    Mode pulley;
    Mode selector;
    Mode idler;
};

constexpr Modes Homing_modes = {Mode::Homing, Mode::Homing, Mode::Homing};
constexpr Modes Normal_modes = {Mode::Normal, Mode::Normal, Mode::Normal};
constexpr Modes Stealth_modes = {Mode::Normal, Mode::Stealth, Mode::Stealth};

inline bool operator== (Modes a, Modes b)
{
    static_assert(Homing_modes.idler != Normal_modes.idler, "");
    static_assert(Normal_modes.idler != Stealth_modes.idler, "");
    return ( a.idler == b.idler);
}

extern int8_t tmc2130_init(Modes modes);

extern int8_t tmc2130_init_axis(uint8_t axis, Mode mode);
extern int8_t tmc2130_init_axis_current_normal(uint8_t axis, uint8_t current_h, uint8_t current_r, bool enable = true);
extern int8_t tmc2130_init_axis_current_stealth(uint8_t axis, uint8_t current_h, uint8_t current_r, bool enable = true);
extern void tmc2130_disable_axis(uint8_t axis, Mode mode);

extern uint8_t tmc2130_check_axis(uint8_t axis);

extern uint16_t tmc2130_read_sg(uint8_t axis);
extern uint8_t tmc2130_read_gstat();


#endif //_TMC2130_H
