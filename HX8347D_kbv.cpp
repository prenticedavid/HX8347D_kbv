#if defined(AVR)
#include "HX8347D_kbv.h"

#if USE_MBED
#define CS_IDLE       { _lcd_pin_cs = 1; }
#define CS_ACTIVE     _lcd_pin_cs = 0
#define CD_DATA       { _lcd_pin_rs = 1; }
#define CD_COMMAND    _lcd_pin_rs = 0
#define RESET_IDLE    _lcd_pin_reset = 1
#define RESET_ACTIVE  _lcd_pin_reset = 0
#define xchg8(x)     _spi.write((uint8_t)x)
#define write8(x)    { _spi.write((uint8_t)x); }
#define write16(x)   { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define WriteCmd(x)  { CD_COMMAND; write8(x); CD_DATA; } //check flush()
#define WriteData(x) { write8(x); }

HX8347D_kbv::HX8347D_kbv(PinName CS, PinName RS, PinName RST)
    : _lcd_pin_rs(RS), _lcd_pin_cs(CS), _lcd_pin_reset(RST), _spi(D11, D12, D13), Adafruit_GFX(240, 320)
{
    _spi.format(8, 0);
    _spi.frequency(12000000);
    CS_IDLE;
    RESET_IDLE;
}
#else
#if 1
#include <avr/pgmspace.h>
#if 0
#elif defined(__AVR_ATmega328P__)
#define CD_PORT PORTD
#define CD_PIN  PD7      //digital#7
#define CS_PORT PORTB
#define CS_PIN  PB2      //digital#10
#define RESET_PORT PORTB
#define RESET_PIN  PB1   //Backlight //digital#9
#define SD_PORT PORTD
#define SD_PIN  PD5      //digital#5
#define XPT_PORT PORTD
#define XPT_PIN  PD4      //digital#4
#define SPI_SS   PB2
#define SPI_SCK  PB5
#define SPI_MOSI PB3
#elif defined(__AVR_ATmega32U4__)
#define CD_PORT PORTE
#define CD_PIN  PE6      //digital#7
#define CS_PORT PORTB
#define CS_PIN  PB6      //digital#10
#define RESET_PORT PORTB
#define RESET_PIN  PB5   //Backlight //digital#9
#define SD_PORT PORTC
#define SD_PIN  PC6      //digital#5
#define XPT_PORT PORTD
#define XPT_PIN  PD4      //digital#4
#define SPI_SS   PB0
#define SPI_SCK  PB1
#define SPI_MOSI PB2
#elif defined(__AVR_ATmega2560__)
#define CD_PORT PORTH
#define CD_PIN  PH4      //digital#7
#define CS_PORT PORTB
#define CS_PIN  PB4      //digital#10
#define RESET_PORT PORTH
#define RESET_PIN  PH6   //Backlight //digital#9
#define SD_PORT PORTE
#define SD_PIN  PE3      //digital#5
#define XPT_PORT PORTG
#define XPT_PIN  PG5      //digital#4
#define SPI_SS   PB0
#define SPI_SCK  PB1
#define SPI_MOSI PB2
#elif defined(__AVR_ATmega4809__)  // Arduino NANO-EVERY
#define CD_PORT VPORTA_OUT
#define CD_PIN  1 //PA1      //digital#7
#define CS_PORT VPORTB_OUT
#define CS_PIN  1 //PB1      //digital#10
#define RESET_PORT VPORTB_OUT
#define RESET_PIN  0 //PB0   //Backlight //digital#9
#define SD_PORT VPORTB_OUT
#define SD_PIN  2 //PB2      //digital#5
#define XPT_PORT VPORTC_OUT
#define XPT_PIN  6 //PC6      //digital#4
#define SPI_SS   1 //PB1
#define SPI_SCK  2 //PE2
#define SPI_MOSI 0 //PE0
#else
#error
#endif

#define SETDDR  { CS_OUTPUT; RESET_OUTPUT; CD_OUTPUT; SD_OUTPUT; XPT_OUTPUT; }
#define INIT()  { CS_IDLE; RESET_IDLE; SD_IDLE; XPT_IDLE; SETDDR; spi_init(); }

#if defined(__AVR_ATmega4809__)
#define RESTORE_SPI() { SPI0_CTRLA = SPI_MASTER_bm | SPI_ENABLE_bm; }
static inline void spi_init(void)
{
    PORTMUX.TWISPIROUTEA = (PORTMUX.TWISPIROUTEA & ~PORTMUX_SPI0_gm) | PORTMUX_SPI0_ALT2_gc;
    PORTB_DIR |= (1 << SPI_SS);
    PORTE_DIR |= (1 << SPI_SCK) | (1 << SPI_MOSI);
    SPI0_CTRLA = SPI_MASTER_bm | SPI_ENABLE_bm;
    SPI0_CTRLB = SPI_MODE_0_gc | (1 << SPI_SSD_bp); //mode#0
}
static inline void write8(uint8_t x)   {
    SPI0_DATA = x;
    while ((SPI0_INTFLAGS & 0x80) == 0);
}
static inline uint8_t read8(void)      {
    while ((SPI0_INTFLAGS & 0x80) == 0);
    return SPI0_DATA;
}
static inline uint8_t xchg8(uint8_t x) {
    write8(x);
    return read8();
}
static inline void flush(void)         { }
static uint8_t running;
#else
#define SPCRVAL ((1<<SPE)|(1<<MSTR)|(0<<CPHA)|(0<<SPR0))
#define RESTORE_SPI() { SPCR = SPCRVAL; SPSR = (1<<SPI2X); }
static inline void spi_init(void)
{
    PORTB |= (1 << SPI_SS);
    DDRB |= (1 << SPI_SS) | (1 << SPI_SCK) | (1 << SPI_MOSI);
    SPCR = SPCRVAL;
    SPSR = (1 << SPI2X);
    SPSR;
    SPDR;
}
static inline void write8(uint8_t x)   {
    SPDR = x;
    while ((SPSR & 0x80) == 0);
}
static inline uint8_t read8(void)      {
    while ((SPSR & 0x80) == 0);
    return SPDR;
}
static inline uint8_t xchg8(uint8_t x) {
    write8(x);
    return read8();
}
static inline void flush(void)         { }
static uint8_t running;
//extern uint8_t running;
#endif

#if 0
//extern uint8_t running;
static uint8_t running;
#define write8(x)    {if (running) {while ((SPSR & 0x80) == 0);SPDR;}SPDR = x;running = 1;}
#define flush()      {if (running) {while ((SPSR & 0x80) == 0);}running = 0;SPDR;}
static uint8_t read8(void)    {
    flush();
    return SPDR;
}
static uint8_t xchg8(uint8_t x) {
    write8(x);
    return read8();
}
#endif

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))

#else
#include "tftspi_keil.h"
#undef CD_PORT
#undef CD_PIN
#undef RESET_PORT
#undef RESET_PIN
#define CD_PORT GPIOA
#define CD_PIN  8
#define RESET_PORT GPIOC
#define RESET_PIN  9
#endif

#define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
//#define CS_ACTIVE  { SPCR = SPCRVAL; SPSR = (1<<SPI2X); running = 0; PIN_LOW(CS_PORT, CS_PIN); }
//#define CS_ACTIVE  { RESTORE_SPI(); running = 0; PIN_LOW(CS_PORT, CS_PIN); }
#define CS_ACTIVE  { running = 0; PIN_LOW(CS_PORT, CS_PIN); }
#define CS_IDLE    { flush(); PIN_HIGH(CS_PORT, CS_PIN); }
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)
#define SD_ACTIVE  PIN_LOW(SD_PORT, SD_PIN)
#define SD_IDLE    PIN_HIGH(SD_PORT, SD_PIN)
#define SD_OUTPUT  PIN_OUTPUT(SD_PORT, SD_PIN)
#define XPT_ACTIVE  PIN_LOW(XPT_PORT, XPT_PIN)
#define XPT_IDLE    PIN_HIGH(XPT_PORT, XPT_PIN)
#define XPT_OUTPUT  PIN_OUTPUT(XPT_PORT, XPT_PIN)

//uint8_t running = 0;

#define write16(x)   { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define WriteCmd(x)  { flush(); CD_COMMAND; write8(x); flush(); CD_DATA; }
#define WriteData(x) { write8(x); }

#define wait_ms(ms)  delay(ms)

HX8347D_kbv::HX8347D_kbv(): Adafruit_GFX(240, 320)
{
    INIT();
    CS_IDLE;
    RESET_IDLE;
    _lcd_ID = 0x7575;
}
#endif

void HX8347D_kbv::reset(void)
{
    wait_ms(50);
    RESET_ACTIVE;
    wait_ms(100);
    RESET_IDLE;
    wait_ms(100);
}

void HX8347D_kbv::WriteCmdData(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

#define WriteCmdDataPair(c, d) { WriteCmdData(c, (d)>>8); WriteCmdData(c+1, (d)&0xFF); }
#define HX8347G_COLADDRSTART_HI    0x02
#define HX8347G_COLADDRSTART_LO    0x03
#define HX8347G_COLADDREND_HI      0x04
#define HX8347G_COLADDREND_LO      0x05
#define HX8347G_ROWADDRSTART_HI    0x06
#define HX8347G_ROWADDRSTART_LO    0x07
#define HX8347G_ROWADDREND_HI      0x08
#define HX8347G_ROWADDREND_LO      0x09
#define HX8347G_MEMACCESS          0x16
#define HX8347G_MEMWRITE           0x22
#define HX8347G_MEMREAD            0x22

uint16_t HX8347D_kbv::readReg(uint16_t reg)
{
    return 0;
}

int16_t HX8347D_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    return -1;          // .kbv HX8347-D has IM=0110. HX8347-G is r/w IM=1110 (4-wire Serial II)
}

void HX8347D_kbv::setRotation(uint8_t r)
{
    uint16_t mac = 0x0800;
    Adafruit_GFX::setRotation(r & 3);
    switch (rotation) {
        case 0:
            mac = 0x0800;   // BGR=1
            break;
        case 1:
            mac = 0x6800;   //MY=0, MX=1, MV=0, ML=0, BGR=1
            break;
        case 2:
            mac = 0xD800;   //MY=1, MX=1, MV=0, ML=1, BGR=1
            break;
        case 3:
            mac = 0xB800;   //MY=1, MX=0, MV=1, ML=1, BGR=1
            break;
    }
    WriteCmdData(HX8347G_MEMACCESS, mac >> 8);
}

void HX8347D_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= width() || y >= height()) return;
    WriteCmdDataPair(HX8347G_COLADDRSTART_HI, x);
    WriteCmdDataPair(HX8347G_ROWADDRSTART_HI, y);
    CS_ACTIVE;
    WriteCmd(HX8347G_MEMWRITE);
    write16(color);
    CS_IDLE;
}

void HX8347D_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
    CS_ACTIVE;
    WriteCmdDataPair(HX8347G_COLADDRSTART_HI, x);
    WriteCmdDataPair(HX8347G_COLADDREND_HI, x1);
    WriteCmdDataPair(HX8347G_ROWADDRSTART_HI, y);
    WriteCmdDataPair(HX8347G_ROWADDREND_HI, y1);
    CS_IDLE;
}

#define NOP             asm("nop");
void HX8347D_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
    if (w < 0) {
        w = -w;    //+ve w
        x -= w;
    }
    end = x + w;
    if (x < 0) x = 0;
    if (end > width()) end = width();
    w = end - x;
    if (h < 0) {
        h = -h;    //+ve h
        y -= h;
    }
    end = y + h;
    if (y < 0) y = 0;
    if (end > height()) end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(HX8347G_MEMWRITE);
#if 0
    uint8_t hi = color >> 8;
    uint8_t lo = color;
    int32_t cnt = w * h - 1;
    SPDR = hi;
    while (cnt-- > 0) {
        while ((SPSR & 0x80) == 0);
        SPDR;
        SPDR = lo;
        while ((SPSR & 0x80) == 0);
        SPDR;
        SPDR = hi;
    }
    while ((SPSR & 0x80) == 0);       
    SPDR;
    SPDR = lo;
    while ((SPSR & 0x80) == 0);
    SPDR;
#else
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    uint8_t hi = color >> 8;
    uint8_t lo = color;
    while (h-- > 0) {
        for (int16_t i = w; i-- > 0; ) {
            SPDR = hi; NOP; while ((SPSR & 0x80) == 0);
            SPDR = lo; NOP; while ((SPSR & 0x80) == 0);
            //write8(hi);
            //NOP;
            //write8(lo);
            //NOP; NOP;
        }
    }
#endif
    CS_IDLE;
    setAddrWindow(0, 0, width() - 1, height() - 1);
}

void HX8347D_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
    uint16_t color;
    CS_ACTIVE;
    if (first) {
        WriteCmd(HX8347G_MEMWRITE);
    }
    while (n-- > 0) {
        color = *block++;
        write16(color);
    }
    CS_IDLE;
}

void HX8347D_kbv::pushColors(const uint8_t * block, int16_t n, bool first)
{
    uint16_t color;
    uint8_t h, l;
    CS_ACTIVE;
    if (first) {
        WriteCmd(HX8347G_MEMWRITE);
    }
    while (n-- > 0) {
        l = pgm_read_byte(block++);
        h = pgm_read_byte(block++);
        color = h << 8 | l;
        write16(color);
    }
    CS_IDLE;
}

void HX8347D_kbv::invertDisplay(bool i)
{
    WriteCmdData(0x36, i ? 2 : 0);
}

void HX8347D_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
    vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    WriteCmdData(0x01, 8);   //VLE
    WriteCmdDataPair(0x0E, top);        //TOP
    WriteCmdDataPair(0x10, scrollines);
    WriteCmdDataPair(0x12, 320 - top - scrollines);
    WriteCmdDataPair(0x14, vsp);  //VL#

}

#define TFTLCD_DELAY  0xFF

static const uint8_t HX8347G_2_regValues[] PROGMEM = {
    0xEA, 0x00, //PTBA[15:8]
    0xEB, 0x20, //PTBA[7:0]
    0xEC, 0x0C, //STBA[15:8]
    0xED, 0xC4, //STBA[7:0]
    0xE8, 0x38, //OPON[7:0]
    0xE9, 0x10, //OPON1[7:0]
    0xF1, 0x01, //OTPS1B
    0xF2, 0x10, //GEN
    //Gamma 2.2 Setting
    0x40, 0x01, //
    0x41, 0x00, //
    0x42, 0x00, //
    0x43, 0x10, //
    0x44, 0x0E, //
    0x45, 0x24, //
    0x46, 0x04, //
    0x47, 0x50, //
    0x48, 0x02, //
    0x49, 0x13, //
    0x4A, 0x19, //
    0x4B, 0x19, //
    0x4C, 0x16, //
    0x50, 0x1B, //
    0x51, 0x31, //
    0x52, 0x2F, //
    0x53, 0x3F, //
    0x54, 0x3F, //
    0x55, 0x3E, //
    0x56, 0x2F, //
    0x57, 0x7B, //
    0x58, 0x09, //
    0x59, 0x06, //
    0x5A, 0x06, //
    0x5B, 0x0C, //
    0x5C, 0x1D, //
    0x5D, 0xCC, //
    //Power Voltage Setting
    0x1B, 0x1B, //VRH=4.65V
    0x1A, 0x01, //BT (VGH~15V,VGL~-10V,DDVDH~5V)
    0x24, 0x2F, //VMH(VCOM High voltage ~3.2V)
    0x25, 0x57, //VML(VCOM Low voltage -1.2V)
    //****VCOM offset**///
    0x23, 0x88, //for Flicker adjust //can reload from OTP
    //Power on Setting
    0x18, 0x34, //I/P_RADJ,N/P_RADJ, Normal mode 60Hz
    0x19, 0x01, //OSC_EN='1', start Osc
    0x01, 0x00, //DP_STB='0', out deep sleep
    0x1F, 0x88, // GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
    TFTLCD_DELAY, 5,
    0x1F, 0x80, // GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0
    TFTLCD_DELAY, 5,
    0x1F, 0x90, // GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0
    TFTLCD_DELAY, 5,
    0x1F, 0xD0, // GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0, STB=0
    TFTLCD_DELAY, 5,
    //262k/65k color selection
    0x17, 0x05, //default 0x06 262k color // 0x05 65k color
    //SET PANEL
    0x36, 0x00, //SS_P, GS_P,REV_P,BGR_P
    //Display ON Setting
    0x28, 0x38, //GON=1, DTE=1, D=1000
    TFTLCD_DELAY, 40,
    0x28, 0x3F, //GON=1, DTE=1, D=1100

    0x16, 0x18,
    //Set GRAM Area
    0x02, 0x00,
    0x03, 0x00, //Column Start
    0x04, 0x00,
    0x05, 0xEF, //Column End
    0x06, 0x00,
    0x07, 0x00, //Row Start
    0x08, 0x01,
    0x09, 0x3F, //Row End
};

void HX8347D_kbv::begin(uint16_t ID)
{
    _lcd_ID = ID;
    uint8_t cmd, d;
    uint8_t *p = (uint8_t *) HX8347G_2_regValues;
    int16_t size = sizeof(HX8347G_2_regValues);
    reset();
    while (size > 0) {
        cmd = pgm_read_byte(p++);
        d = pgm_read_byte(p++);
        size -= 2;
        if (cmd == TFTLCD_DELAY) wait_ms(d);
        else WriteCmdData(cmd, d);
    }
    setRotation(0);                //PORTRAIT
}
#endif
