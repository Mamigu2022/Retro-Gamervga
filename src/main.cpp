#include <Arduino.h>

#include <fabgl.h>
#include <SD.h>
#include <SPI.h>
#include <esp_heap_caps.h>

// ============================================================================
// EXT_RAM_ATTR — mueve variables estáticas grandes a PSRAM (extern_ram_seg)
// Úsalo en arrays que NO necesitan acceso ultra-rápido desde ISR.
// Requiere CONFIG_SPIRAM_ALLOW_BSS_SEG=y en sdkconfig (activo por defecto
// cuando PSRAM está habilitada en ESP32 Arduino).
// ============================================================================
#ifndef EXT_RAM_ATTR
  #define EXT_RAM_ATTR __attribute__((section(".ext_ram.bss")))
#endif

// ============================================================================
// BLUEPAD32 SUPPORT
// Compilar con -D BT_GAMEPAD_INPUT_BLUEPAD en platformio.ini para activar
// ============================================================================
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
#include <Bluepad32.h>

// bp32_gamepads DEBE estar en DRAM interna — los callbacks de Bluetooth
// se ejecutan desde contexto de ISR/tarea BT donde PSRAM no es accesible.
// Es solo BP32_MAX_GAMEPADS punteros (~16 bytes), no merece la pena moverlo a PSRAM.
static GamepadPtr bp32_gamepads[BP32_MAX_GAMEPADS];
// volatile debe permanecer en DRAM (accedido desde ISR/callbacks)
static volatile uint32_t gp_buttons = 0;
static volatile int16_t  gp_axis_lx = 0;
static volatile int16_t  gp_axis_ly = 0;

// Umbral de stick analógico
#define GP_AXIS_THRESHOLD 200

void onConnectedController(GamepadPtr ctl) {
    ControllerProperties props = ctl->getProperties();
    Serial.printf("[BP32] Conexión — VID: 0x%04x PID: 0x%04x\n",
        props.vendor_id, props.product_id);
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (bp32_gamepads[i] == nullptr) {
            bp32_gamepads[i] = ctl;

            return;
        }
    }
}

void onDisconnectedController(GamepadPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (bp32_gamepads[i] == ctl) {
            bp32_gamepads[i] = nullptr;

            return;
        }
    }
}

bool bluepad32_has_gamepad() {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
        if (bp32_gamepads[i] && bp32_gamepads[i]->isConnected()) return true;
    return false;
}

void initBluePad32() {
    memset(bp32_gamepads, 0, sizeof(bp32_gamepads));
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);
    BP32.forgetBluetoothKeys();
}

#endif // BT_GAMEPAD_INPUT_BLUEPAD

// Debug system
#define DEBUG_ENABLED 0
#define DEBUG_VGA_OSD 0
#include "debug.h"

// Sound control
#include "audio_i2s.h"
#include "sound_control.h"

// Emulator bridges
extern "C" {
#include "nes_bridge.h"
#include "gb_bridge.h"
#include "sms_bridge.h"
#include "pce_bridge.h"
#include "snes_bridge.h"
#include "lynx_bridge.h"
#include "genesis_bridge.h"
}

void drawMenu();


#define BOARD_TYPE BOARD_TTGO_VGA32

#if BOARD_TYPE == BOARD_TTGO_VGA32
#define BOARD_NAME "LilyGO TTGO VGA32"
#define VGA_RED1   GPIO_NUM_21
#define VGA_RED0   GPIO_NUM_22
#define VGA_GREEN1 GPIO_NUM_18
#define VGA_GREEN0 GPIO_NUM_19
#define VGA_BLUE1  GPIO_NUM_4
#define VGA_BLUE0  GPIO_NUM_5
#define VGA_HSYNC  GPIO_NUM_23
#define VGA_VSYNC  GPIO_NUM_15
#define SD_CS      13
#define KBD_CLK    GPIO_NUM_33
#define KBD_DAT    GPIO_NUM_32
#define AUDIO_DAC  GPIO_NUM_25

#elif BOARD_TYPE == BOARD_OLIMEX_SBC
#define BOARD_NAME "Olimex ESP32-SBC-FabGL Rev B"
#define VGA_RED1   GPIO_NUM_21
#define VGA_RED0   GPIO_NUM_22
#define VGA_GREEN1 GPIO_NUM_18
#define VGA_GREEN0 GPIO_NUM_19
#define VGA_BLUE1  GPIO_NUM_4
#define VGA_BLUE0  GPIO_NUM_5
#define VGA_HSYNC  GPIO_NUM_23
#define VGA_VSYNC  GPIO_NUM_15
#define SD_CS      4
#define KBD_CLK    GPIO_NUM_33
#define KBD_DAT    GPIO_NUM_32
#define AUDIO_DAC  GPIO_NUM_25

#else
#error "Unknown board type!"
#endif

#define SD_MISO 35
#define SD_MOSI 12
#define SD_CLK  14

#define ROM_CACHE_FILE "/retro-go/rom_cache.txt"

// ============================================================================
// EMULATOR TYPES
// ============================================================================
typedef enum {
    EMU_NONE = 0,
    EMU_NES, EMU_GB, EMU_SMS, EMU_GG, EMU_SG1000, EMU_COLECO,
    EMU_PCE, EMU_SNES, EMU_GENESIS, EMU_LYNX,
} emu_type_t;

static emu_type_t currentEmu = EMU_NONE;

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================
fabgl::VGAController DisplayController;
#ifdef TECLADO
fabgl::PS2Controller PS2Ctrl;
fabgl::Keyboard * Kbd = nullptr;
#endif

// pce_raw_cache en DRAM interna — se accede 320×240=76800 veces por frame,
// PSRAM es demasiado lenta para este patrón de acceso aleatorio.
// 512 bytes caben fácilmente en DRAM.
static uint8_t pce_raw_cache[256];  // 256 B en DRAM — índices de pixel son 8 bits (0-255)
static uint16_t *pce_last_pal  = nullptr;

// ============================================================================
// RGB CONVERSION FUNCTIONS
// ============================================================================
IRAM_ATTR uint8_t makeRawPixelFromRGB222Components(uint8_t r2, uint8_t g2, uint8_t b2) {
    fabgl::RGB222 rgb;
    rgb.R = r2 & 0x3;
    rgb.G = g2 & 0x3;
    rgb.B = b2 & 0x3;
    return DisplayController.createRawPixel(rgb);
}

IRAM_ATTR uint8_t rgb888_to_rgb222_fabgl(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t r2 = r >> 6;
    uint8_t g2 = g >> 6;
    uint8_t b2 = b >> 6;
    return (r2 << 4) | (g2 << 2) | b2;
}

IRAM_ATTR uint8_t makeRawPixelFromRGB565(uint16_t rgb565) {
    uint8_t r5 = (rgb565 >> 11) & 0x1F;
    uint8_t g6 = (rgb565 >>  5) & 0x3F;
    uint8_t b5 =  rgb565        & 0x1F;
    uint8_t r2 = r5 >> 3;
    uint8_t g2 = g6 >> 4;
    uint8_t b2 = b5 >> 3;
    return makeRawPixelFromRGB222Components(r2, g2, b2);
}

IRAM_ATTR uint8_t makeRawPixelFromRGB222(uint8_t rgb222) {
    uint8_t r2 = (rgb222 >> 4) & 0x3;
    uint8_t g2 = (rgb222 >> 2) & 0x3;
    uint8_t b2 =  rgb222       & 0x3;
    return makeRawPixelFromRGB222Components(r2, g2, b2);
}

IRAM_ATTR void rebuildPCERawCache(uint16_t *pal) {
    for (int i = 0; i < 256; i++) {
        pce_raw_cache[i] = makeRawPixelFromRGB565(pal[i]);
    }
    pce_last_pal = pal;
}

// ============================================================================
// COLORS
// ============================================================================
#define C_BLACK          fabgl::RGB888(  0,   0,   0)
#define C_BRIGHT_WHITE   fabgl::RGB888(255, 255, 255)
#define C_GRAY           fabgl::RGB888(170, 170, 170)
#define C_DARK_GRAY      fabgl::RGB888( 85,  85,  85)
#define C_BRIGHT_RED     fabgl::RGB888(255,  85,  85)
#define C_BRIGHT_GREEN   fabgl::RGB888( 85, 255,  85)
#define C_BLUE           fabgl::RGB888(  0,   0, 255)
#define C_BRIGHT_BLUE    fabgl::RGB888( 85,  85, 255)
#define C_BRIGHT_CYAN    fabgl::RGB888( 85, 255, 255)
#define C_BRIGHT_MAGENTA fabgl::RGB888(255,  85, 255)
#define C_YELLOW         fabgl::RGB888(255, 255,   0)
#define C_BRIGHT_YELLOW  fabgl::RGB888(255, 255,  85)

// ============================================================================
// NES PALETTE SYSTEM
// FIX: paletas movidas a Flash (PROGMEM) para liberar ~960 bytes de DRAM
// FIX: nesPalette222 y nesPaletteRawPixels → heap_caps PSRAM.
//      Se usan punteros en vez de arrays EXT_RAM_ATTR para no depender de
//      CONFIG_SPIRAM_ALLOW_BSS_SEG. Se inicializan en setup() con heap_caps_malloc.
// ============================================================================
static uint8_t *nesPalette222        = nullptr;  // 256 B en PSRAM (alloc en setup)
static uint8_t *nesPaletteRawPixels  = nullptr;  //  64 B en PSRAM (alloc en setup)
static bool     nesPaletteRawPixelsValid = false;

#define NES_PALETTE_COUNT 5
static int nesCurrentPalette = 0;

// FIX: strings en Flash
static const char * const nes_palette_names[NES_PALETTE_COUNT] PROGMEM = {
    "Default", "Vibrant", "Pastel", "Dark", "Mono"
};

// FIX: paletas en Flash (DROM) en vez de DRAM — ahorra ~960 bytes de DRAM
static const uint8_t nes_palettes[NES_PALETTE_COUNT][64][3] PROGMEM = {
// DEFAULT
{
{128,128,128}, {  0, 61,166}, {  0, 18,176}, { 68,  0,150},
{161,  0, 94}, {199,  0, 40}, {186,  6,  0}, {140, 23,  0},
{ 92, 47,  0}, { 16, 69,  0}, {  5, 74,  0}, {  0, 71, 46},
{  0, 65,102}, {  0,  0,  0}, {  5,  5,  5}, {  5,  5,  5},
{199,199,199}, {  0,119,255}, { 33, 85,255}, {130, 55,250},
{235, 47,181}, {255, 41, 80}, {255, 34,  0}, {214, 50,  0},
{ 94, 51,  8}, { 53,128,  0}, {  5,143,  0}, {  0,138, 85},
{  0,153,204}, { 33, 33, 33}, {  9,  9,  9}, {  9,  9,  9},
{255,255,255}, { 15,215,255}, {105,162,255}, {212,128,255},
{255, 69,243}, {255, 97,139}, {255,136, 51}, {255,230,150},
{250,188, 32}, {159,227, 14}, { 43,240, 53}, { 12,240,164},
{  5,251,255}, { 94, 94, 94}, { 13, 13, 13}, { 13, 13, 13},
{255,255,255}, {166,252,255}, {179,236,255}, {218,171,235},
{255,168,249}, {255,171,179}, {255,255,255}, {255,239,166},
{255,247,156}, {215,232,149}, {166,237,175}, {162,242,218},
{153,255,252}, {221,221,221}, { 17, 17, 17}, { 17, 17, 17},
},
// VIBRANT
{
{180,180,180}, {  0,100,220}, {  0, 50,240}, {100,  0,210},
{210,  0,130}, {250,  0, 70}, {240, 20,  0}, {190, 40,  0},
{130, 70,  0}, { 30,100,  0}, { 15,110,  0}, {  0,105, 70},
{  0, 95,140}, {  0,  0,  0}, { 10, 10, 10}, { 10, 10, 10},
{240,240,240}, {  0,160,255}, { 50,120,255}, {170, 80,255},
{255, 70,240}, {255, 60,110}, {255, 50, 10}, {250, 70, 10},
{130, 75, 15}, { 80,170,  0}, { 20,190,  0}, {  0,180,110},
{  0,200,255}, { 50, 50, 50}, { 15, 15, 15}, { 15, 15, 15},
{255,255,255}, { 30,240,255}, {130,200,255}, {240,160,255},
{255,100,255}, {255,120,180}, {255,160, 80}, {255,250,180},
{255,220, 50}, {200,255, 30}, { 70,255, 80}, { 25,255,190},
{ 15,255,255}, {120,120,120}, { 20, 20, 20}, { 20, 20, 20},
{255,255,255}, {190,255,255}, {200,250,255}, {240,200,255},
{255,190,255}, {255,195,210}, {255,255,255}, {255,250,200},
{255,255,190}, {240,250,180}, {190,255,200}, {185,255,230},
{180,255,255}, {240,240,240}, { 25, 25, 25}, { 25, 25, 25},
},
// PASTEL
{
{160,160,160}, { 60,100,180}, { 60, 70,190}, {110, 60,170},
{180, 60,130}, {210, 60, 90}, {200, 70, 50}, {160, 80, 50},
{120, 90, 50}, { 70,100, 50}, { 60,110, 50}, { 50,105, 80},
{ 50,100,130}, { 20, 20, 20}, { 30, 30, 30}, { 30, 30, 30},
{210,210,210}, { 60,140,220}, { 90,120,220}, {160,100,220},
{220, 90,210}, {230, 90,150}, {230,100, 90}, {200,110, 90},
{120,100, 60}, {100,140, 60}, { 70,150, 60}, { 60,145,110},
{ 60,155,200}, { 70, 70, 70}, { 40, 40, 40}, { 40, 40, 40},
{240,240,240}, { 90,210,240}, {140,190,240}, {210,170,240},
{240,150,240}, {240,160,200}, {240,180,150}, {240,230,170},
{230,210,120}, {190,230,110}, {130,235,130}, {110,235,180},
{100,240,240}, {130,130,130}, { 45, 45, 45}, { 45, 45, 45},
{240,240,240}, {200,235,240}, {210,225,240}, {230,210,235},
{240,205,240}, {240,210,220}, {240,240,240}, {240,235,210},
{240,240,205}, {225,235,200}, {200,240,210}, {195,240,225},
{190,240,240}, {225,225,225}, { 50, 50, 50}, { 50, 50, 50},
},
// DARK
{
{ 60, 60, 60}, {  0, 30,100}, {  0, 10,110}, { 40,  0, 90},
{ 90,  0, 60}, {110,  0, 30}, {105,  5,  0}, { 80, 15,  0},
{ 55, 30,  0}, { 10, 40,  0}, {  5, 45,  0}, {  0, 45, 30},
{  0, 40, 65}, {  0,  0,  0}, {  5,  5,  5}, {  5,  5,  5},
{100,100,100}, {  0, 70,150}, { 20, 50,150}, { 75, 35,145},
{135, 30,110}, {145, 25, 50}, {145, 20,  5}, {120, 30,  5},
{ 55, 30,  8}, { 30, 75,  0}, {  5, 85,  0}, {  0, 80, 50},
{  0, 90,120}, { 20, 20, 20}, {  8,  8,  8}, {  8,  8,  8},
{140,140,140}, { 10,120,140}, { 60, 95,140}, {120, 75,140},
{140, 45,135}, {140, 55, 80}, {140, 75, 30}, {140,130, 85},
{140,105, 20}, { 90,125, 10}, { 25,130, 30}, { 10,130, 90},
{  5,135,140}, { 55, 55, 55}, { 10, 10, 10}, { 10, 10, 10},
{140,140,140}, { 95,140,140}, {100,130,140}, {125,100,135},
{140, 95,140}, {140,100,105}, {140,140,140}, {140,135, 95},
{140,140, 90}, {120,130, 85}, { 95,135,100}, { 90,135,125},
{ 85,140,140}, {125,125,125}, { 15, 15, 15}, { 15, 15, 15},
},
// MONO
{
{128,128,128}, { 40, 40, 40}, { 30, 30, 30}, { 50, 50, 50},
{ 70, 70, 70}, { 90, 90, 90}, {100,100,100}, { 85, 85, 85},
{ 75, 75, 75}, { 65, 65, 65}, { 60, 60, 60}, { 70, 70, 70},
{ 80, 80, 80}, {  0,  0,  0}, { 20, 20, 20}, { 20, 20, 20},
{200,200,200}, { 50, 50, 50}, { 60, 60, 60}, { 70, 70, 70},
{ 80, 80, 80}, {100,100,100}, {120,120,120}, {110,110,110},
{ 95, 95, 95}, { 85, 85, 85}, { 75, 75, 75}, { 85, 85, 85},
{ 95, 95, 95}, { 40, 40, 40}, { 25, 25, 25}, { 25, 25, 25},
{255,255,255}, {130,130,130}, {140,140,140}, {150,150,150},
{160,160,160}, {170,170,170}, {180,180,180}, {200,200,200},
{190,190,190}, {170,170,170}, {150,150,150}, {140,140,140},
{130,130,130}, { 80, 80, 80}, { 35, 35, 35}, { 35, 35, 35},
{255,255,255}, {220,220,220}, {210,210,210}, {200,200,200},
{190,190,190}, {185,185,185}, {255,255,255}, {210,210,210},
{200,200,200}, {190,190,190}, {180,180,180}, {170,170,170},
{160,160,160}, {160,160,160}, { 45, 45, 45}, { 45, 45, 45},
},
};

// FIX: leer paleta desde Flash con pgm_read_byte
void updateNESPalette() {
    for (int i = 0; i < 64; i++) {
        uint8_t r = pgm_read_byte(&nes_palettes[nesCurrentPalette][i][0]);
        uint8_t g = pgm_read_byte(&nes_palettes[nesCurrentPalette][i][1]);
        uint8_t b = pgm_read_byte(&nes_palettes[nesCurrentPalette][i][2]);
        uint8_t v = rgb888_to_rgb222_fabgl(r, g, b);
        nesPalette222[i]       = v;
        nesPaletteRawPixels[i] = makeRawPixelFromRGB222(v);
    }
    // Replicar las 64 entradas en los 4 bloques de 256
    memcpy(nesPalette222,       nesPalette222,      64);
    memcpy(nesPalette222 + 64,  nesPalette222,      64);
    memcpy(nesPalette222 + 128, nesPalette222,      64);
    memcpy(nesPalette222 + 192, nesPalette222,      64);
    nesPaletteRawPixelsValid = true;
}

// ============================================================================
// APPLICATION STATE
// ============================================================================
typedef enum {
    STATE_BOOT,
    STATE_FILE_SELECT,
    STATE_EMULATING,
    STATE_INGAME_MENU,
} app_state_t;

static volatile app_state_t appState = STATE_BOOT;

#define MAX_ROMS 50
// FIX DRAM: arrays grandes → heap_caps PSRAM (punteros, alloc en setup)
static char **romFiles      = nullptr;   // 50 * 4 B = 200 B en PSRAM
static char  *currentRomPath = nullptr;  // 256 B en PSRAM
static int    romCount    = 0;
static int    selectedRom = 0;
static int    scrollOffset = 0;

typedef enum {
    IGMENU_RESUME = 0,
    IGMENU_SAVE_STATE,
    IGMENU_LOAD_STATE,
    IGMENU_TURBO,
    IGMENU_SOUND,
    IGMENU_PALETTE,
    IGMENU_ROTATE,
    IGMENU_RESET,
    IGMENU_QUIT,
    IGMENU_COUNT
} igmenu_item_t;

static int igMenuSel = 0;
static int saveSlot  = 0;

static bool turboMode   = false;
static int  fskip       = 0;
static int  frameCounter = 0;

// Lynx rotation support
static bool lynxRotated = false;
// FIX: strings en Flash
static const char * const lynxRotNames[] PROGMEM = { "Normal", "Rotated" };

#define MAX_FAVORITES 20
#define MAX_RECENT    10
// FIX DRAM: listas de punteros → heap_caps PSRAM (alloc en setup)
static char **favoritesList = nullptr;  // 20 * 4 B =  80 B en PSRAM
static char **recentList    = nullptr;  // 10 * 4 B =  40 B en PSRAM
static int    favCount    = 0;
static int    recentCount = 0;

typedef enum { 
    VIEW_ALL = 0, VIEW_FAVORITES, VIEW_RECENT,
    VIEW_NES, VIEW_GB, VIEW_SMS, VIEW_PCE, VIEW_GEN, VIEW_SNES, VIEW_LYNX,
    VIEW_COUNT 
} view_mode_t;
static view_mode_t viewMode = VIEW_ALL;

// ============================================================================
// HELPER: SCANLINE CLEAR
// ============================================================================
static uint8_t blackRawPixel = 0;

IRAM_ATTR void clearScanline(int y) {
    uint8_t *line = (uint8_t*)DisplayController.getScanline(y);
    if (line) memset(line, 0, 320);
}

IRAM_ATTR void writeRawPixelToScanline(uint8_t *line, int x, uint8_t rawPixel) {
    if (!line) return;
    int dword      = x >> 2;
    int byteOffset = ((x & 3) + 2) & 3;
    line[dword * 4 + byteOffset] = rawPixel;
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================
bool initVGA() {


    DisplayController.begin(
        VGA_RED1, VGA_RED0,
        VGA_GREEN1, VGA_GREEN0,
        VGA_BLUE1, VGA_BLUE0,
        VGA_HSYNC, VGA_VSYNC
    );


    DisplayController.setResolution(QVGA_320x240_60Hz, -1, -1, false);  // false = sin doble buffer DMA


    // Verificar que los punteros de scanline son válidos antes de usarlos
    void *sl0   = DisplayController.getScanline(0);
    void *sl239 = DisplayController.getScanline(239);


    // Un puntero válido de DRAM interna debe estar entre 0x3F800000 y 0x40000000
    bool sl0Valid   = ((uint32_t)sl0   >= 0x3F800000 && (uint32_t)sl0   < 0x40000000);
    bool sl239Valid = ((uint32_t)sl239 >= 0x3F800000 && (uint32_t)sl239 < 0x40000000);
    if (!sl0Valid || !sl239Valid) {
        Serial.println("[VGA] ERROR: setResolution falló — punteros de scanline inválidos!");
        Serial.println("[VGA] CAUSA: falta memoria DMA contigua para el framebuffer.");
        while(1) delay(1000);  // Parar aquí en vez de crash aleatorio
    }

    fabgl::RGB222 blackRGB;
    blackRGB.B = 0; blackRGB.G = 0; blackRGB.R = 0;
    blackRawPixel = DisplayController.createRawPixel(blackRGB);

    fabgl::Canvas *cv = new fabgl::Canvas(&DisplayController);
    if (cv) { cv->setBrushColor(C_BLACK); cv->clear(); delete cv; }

    for (int y = 0; y < 240; y++) clearScanline(y);
    return true;
}

#ifdef TECLADO
bool initPS2() {
    PS2Ctrl.begin(fabgl::PS2Preset::KeyboardPort0);
    Kbd = PS2Ctrl.keyboard();
    Kbd->begin(true, false, 0);
    bool isAvailable = Kbd && Kbd->isKeyboardAvailable();
    DBG_INFO("PS2", "Keyboard: %s", isAvailable ? "OK" : "N/A");
    return isAvailable;
}
#endif

bool initSD() {
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SPI, 20000000)) {
        DBG_WARN("SD", "Mount failed with 20MHz, trying 10MHz...");
        if (!SD.begin(SD_CS, SPI, 10000000)) {
            DBG_ERROR("SD", "Mount failed!");
            return false;
        }
    }
    if (SD.cardType() == CARD_NONE) {
        DBG_ERROR("SD", "No card found!");
        return false;
    }
    DBG_INFO("SD", "Card type: %d, size: %llu MB", SD.cardType(), SD.cardSize() / (1024*1024));
    return true;
}

// ============================================================================
// ROM TYPE DETECTION
// ============================================================================
emu_type_t getEmuType(const char *filename) {
    const char *ext = strrchr(filename, '.');
    if (!ext) return EMU_NONE;
    ext++;
    if (strcasecmp(ext, "nes") == 0 || strcasecmp(ext, "fc")  == 0) return EMU_NES;
    if (strcasecmp(ext, "gb")  == 0 || strcasecmp(ext, "gbc") == 0) return EMU_GB;
    if (strcasecmp(ext, "sms") == 0) return EMU_SMS;
    if (strcasecmp(ext, "gg")  == 0) return EMU_GG;
    if (strcasecmp(ext, "sg")  == 0) return EMU_SG1000;
    if (strcasecmp(ext, "pce") == 0) return EMU_PCE;
    if (strcasecmp(ext, "sfc") == 0 || strcasecmp(ext, "smc") == 0) return EMU_SNES;
    if (strcasecmp(ext, "lnx") == 0) return EMU_LYNX;
    if (strcasecmp(ext, "col") == 0) return EMU_COLECO;
    if (strcasecmp(ext, "md")  == 0 || strcasecmp(ext, "gen") == 0 ||
        strcasecmp(ext, "bin") == 0 || strcasecmp(ext, "smd") == 0) return EMU_GENESIS;
    return EMU_NONE;
}

const char* emuName(emu_type_t t) {
    switch(t) {
        case EMU_NES:     return "NES";
        case EMU_GB:      return "GB";
        case EMU_SMS:     return "SMS";
        case EMU_GG:      return "GG";
        case EMU_PCE:     return "PCE";
        case EMU_SNES:    return "SNES";
        case EMU_LYNX:    return "LYNX";
        case EMU_SG1000:  return "SG";
        case EMU_COLECO:  return "COL";
        case EMU_GENESIS: return "MD";
        default:          return "?";
    }
}

const char* emuShortName(emu_type_t t) {
    switch(t) {
        case EMU_NES:     return "nes";
        case EMU_GB:      return "gb";
        case EMU_SMS:     return "sms";
        case EMU_GG:      return "gg";
        case EMU_PCE:     return "pce";
        case EMU_SNES:    return "snes";
        case EMU_LYNX:    return "lnx";
        case EMU_SG1000:  return "sms";
        case EMU_COLECO:  return "col";
        case EMU_GENESIS: return "md";
        default:          return "misc";
    }
}

// ============================================================================
// ROM SCANNER WITH CACHE
// ============================================================================
static time_t getRomsDirectoriesModTime() {
    time_t maxTime = 0;
    const char *dirs[] = {
        "/roms/nes", "/roms/gb", "/roms/gbc", "/roms/sms", "/roms/gg",
        "/roms/pce", "/roms/snes", "/roms/lnx", "/roms/col",
        "/roms/md",  "/roms/sg1000"
    };
    for (int i = 0; i < (int)(sizeof(dirs)/sizeof(dirs[0])); i++) {
        File d = SD.open(dirs[i]);
        if (d && d.isDirectory()) {
            time_t t = d.getLastWrite();
            if (t > maxTime) maxTime = t;
            d.close();
        }
    }
    return maxTime;
}

static void showStatusMessage(const char *message) {
    auto cv = new fabgl::Canvas(&DisplayController);
    cv->setBrushColor(C_BLACK);
    cv->fillRectangle(8, 228, 150, 238);
    cv->setPenColor(C_GRAY);
    cv->selectFont(&fabgl::FONT_8x8);
    cv->drawText(10, 230, message);
    delete cv;
}

static void showLoadingMessage(const char *message) {
    auto cv = new fabgl::Canvas(&DisplayController);
    cv->setBrushColor(C_BLACK);
    cv->fillRectangle(100, 110, 220, 130);
    cv->setPenColor(C_BRIGHT_YELLOW);
    cv->selectFont(&fabgl::FONT_8x8);
    cv->drawText(108, 118, message);
    delete cv;
}

void scanDir(File dir, const char *base) {
    File entry;
    while ((entry = dir.openNextFile()) && romCount < MAX_ROMS) {
        if (entry.isDirectory()) {
            String sub = String(base) + "/" + entry.name();
            scanDir(entry, sub.c_str());
        } else {
            if (getEmuType(entry.name()) != EMU_NONE) {
                char fp[256];
                snprintf(fp, sizeof(fp), "%s/%s", base, entry.name());
                romFiles[romCount++] = strdup(fp);
            }
        }
        entry.close();
    }
}

void saveRomCache() {
    if (!SD.exists("/retro-go")) SD.mkdir("/retro-go");
    File cache = SD.open(ROM_CACHE_FILE, FILE_WRITE);
    if (!cache) return;
    time_t romsTime = getRomsDirectoriesModTime();
    cache.print("TIMESTAMP:");
    cache.println((int32_t)romsTime);
    for (int i = 0; i < romCount; i++) cache.println(romFiles[i]);
    cache.close();
    DBG_VERBOSE("ROM", "Cache saved with %d files", romCount);
}

bool loadRomCache() {
    File cache = SD.open(ROM_CACHE_FILE, FILE_READ);
    if (!cache) return false;

    int32_t savedTimestamp = 0;
    String firstLine = cache.readStringUntil('\n');
    firstLine.trim();
    cache.seek(0);

    if (firstLine.startsWith("TIMESTAMP:")) {
        savedTimestamp = firstLine.substring(10).toInt();
        cache.readStringUntil('\n');
    }

    romCount = 0;
    while (cache.available() && romCount < MAX_ROMS) {
        String lineStr = cache.readStringUntil('\n');
        lineStr.trim();
        if (lineStr.length() > 0 && !lineStr.startsWith("TIMESTAMP:"))
            romFiles[romCount++] = strdup(lineStr.c_str());
    }
    cache.close();

    time_t currentRomsTime = getRomsDirectoriesModTime();
    if (savedTimestamp != 0 && currentRomsTime > savedTimestamp) {
        for (int i = 0; i < romCount; i++) { free(romFiles[i]); romFiles[i] = NULL; }
        romCount = 0;
        return false;
    }
    return romCount > 0;
}

void scanAllRoms() {
    for (int i = 0; i < romCount; i++) { free(romFiles[i]); romFiles[i] = NULL; }
    romCount = 0; selectedRom = 0; scrollOffset = 0;

    if (loadRomCache()) return;

    showLoadingMessage("Reading SD...");
    delay(50);

    const char *dirs[] = {
        "/roms/nes", "/roms/gb", "/roms/gbc", "/roms/sms", "/roms/gg",
        "/roms/pce", "/roms/snes", "/roms/lnx", "/roms/col",
        "/roms/md",  "/roms/sg1000"
    };
    for (int i = 0; i < (int)(sizeof(dirs)/sizeof(dirs[0])); i++) {
        File d = SD.open(dirs[i]);
        if (d && d.isDirectory()) { scanDir(d, dirs[i]); d.close(); }
    }

    if (romCount > 0) saveRomCache();

    auto cv = new fabgl::Canvas(&DisplayController);
    cv->setBrushColor(C_BLACK);
    cv->fillRectangle(100, 110, 220, 130);
    delete cv;
}

void refreshRomList() {
    showLoadingMessage("Refreshing...");
    delay(50);
    if (SD.exists(ROM_CACHE_FILE)) SD.remove(ROM_CACHE_FILE);
    for (int i = 0; i < romCount; i++) { free(romFiles[i]); romFiles[i] = NULL; }
    romCount = 0;
    scanAllRoms();
    drawMenu();
}

// ============================================================================
// FAVORITES & RECENT
// ============================================================================
bool isFavorite(const char *path) {
    for (int i = 0; i < favCount; i++)
        if (strcmp(favoritesList[i], path) == 0) return true;
    return false;
}

static int loadStringList(const char *path, char **list, int maxCount) {
    File f = SD.open(path, FILE_READ);
    if (!f) return 0;
    int count = 0;
    char line[256];
    while (f.available() && count < maxCount) {
        int len = 0;
        while (f.available() && len < 255) {
            char c = f.read();
            if (c == '\n' || c == '\r') break;
            line[len++] = c;
        }
        line[len] = '\0';
        if (len > 0) list[count++] = strdup(line);
    }
    f.close();
    return count;
}

void loadFavorites() {
    for (int i = 0; i < favCount; i++) { free(favoritesList[i]); favoritesList[i] = NULL; }
    favCount = loadStringList("/retro-go/config/favorites.txt", favoritesList, MAX_FAVORITES);
}

void loadRecent() {
    for (int i = 0; i < recentCount; i++) { free(recentList[i]); recentList[i] = NULL; }
    recentCount = loadStringList("/retro-go/config/recent.txt", recentList, MAX_RECENT);
}

void saveFavorites() {
    if (!SD.exists("/retro-go")) SD.mkdir("/retro-go");
    if (!SD.exists("/retro-go/config")) SD.mkdir("/retro-go/config");
    File f = SD.open("/retro-go/config/favorites.txt", FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < favCount; i++) f.println(favoritesList[i]);
    f.close();
}

void toggleFavorite(const char *path) {
    for (int i = 0; i < favCount; i++) {
        if (strcmp(favoritesList[i], path) == 0) {
            free(favoritesList[i]);
            for (int j = i; j < favCount-1; j++) favoritesList[j] = favoritesList[j+1];
            favCount--;
            saveFavorites();
            return;
        }
    }
    if (favCount < MAX_FAVORITES) {
        favoritesList[favCount++] = strdup(path);
        saveFavorites();
    }
}

void saveRecent() {
    if (!SD.exists("/retro-go")) SD.mkdir("/retro-go");
    if (!SD.exists("/retro-go/config")) SD.mkdir("/retro-go/config");
    File f = SD.open("/retro-go/config/recent.txt", FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < recentCount; i++) f.println(recentList[i]);
    f.close();
}

void addToRecent(const char *path) {
    for (int i = 0; i < recentCount; i++) {
        if (strcmp(recentList[i], path) == 0) {
            free(recentList[i]);
            for (int j = i; j < recentCount-1; j++) recentList[j] = recentList[j+1];
            recentCount--;
            break;
        }
    }
    if (recentCount >= MAX_RECENT) {
        free(recentList[recentCount-1]);
        recentCount--;
    }
    for (int i = recentCount; i > 0; i--) recentList[i] = recentList[i-1];
    recentList[0] = strdup(path);
    recentCount++;
    saveRecent();
}

// ============================================================================
// BLITTING FUNCTIONS
// ============================================================================

// --- NES ---
// Empaqueta 4 píxeles raw en una palabra de 32 bits según formato FabGL VGA222
// Orden en memoria: [x%4=2][x%4=3][x%4=0][x%4=1]
IRAM_ATTR inline uint32_t packPixels4(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3) {
    return ((uint32_t)p2) | ((uint32_t)p3 << 8) | ((uint32_t)p0 << 16) | ((uint32_t)p1 << 24);
}
IRAM_ATTR inline uint32_t blackWord4(uint8_t bp) { return packPixels4(bp,bp,bp,bp); }

IRAM_ATTR void blitNES(uint8_t *fb) {
    if (!fb) return;
    if (!nesPaletteRawPixelsValid) updateNESPalette();

    const int nesPitch     = 272;
    const int nesOffset    = 8;
    const int visibleWidth = 256;  // múltiplo de 4 ✓
    const int targetX      = 32;   // múltiplo de 4 ✓

    uint32_t bw = blackWord4(blackRawPixel);

    for (int y = 0; y < 240; y++) {
        uint32_t *dest = (uint32_t*)DisplayController.getScanline(y);
        if (!dest) continue;
        uint8_t *src = fb + y * nesPitch + nesOffset;

        // Borde izquierdo: 32px = 8 palabras
        for (int i = 0; i < 8; i++) dest[i] = bw;

        // Píxeles NES: 256px = 64 palabras
        uint32_t *out = dest + 8;
        const uint8_t *pal = nesPaletteRawPixels;
        for (int i = 0; i < 64; i++) {
            int x = i * 4;
            out[i] = packPixels4(pal[src[x]&0x3F], pal[src[x+1]&0x3F],
                                  pal[src[x+2]&0x3F], pal[src[x+3]&0x3F]);
        }

        // Borde derecho: 32px = 8 palabras
        for (int i = 72; i < 80; i++) dest[i] = bw;
    }
}

IRAM_ATTR void blitGB(uint16_t *fb) {
    if (!fb) return;
    const int gbWidth  = 160;  // múltiplo de 4 ✓
    const int gbHeight = 144;
    const int xOffset  = 80;   // múltiplo de 4 ✓
    const int yOffset  = 48;

    uint32_t bw = blackWord4(blackRawPixel);

    for (int y = 0; y < 240; y++) {
        uint32_t *dest = (uint32_t*)DisplayController.getScanline(y);
        if (!dest) continue;
        if (y < yOffset || y >= yOffset + gbHeight) {
            for (int i = 0; i < 80; i++) dest[i] = bw;
            continue;
        }
        uint16_t *src = fb + (y - yOffset) * gbWidth;

        // Borde izquierdo: 80px = 20 palabras
        for (int i = 0; i < 20; i++) dest[i] = bw;

        // Píxeles GB: 160px = 40 palabras
        uint32_t *out = dest + 20;
        for (int i = 0; i < 40; i++) {
            int x = i * 4;
            out[i] = packPixels4(
                makeRawPixelFromRGB565(__builtin_bswap16(src[x])),
                makeRawPixelFromRGB565(__builtin_bswap16(src[x+1])),
                makeRawPixelFromRGB565(__builtin_bswap16(src[x+2])),
                makeRawPixelFromRGB565(__builtin_bswap16(src[x+3]))
            );
        }

        // Borde derecho: 80px = 20 palabras
        for (int i = 60; i < 80; i++) dest[i] = bw;
    }
}

// FIX DRAM: caches de paleta SMS y Genesis → heap_caps PSRAM (punteros, alloc en setup)
// No depende de CONFIG_SPIRAM_ALLOW_BSS_SEG a diferencia de EXT_RAM_ATTR en arrays.
static uint8_t  *smsPaletteRawPixels = nullptr;  // 32 B en PSRAM (suficiente — solo 32 entradas)
static uint16_t *smsLastPal          = nullptr;  // 64 B en PSRAM
static bool      smsPaletteValid     = false;
static uint8_t   smsBlackRaw        = 0;
static bool      smsBlackValid      = false;
static uint8_t  *cramRaw             = nullptr;  //  64 B en PSRAM
static uint16_t *lastCRAM565         = nullptr;  // 128 B en PSRAM
static bool      cramValid           = false;

// --- SMS / Game Gear / SG-1000 / Coleco ---
IRAM_ATTR void blitSMS(uint8_t *fb, uint16_t *pal, int w, int h) {
    if (!fb || !pal) return;

    bool isGameGear = (w == 160);
    int  startPixel   = 0;
    int  visibleWidth = w;
    int  pitch        = 256;

    if (!isGameGear) {
        startPixel   = 8;
        visibleWidth = w - 8;
    } else {
        int viewportX = sms_bridge_get_viewport_x();
        startPixel    = viewportX;
        visibleWidth  = w;
    }

    int xOffset = (320 - visibleWidth) / 2;
    int yOffset = (240 - h) / 2;

    if (!smsPaletteValid || memcmp(pal, smsLastPal, 32 * sizeof(uint16_t)) != 0) {
        for (int i = 0; i < 32; i++)
            smsPaletteRawPixels[i] = makeRawPixelFromRGB565(pal[i]);
        memcpy(smsLastPal, pal, 32 * sizeof(uint16_t));
        smsPaletteValid = true;
    }
    if (!smsBlackValid) {
        smsBlackRaw   = makeRawPixelFromRGB565(0);
        smsBlackValid = true;
    }

    int yStart = (yOffset < 0)       ? 0   : yOffset;
    int yEnd   = (yOffset + h > 240) ? 240 : yOffset + h;

    // Alinear xOffset a múltiplo de 4 para escritura en palabras
    int xOff4 = xOffset & ~3;
    int leftWords  = xOff4 / 4;
    int pixWords   = visibleWidth / 4;
    int rightStart = (xOff4 + visibleWidth + 3) & ~3;
    uint32_t bw = blackWord4(smsBlackRaw);

    for (int y = 0; y < 240; y++) {
        uint32_t *dest = (uint32_t*)DisplayController.getScanline(y);
        if (!dest) continue;
        if (y < yStart || y >= yEnd) {
            for (int i = 0; i < 80; i++) dest[i] = bw;
            continue;
        }
        uint8_t *src = fb + (y - yOffset) * pitch;

        for (int i = 0; i < leftWords; i++) dest[i] = bw;

        uint32_t *out = dest + leftWords;
        for (int i = 0; i < pixWords; i++) {
            int x = i * 4;
            out[i] = packPixels4(
                smsPaletteRawPixels[src[startPixel+x]   & 0x1F],
                smsPaletteRawPixels[src[startPixel+x+1] & 0x1F],
                smsPaletteRawPixels[src[startPixel+x+2] & 0x1F],
                smsPaletteRawPixels[src[startPixel+x+3] & 0x1F]
            );
        }
        for (int x = pixWords*4; x < visibleWidth; x++)
            writeRawPixelToScanline((uint8_t*)dest, xOffset+x, smsPaletteRawPixels[src[startPixel+x]&0x1F]);

        for (int i = rightStart/4; i < 80; i++) dest[i] = bw;
    }
}

// --- PC Engine ---
IRAM_ATTR void blitPCE(uint8_t *fb, uint16_t *pal, int w, int h) {
    if (!fb || !pal) return;
    if (pal != pce_last_pal || pce_pal_dirty) rebuildPCERawCache(pal);

    // xOffset debe ser múltiplo de 4 para escritura en palabras alineadas
    int xOffset = ((320 - w) / 2) & ~3;
    int yOffset = (240 - h) / 2;
    int pitch   = 368;

    uint32_t blackWord = blackWord4(blackRawPixel);
    int leftWords  = xOffset / 4;
    int rightWords = (320 - xOffset - w) / 4;
    int pixWords   = w / 4;

    for (int y = 0; y < 240; y++) {
        uint32_t *dest = (uint32_t*)DisplayController.getScanline(y);
        if (!dest) continue;

        if (y < yOffset || y >= yOffset + h) {
            // Fila negra — 80 palabras de 32 bits
            uint32_t *d = dest;
            for (int i = 0; i < 80; i++) *d++ = blackWord;
            continue;
        }

        uint8_t *src = fb + (y - yOffset) * pitch;

        // Borde izquierdo
        for (int i = 0; i < leftWords; i++) dest[i] = blackWord;

        // Píxeles PCE — 4 a la vez usando pce_raw_cache en DRAM
        uint32_t *out = dest + leftWords;
        for (int i = 0; i < pixWords; i++) {
            int x = i * 4;
            out[i] = packPixels4(pce_raw_cache[src[x]], pce_raw_cache[src[x+1]],
                                  pce_raw_cache[src[x+2]], pce_raw_cache[src[x+3]]);
        }
        // Píxeles restantes si w no es múltiplo de 4
        for (int x = pixWords * 4; x < w; x++)
            writeRawPixelToScanline((uint8_t*)dest, xOffset + x, pce_raw_cache[src[x]]);

        // Borde derecho
        uint32_t *right = dest + leftWords + pixWords;
        for (int i = 0; i < rightWords; i++) right[i] = blackWord;
    }
}

// --- SNES ---
IRAM_ATTR void blitSNES(uint16_t *fb, int w, int h) {
    if (!fb) return;
    int xOffset = (320 - w) / 2;
    int yOffset = (240 - h) / 2;

    for (int y = 0; y < 240; y++) {
        uint8_t *dest = (uint8_t*)DisplayController.getScanline(y);
        if (!dest) continue;
        if (y < yOffset || y >= yOffset + h) {
            for (int x = 0; x < 320; x++) writeRawPixelToScanline(dest, x, blackRawPixel);
            continue;
        }
        uint16_t *src = fb + (y - yOffset) * w;
        for (int x = 0; x < xOffset; x++)
            writeRawPixelToScanline(dest, x, blackRawPixel);
        for (int x = 0; x < w; x++)
            writeRawPixelToScanline(dest, xOffset + x, makeRawPixelFromRGB565(src[x]));
        for (int x = xOffset + w; x < 320; x++)
            writeRawPixelToScanline(dest, x, blackRawPixel);
    }
}

// --- Genesis ---
IRAM_ATTR void blitGenesis(uint8_t *fb, int w, int h) {
    if (!fb) return;
    extern unsigned short CRAM565[];

    if (!cramValid || memcmp(CRAM565, lastCRAM565, 64 * sizeof(uint16_t)) != 0) {
        for (int i = 0; i < 64; i++)
            cramRaw[i] = makeRawPixelFromRGB565(CRAM565[i]);
        memcpy(lastCRAM565, CRAM565, 64 * sizeof(uint16_t));
        cramValid = true;
    }

    const int pitch   = 320;
    int xOffset = (320 - w) / 2;
    int yOffset = (240 - h) / 2;

    for (int y = 0; y < 240; y++) {
        uint8_t *dest = (uint8_t*)DisplayController.getScanline(y);
        if (!dest) continue;
        if (y < yOffset || y >= yOffset + h) {
            for (int x = 0; x < 320; x++) writeRawPixelToScanline(dest, x, blackRawPixel);
            continue;
        }
        uint8_t *src = fb + (y - yOffset) * pitch + xOffset;
        for (int x = 0; x < xOffset; x++)
            writeRawPixelToScanline(dest, x, blackRawPixel);
        for (int x = 0; x < w; x++)
            writeRawPixelToScanline(dest, xOffset + x, cramRaw[src[x] & 0x3F]);
        for (int x = xOffset + w; x < 320; x++)
            writeRawPixelToScanline(dest, x, blackRawPixel);
    }
}

// --- Lynx ---
IRAM_ATTR void blitLynxRotated(uint16_t *fb) {
    if (!fb) return;
    const int srcWidth  = 160;
    const int srcHeight = 102;
    const int dstWidth  = 102;
    const int dstHeight = 160;
    int xOffset = (320 - dstWidth)  / 2;
    int yOffset = (240 - dstHeight) / 2;

    for (int y = 0; y < yOffset; y++) {
        uint8_t *dest = (uint8_t*)DisplayController.getScanline(y);
        if (dest) for (int x = 0; x < 320; x++) writeRawPixelToScanline(dest, x, blackRawPixel);
    }
    for (int y = yOffset + dstHeight; y < 240; y++) {
        uint8_t *dest = (uint8_t*)DisplayController.getScanline(y);
        if (dest) for (int x = 0; x < 320; x++) writeRawPixelToScanline(dest, x, blackRawPixel);
    }
    for (int y = 0; y < dstHeight; y++) {
        uint8_t *dest = (uint8_t*)DisplayController.getScanline(y + yOffset);
        if (!dest) continue;
        for (int x = 0; x < xOffset; x++)
            writeRawPixelToScanline(dest, x, blackRawPixel);
        for (int x = 0; x < dstWidth; x++) {
            int srcX = y;
            int srcY = srcHeight - 1 - x;
            uint16_t pixel = fb[srcY * srcWidth + srcX];
            writeRawPixelToScanline(dest, xOffset + x, makeRawPixelFromRGB565(pixel));
        }
        for (int x = xOffset + dstWidth; x < 320; x++)
            writeRawPixelToScanline(dest, x, blackRawPixel);
    }
}

IRAM_ATTR void blitLynx(uint16_t *fb) {
    if (!fb) return;
    if (lynxRotated) { blitLynxRotated(fb); return; }

    const int srcWidth  = 160;
    const int srcHeight = 102;
    const int xOffset   = 80;
    const int yOffset   = 69;

    for (int y = 0; y < 240; y++) {
        uint8_t *dest = (uint8_t*)DisplayController.getScanline(y);
        if (!dest) continue;
        if (y < yOffset || y >= yOffset + srcHeight) {
            for (int x = 0; x < 320; x++) writeRawPixelToScanline(dest, x, blackRawPixel);
            continue;
        }
        uint16_t *src = fb + (y - yOffset) * srcWidth;
        for (int x = 0; x < xOffset; x++)
            writeRawPixelToScanline(dest, x, blackRawPixel);
        for (int x = 0; x < srcWidth; x++) {
            uint16_t pixel = src[x];
            writeRawPixelToScanline(dest, xOffset + x, makeRawPixelFromRGB565(pixel));
        }
        for (int x = xOffset + srcWidth; x < 320; x++)
            writeRawPixelToScanline(dest, x, blackRawPixel);
    }
}

// ============================================================================
// SAVE STATE HELPERS
// ============================================================================
void getSaveStatePath(char *out, size_t sz, int slot) {
    const char *fn = strrchr(currentRomPath, '/');
    fn = fn ? fn + 1 : currentRomPath;
    char cleanName[256];
    strncpy(cleanName, fn, sizeof(cleanName) - 1);
    cleanName[sizeof(cleanName) - 1] = '\0';
    for (int i = 0; cleanName[i] != '\0'; i++)
        if (cleanName[i] == ' ') cleanName[i] = '_';
    char *dot = strrchr(cleanName, '.');
    if (dot) *dot = '\0';
    snprintf(out, sz, "/retro-go/saves/%s/%s.sav%d", emuShortName(currentEmu), cleanName, slot);
}

void createAllSaveDirectories() {
    const char *emu_dirs[] = {
        "/retro-go/saves/nes",  "/retro-go/saves/sms",  "/retro-go/saves/gg",
        "/retro-go/saves/gb",   "/retro-go/saves/pce",  "/retro-go/saves/snes",
        "/retro-go/saves/lnx",  "/retro-go/saves/md",   "/retro-go/saves/genesis",
        "/retro-go/saves/col",  "/retro-go/saves/sg",   "/retro-go/saves/misc"
    };
    if (!SD.exists("/retro-go"))        SD.mkdir("/retro-go");
    if (!SD.exists("/retro-go/saves"))  SD.mkdir("/retro-go/saves");
    for (int i = 0; i < (int)(sizeof(emu_dirs)/sizeof(emu_dirs[0])); i++)
        if (!SD.exists(emu_dirs[i])) SD.mkdir(emu_dirs[i]);
}

bool ensureSaveDir() {
    char dir_sd[128];
    snprintf(dir_sd, sizeof(dir_sd), "/retro-go/saves/%s", emuShortName(currentEmu));
    if (!SD.exists(dir_sd) && !SD.mkdir(dir_sd)) return false;

    char testPath[256];
    snprintf(testPath, sizeof(testPath), "/sd/retro-go/saves/%s/test.txt", emuShortName(currentEmu));
    FILE *test = fopen(testPath, "w");
    if (test) { fprintf(test, "test"); fclose(test); remove(testPath); return true; }
    return false;
}

bool saveStateToSD(int slot) {
    if (!ensureSaveDir()) return false;
    char path[256];
    getSaveStatePath(path, sizeof(path), slot);
    bool ok = false;
    switch(currentEmu) {
        case EMU_NES:    ok = nes_bridge_save_state(path);     break;
        case EMU_GB:     ok = gb_bridge_save_state(path);      break;
        case EMU_SMS: case EMU_GG: case EMU_SG1000: case EMU_COLECO:
                         ok = sms_bridge_save_state(path);     break;
        case EMU_PCE:    ok = pce_bridge_save_state(path);     break;
        case EMU_SNES:   ok = snes_bridge_save_state(path);    break;
        case EMU_LYNX:   ok = lynx_bridge_save_state(path);    break;
        case EMU_GENESIS:ok = genesis_bridge_save_state(path); break;
        default: break;
    }
    return ok;
}

bool loadStateFromSD(int slot) {
    char path[256];
    getSaveStatePath(path, sizeof(path), slot);
    if (!SD.exists(path)) return false;
    bool ok = false;
    switch(currentEmu) {
        case EMU_NES:    ok = nes_bridge_load_state(path);     break;
        case EMU_GB:     ok = gb_bridge_load_state(path);      break;
        case EMU_SMS: case EMU_GG: case EMU_SG1000: case EMU_COLECO:
                         ok = sms_bridge_load_state(path);     break;
        case EMU_PCE:    ok = pce_bridge_load_state(path);     break;
        case EMU_SNES:   ok = snes_bridge_load_state(path);    break;
        case EMU_LYNX:   ok = lynx_bridge_load_state(path);    break;
        case EMU_GENESIS:ok = genesis_bridge_load_state(path); break;
        default: break;
    }
    return ok;
}

// ============================================================================
// CLEAN EXIT TO MENU
// ============================================================================
void cleanExitToMenu() {
    audioSetEnabled(false);
    delay(50);
    if (currentEmu != EMU_NONE) {
        switch(currentEmu) {
            case EMU_NES:     nes_bridge_shutdown();     break;
            case EMU_GB:      gb_bridge_shutdown();      break;
            case EMU_SMS: case EMU_GG: case EMU_SG1000: case EMU_COLECO:
                              sms_bridge_shutdown();     break;
            case EMU_PCE:     pce_bridge_shutdown();     break;
            case EMU_SNES:    snes_bridge_shutdown();    break;
            case EMU_LYNX:    lynx_bridge_shutdown();    break;
            case EMU_GENESIS: genesis_bridge_shutdown(); break;
            default: break;
        }
        delay(50);
    }
    currentEmu = EMU_NONE;
    memset(currentRomPath, 0, 256);
    frameCounter = 0;
    turboMode    = false;
    audioSetEnabled(true);
    for (int y = 0; y < 240; y++) clearScanline(y);
    delay(100);
}

// ============================================================================
// DRAWING FUNCTIONS
// ============================================================================
void drawMenu() {
    auto cv = new fabgl::Canvas(&DisplayController);
    cv->setBrushColor(C_BLACK);
    cv->clear();

    cv->setPenColor(C_BRIGHT_CYAN);
    cv->selectFont(&fabgl::FONT_8x16);
    cv->drawText(4, 4, "RETRO-GAMER");

    cv->selectFont(&fabgl::FONT_8x8);
    const char *tabs[] = { "All", "Fav", "Rcn", "NES", "GBC", "SMS", "PCE", "GEN", "SNS", "LNX" };
    int tx = 2;
    for (int i = 0; i < VIEW_COUNT; i++) {
        if (i == viewMode) {
            cv->setBrushColor(C_BLUE);
            cv->fillRectangle(tx-1, 19, tx+30, 29);
            cv->setPenColor(C_BRIGHT_WHITE);
        } else {
            cv->setPenColor(C_GRAY);
        }
        cv->drawText(tx, 21, tabs[i]);
        cv->setBrushColor(C_BLACK);
        tx += 32;
    }

    int filteredCount = 0;
    if (viewMode == VIEW_FAVORITES) {
        for (int i = 0; i < romCount; i++) if (isFavorite(romFiles[i])) filteredCount++;
    } else if (viewMode == VIEW_RECENT) {
        filteredCount = recentCount;
    } else if (viewMode >= VIEW_NES) {
        // Pestañas por consola — mapeo viewMode → emu_type_t
        const emu_type_t emuMap[] = { EMU_NES, EMU_GB, EMU_SMS, EMU_PCE, EMU_GENESIS, EMU_SNES, EMU_LYNX };
        emu_type_t filterEmu = emuMap[viewMode - VIEW_NES];
        for (int i = 0; i < romCount; i++)
            if (getEmuType(romFiles[i]) == filterEmu) filteredCount++;
    } else {
        filteredCount = romCount;
    }
    if (filteredCount > 0) {
        cv->setPenColor(C_BRIGHT_GREEN);
        cv->drawTextFmt(260, 8, "%d/%d", selectedRom + 1, filteredCount);
    }

    cv->setPenColor(C_BRIGHT_YELLOW);
    cv->drawLine(4, 36, 316, 36);
    cv->selectFont(&fabgl::FONT_8x8);

    if (romCount == 0 && viewMode == VIEW_ALL) {
        cv->setPenColor(C_BRIGHT_RED);
        cv->drawText(30, 100, "No ROM files found!");
        cv->setPenColor(C_GRAY);
        cv->drawText(8, 118, ".nes .fc .gb .gbc .sms .gg .pce .lnx");
        cv->drawText(8, 134, ".sfc .smc .col .rom .md .gen .bin");
        cv->drawText(15, 150, "Put ROMs in /roms/{nes,gb,...}/");
    } else {
        int filtered[MAX_ROMS];
        int fCount = 0;
        if (viewMode == VIEW_FAVORITES) {
            for (int i = 0; i < romCount && fCount < MAX_ROMS; i++)
                if (isFavorite(romFiles[i])) filtered[fCount++] = i;
        } else if (viewMode == VIEW_RECENT) {
            for (int r = 0; r < recentCount && fCount < MAX_ROMS; r++) {
                for (int i = 0; i < romCount; i++) {
                    if (strcmp(romFiles[i], recentList[r]) == 0) { filtered[fCount++] = i; break; }
                }
            }
        } else if (viewMode >= VIEW_NES) {
            const emu_type_t emuMap[] = { EMU_NES, EMU_GB, EMU_SMS, EMU_PCE, EMU_GENESIS, EMU_SNES, EMU_LYNX };
            emu_type_t filterEmu = emuMap[viewMode - VIEW_NES];
            for (int i = 0; i < romCount && fCount < MAX_ROMS; i++)
                if (getEmuType(romFiles[i]) == filterEmu) filtered[fCount++] = i;
        } else {
            for (int i = 0; i < romCount && fCount < MAX_ROMS; i++) filtered[fCount++] = i;
        }

        if (fCount == 0) {
            cv->setPenColor(C_GRAY);
            if (viewMode == VIEW_FAVORITES) cv->drawText(50, 100, "No favorites yet (F=add)");
            else if (viewMode == VIEW_RECENT) cv->drawText(70, 100, "No recent games");
            else cv->drawText(70, 100, "No ROMs for this system");
        } else {
            if (selectedRom >= fCount) selectedRom = fCount - 1;
            if (selectedRom < 0)       selectedRom = 0;

            int y   = 40;
            int lh  = 10;
            int vis = 18;

            if (scrollOffset > fCount - vis) scrollOffset = fCount - vis;
            if (scrollOffset < 0)            scrollOffset = 0;
            if (selectedRom < scrollOffset)              scrollOffset = selectedRom;
            if (selectedRom >= scrollOffset + vis)       scrollOffset = selectedRom - vis + 1;

            for (int fi = scrollOffset; fi < fCount && fi < scrollOffset + vis; fi++) {
                int i = filtered[fi];
                const char *fn = strrchr(romFiles[i], '/');
                fn = fn ? fn + 1 : romFiles[i];
                emu_type_t et = getEmuType(fn);

                if (fi == selectedRom) {
                    cv->setBrushColor(C_BLUE);
                    cv->fillRectangle(4, y-1, 318, y+lh-2);
                    cv->setPenColor(C_BRIGHT_WHITE);
                } else {
                    if      (et == EMU_NES)     cv->setPenColor(C_BRIGHT_RED);
                    else if (et == EMU_SNES)    cv->setPenColor(C_BRIGHT_MAGENTA);
                    else if (et == EMU_GB)      cv->setPenColor(C_BRIGHT_GREEN);
                    else if (et == EMU_SMS)     cv->setPenColor(C_BRIGHT_CYAN);
                    else if (et == EMU_GG)      cv->setPenColor(C_BRIGHT_YELLOW);
                    else if (et == EMU_PCE)     cv->setPenColor(C_BRIGHT_WHITE);
                    else if (et == EMU_LYNX)    cv->setPenColor(C_YELLOW);
                    else                        cv->setPenColor(C_GRAY);
                }

                char line[56];
                char sn[33];
                strncpy(sn, fn, 32); sn[32] = '\0';
                bool fav = isFavorite(romFiles[i]);
                snprintf(line, sizeof(line), "%s[%-3s] %s", fav ? "*" : "", emuName(et), sn);
                cv->drawText(8, y, line);
                y += lh;
            }
            cv->setBrushColor(C_BLACK);
        }
    }

    cv->setPenColor(C_BRIGHT_YELLOW);
    cv->drawLine(4, 224, 316, 224);
    cv->setPenColor(C_GRAY);
    cv->drawText(8, 228, "Enter=Play F=Fav Tab=View ESC=Refresh");
    delete cv;
}

void drawBoot() {
    auto cv = new fabgl::Canvas(&DisplayController);
    cv->setBrushColor(C_BLACK);
    cv->clear();

    cv->setPenColor(C_BRIGHT_CYAN);
    cv->selectFont(&fabgl::FONT_8x16);
    cv->drawText(4, 4, "RETRO-GAMER");

    cv->setPenColor(C_GRAY);
    cv->drawText(20, 40, "Multi-System Retro Emulator");
    cv->drawText(20, 55, BOARD_NAME);

    cv->setPenColor(C_BRIGHT_YELLOW);
    cv->drawLine(10, 72, 310, 72);

    int y = 82;
    cv->setPenColor(C_BRIGHT_GREEN);
    cv->drawTextFmt(10, y, "CPU: ESP32 @ %dMHz", ESP.getCpuFreqMHz());      y += 14;
    cv->drawTextFmt(10, y, "PSRAM: %d KB",       ESP.getPsramSize() / 1024); y += 14;
    cv->drawTextFmt(10, y, "Free Heap: %d KB",   ESP.getFreeHeap() / 1024);  y += 14;
    cv->drawTextFmt(10, y, "SD: %s",             SD.cardSize() > 0 ? "OK" : "N/A"); y += 14;

    cv->setPenColor(C_BRIGHT_RED);     cv->drawText(10,  y, "NES ");
    cv->setPenColor(C_BRIGHT_MAGENTA); cv->drawText(44,  y, "SNES ");
    cv->setPenColor(C_BRIGHT_GREEN);   cv->drawText(84,  y, "GB ");
    cv->setPenColor(C_BRIGHT_CYAN);    cv->drawText(110, y, "SMS ");
    cv->setPenColor(C_BRIGHT_WHITE);   cv->drawText(144, y, "PCE ");
    cv->setPenColor(C_YELLOW);         cv->drawText(178, y, "Lynx ");
    cv->setPenColor(C_BRIGHT_RED);     cv->drawText(210, y, " MD ");
    y += 18;

    cv->setPenColor(C_BRIGHT_YELLOW);
    cv->drawLine(10, y, 310, y);
    y += 8;

    cv->setPenColor(C_GRAY);
    cv->drawText(10, y,      "Arrows/WASD=Move Z=A X=B");
    cv->drawText(10, y + 14, "Enter=Start RShift=Select Esc=Menu");

    delete cv;
    delay(2500);
}

void drawInGameMenu() {
    auto cv = new fabgl::Canvas(&DisplayController);
    int bx = 50, by = 30, bw = 220, bh = 195;

    cv->setBrushColor(C_BLACK);
    cv->fillRectangle(bx, by, bx + bw, by + bh);
    cv->setPenColor(C_BRIGHT_CYAN);
    cv->drawRectangle(bx,     by,     bx + bw,     by + bh);
    cv->drawRectangle(bx + 1, by + 1, bx + bw - 1, by + bh - 1);

    cv->setPenColor(C_BRIGHT_YELLOW);
    cv->drawText(bx + 50, by + 8, "== PAUSE ==");

    const char *items[IGMENU_COUNT];
    char turboBuf[24], saveBuf[24], loadBuf[24], soundBuf[24], palBuf[32], rotBuf[32];

    const char *fskipLabels[] = {"Auto", "1/2", "1/3", "1/4"};
    snprintf(turboBuf, sizeof(turboBuf), "Frameskip: %s", fskipLabels[fskip < 4 ? fskip : 0]);
    snprintf(saveBuf,  sizeof(saveBuf),  "Save State [%d]",  saveSlot);
    snprintf(loadBuf,  sizeof(loadBuf),  "Load State [%d]",  saveSlot);
    snprintf(soundBuf, sizeof(soundBuf), "Sound: %s",        g_sound_enabled ? "ON" : "OFF");

    if (currentEmu == EMU_GB)
        snprintf(palBuf, sizeof(palBuf), "Palette: %s", gb_bridge_get_palette_name(gb_bridge_get_palette()));
    else if (currentEmu == EMU_NES)
        snprintf(palBuf, sizeof(palBuf), "Palette: %s", (const char*)pgm_read_ptr(&nes_palette_names[nesCurrentPalette]));
    else
        snprintf(palBuf, sizeof(palBuf), "Palette: N/A");

    if (currentEmu == EMU_LYNX)
        snprintf(rotBuf, sizeof(rotBuf), "Rotate: %s", (const char*)pgm_read_ptr(&lynxRotNames[lynxRotated ? 1 : 0]));
    else
        snprintf(rotBuf, sizeof(rotBuf), "Rotate: N/A");

    items[IGMENU_RESUME]     = "Resume";
    items[IGMENU_TURBO]      = turboBuf;
    items[IGMENU_SOUND]      = soundBuf;
    items[IGMENU_RESET]      = "Reset Game";
    items[IGMENU_QUIT]       = "Quit to Menu";
    items[IGMENU_SAVE_STATE] = saveBuf;
    items[IGMENU_LOAD_STATE] = loadBuf;
    items[IGMENU_PALETTE]    = (currentEmu == EMU_GB || currentEmu == EMU_NES) ? palBuf : NULL;
    items[IGMENU_ROTATE]     = (currentEmu == EMU_LYNX)                         ? rotBuf : NULL;

    int iy    = by + 30;
    int drawn = 0;
    for (int i = 0; i < IGMENU_COUNT; i++) {
        if (items[i] == NULL) continue;
        if (drawn == igMenuSel) {
            cv->setBrushColor(C_BLUE);
            cv->fillRectangle(bx + 6, iy - 1, bx + bw - 6, iy + 14);
            cv->setPenColor(C_BRIGHT_WHITE);
        } else {
            cv->setPenColor(C_GRAY);
        }
        cv->drawText(bx + 16, iy, items[i]);
        cv->setBrushColor(C_BLACK);
        iy += 18;
        drawn++;
    }

    cv->selectFont(&fabgl::FONT_8x8);
    cv->setPenColor(C_BRIGHT_GREEN);
    cv->drawText(bx + 4, by + bh - 16, " ^v:Sel <>:Slot ENTER:OK");
    delete cv;
}

// ============================================================================
// INPUT
// ============================================================================
#define BTN_UP     0x0001
#define BTN_DOWN   0x0002
#define BTN_LEFT   0x0004
#define BTN_RIGHT  0x0008
#define BTN_A      0x0010
#define BTN_B      0x0020
#define BTN_START  0x0040
#define BTN_SEL    0x0080
#define BTN_1      0x0100
#define BTN_2      0x0200
#define BTN_3      0x0400
#define BTN_4      0x0800
#define BTN_5      0x1000
#define BTN_6      0x2000
#define BTN_7      0x4000
#define BTN_8      0x8000
#define BTN_9      0x10000
#define BTN_0      0x20000
#define BTN_STAR   0x40000
#define BTN_POUND  0x80000

bool bluepad32_has_gamepad_local() {
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
    return bluepad32_has_gamepad();
#else
    return false;
#endif
}

// readButtons llama internamente a bluepad32_update() — no llamar desde fuera
uint32_t readButtons() {
    uint32_t b = 0;
#ifdef TECLADO
    if (Kbd) {
        if (Kbd->isVKDown(fabgl::VK_UP)    || Kbd->isVKDown(fabgl::VK_w) || Kbd->isVKDown(fabgl::VK_W)) b |= BTN_UP;
        if (Kbd->isVKDown(fabgl::VK_DOWN)  || Kbd->isVKDown(fabgl::VK_s) || Kbd->isVKDown(fabgl::VK_S)) b |= BTN_DOWN;
        if (Kbd->isVKDown(fabgl::VK_LEFT)  || Kbd->isVKDown(fabgl::VK_a) || Kbd->isVKDown(fabgl::VK_A)) b |= BTN_LEFT;
        if (Kbd->isVKDown(fabgl::VK_RIGHT) || Kbd->isVKDown(fabgl::VK_d) || Kbd->isVKDown(fabgl::VK_D)) b |= BTN_RIGHT;
        if (Kbd->isVKDown(fabgl::VK_z)     || Kbd->isVKDown(fabgl::VK_Z) || Kbd->isVKDown(fabgl::VK_SPACE)) b |= BTN_A;
        if (Kbd->isVKDown(fabgl::VK_x)     || Kbd->isVKDown(fabgl::VK_X) || Kbd->isVKDown(fabgl::VK_LCTRL)) b |= BTN_B;
        if (Kbd->isVKDown(fabgl::VK_RETURN))  b |= BTN_START;
        if (Kbd->isVKDown(fabgl::VK_RSHIFT))  b |= BTN_SEL;
        if (Kbd->isVKDown(fabgl::VK_1)) b |= BTN_1;
        if (Kbd->isVKDown(fabgl::VK_2)) b |= BTN_2;
        if (Kbd->isVKDown(fabgl::VK_3)) b |= BTN_3;
        if (Kbd->isVKDown(fabgl::VK_4)) b |= BTN_4;
        if (Kbd->isVKDown(fabgl::VK_5)) b |= BTN_5;
        if (Kbd->isVKDown(fabgl::VK_6)) b |= BTN_6;
        if (Kbd->isVKDown(fabgl::VK_7)) b |= BTN_7;
        if (Kbd->isVKDown(fabgl::VK_8)) b |= BTN_8;
        if (Kbd->isVKDown(fabgl::VK_9)) b |= BTN_9;
        if (Kbd->isVKDown(fabgl::VK_0)) b |= BTN_0;
        if (Kbd->isVKDown(fabgl::VK_MINUS))  b |= BTN_STAR;
        if (Kbd->isVKDown(fabgl::VK_EQUALS)) b |= BTN_POUND;
    }
#endif
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
    //bluepad32_update();
    b |= gp_buttons;
#endif
    return b;
}

uint32_t readButtonsRotated() {
    uint32_t raw = readButtons();
    uint32_t b   = raw & ~(BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT);
    if (raw & BTN_UP)    b |= BTN_LEFT;
    if (raw & BTN_DOWN)  b |= BTN_RIGHT;
    if (raw & BTN_LEFT)  b |= BTN_DOWN;
    if (raw & BTN_RIGHT) b |= BTN_UP;
    return b;
}

// Button mapping helpers
uint32_t toNES(uint32_t b) {
    uint32_t n = 0;
    if(b&BTN_UP)    n|=NES_BTN_UP;    if(b&BTN_DOWN)  n|=NES_BTN_DOWN;
    if(b&BTN_LEFT)  n|=NES_BTN_LEFT;  if(b&BTN_RIGHT) n|=NES_BTN_RIGHT;
    if(b&BTN_A)     n|=NES_BTN_A;     if(b&BTN_B)     n|=NES_BTN_B;
    if(b&BTN_START) n|=NES_BTN_START; if(b&BTN_SEL)   n|=NES_BTN_SELECT;
    return n;
}

uint32_t toGB(uint32_t b) {
    uint32_t g = 0;
    if(b&BTN_UP)    g|=GB_BTN_UP;     if(b&BTN_DOWN)  g|=GB_BTN_DOWN;
    if(b&BTN_LEFT)  g|=GB_BTN_LEFT;   if(b&BTN_RIGHT) g|=GB_BTN_RIGHT;
    if(b&BTN_A)     g|=GB_BTN_A;      if(b&BTN_B)     g|=GB_BTN_B;
    if(b&BTN_START) g|=GB_BTN_START;  if(b&BTN_SEL)   g|=GB_BTN_SELECT;
    return g;
}

uint32_t toSMS(uint32_t b) {
    uint32_t s = 0;
    if(b&BTN_UP)    s|=SMS_BTN_UP;    if(b&BTN_DOWN)  s|=SMS_BTN_DOWN;
    if(b&BTN_LEFT)  s|=SMS_BTN_LEFT;  if(b&BTN_RIGHT) s|=SMS_BTN_RIGHT;
    if(b&BTN_A)     s|=SMS_BTN_A;     if(b&BTN_B)     s|=SMS_BTN_B;
    if(b&BTN_START) s|=SMS_BTN_START;
    if(b&BTN_1)     s|=SMS_BTN_1;     if(b&BTN_2)     s|=SMS_BTN_2;
    if(b&BTN_3)     s|=SMS_BTN_3;     if(b&BTN_4)     s|=SMS_BTN_4;
    if(b&BTN_5)     s|=SMS_BTN_5;     if(b&BTN_6)     s|=SMS_BTN_6;
    if(b&BTN_7)     s|=SMS_BTN_7;     if(b&BTN_8)     s|=SMS_BTN_8;
    if(b&BTN_9)     s|=SMS_BTN_9;     if(b&BTN_0)     s|=SMS_BTN_0;
    if(b&BTN_STAR)  s|=SMS_BTN_STAR;  if(b&BTN_POUND) s|=SMS_BTN_POUND;
    return s;
}

uint32_t toPCE(uint32_t b) {
    uint32_t p = 0;
    if(b&BTN_UP)    p|=PCE_BTN_UP;    if(b&BTN_DOWN)  p|=PCE_BTN_DOWN;
    if(b&BTN_LEFT)  p|=PCE_BTN_LEFT;  if(b&BTN_RIGHT) p|=PCE_BTN_RIGHT;
    if(b&BTN_A)     p|=PCE_BTN_A;     if(b&BTN_B)     p|=PCE_BTN_B;
    if(b&BTN_START) p|=PCE_BTN_RUN;   if(b&BTN_SEL)   p|=PCE_BTN_SELECT;
    return p;
}

uint32_t toSNES(uint32_t b) {
    uint32_t s = 0;
    if(b&BTN_UP)    s|=SNES_BTN_UP;   if(b&BTN_DOWN)  s|=SNES_BTN_DOWN;
    if(b&BTN_LEFT)  s|=SNES_BTN_LEFT; if(b&BTN_RIGHT) s|=SNES_BTN_RIGHT;
    if(b&BTN_A)     s|=SNES_BTN_A;    if(b&BTN_B)     s|=SNES_BTN_B;
    if(b&BTN_START) s|=SNES_BTN_START;if(b&BTN_SEL)   s|=SNES_BTN_SELECT;
    return s;
}

uint32_t toLynx(uint32_t b) {
    uint32_t l = 0;
    if(b&BTN_UP)    l|=LYNX_BTN_UP;   if(b&BTN_DOWN)  l|=LYNX_BTN_DOWN;
    if(b&BTN_LEFT)  l|=LYNX_BTN_LEFT; if(b&BTN_RIGHT) l|=LYNX_BTN_RIGHT;
    if(b&BTN_A)     l|=LYNX_BTN_A;    if(b&BTN_B)     l|=LYNX_BTN_B;
    if(b&BTN_START) l|=LYNX_BTN_OPT2; if(b&BTN_SEL)   l|=LYNX_BTN_OPT1;
    return l;
}

uint32_t toGenesis(uint32_t b) {
    uint32_t g = 0;
    if(b&BTN_UP)    g|=GEN_BTN_UP;    if(b&BTN_DOWN)  g|=GEN_BTN_DOWN;
    if(b&BTN_LEFT)  g|=GEN_BTN_LEFT;  if(b&BTN_RIGHT) g|=GEN_BTN_RIGHT;
    if(b&BTN_A)     g|=GEN_BTN_A;     if(b&BTN_B)     g|=GEN_BTN_B;
    if(b&BTN_START) g|=GEN_BTN_START;
    return g;
}

// ============================================================================
// LOAD & RUN ROM
// ============================================================================
bool loadROM(const char *path) {
    const char *actualPath = path;
    char convertedPath[260] = {0};

    FILE *test = fopen(actualPath, "rb");
    if (!test) {
        snprintf(convertedPath, sizeof(convertedPath), "/sd%s", path);
        test = fopen(convertedPath, "rb");
        if (test) actualPath = convertedPath;
    }
    if (!test) { DBG_ERROR("ROM", "File not found: %s", path); return false; }
    fclose(test);

    emu_type_t emu = getEmuType(actualPath);
    int ret = -1;
    audioSetEnabled(true);

    if (currentEmu != EMU_NONE) {
        switch(currentEmu) {
            case EMU_NES:     nes_bridge_shutdown();     break;
            case EMU_GB:      gb_bridge_shutdown();      break;
            case EMU_SMS: case EMU_GG: sms_bridge_shutdown(); break;
            case EMU_PCE:     pce_bridge_shutdown();     break;
            case EMU_SNES:    snes_bridge_shutdown();    break;
            case EMU_LYNX:    lynx_bridge_shutdown();    break;
            case EMU_SG1000: case EMU_COLECO: sms_bridge_shutdown(); break;
            case EMU_GENESIS: genesis_bridge_shutdown(); break;
            default: break;
        }
        currentEmu = EMU_NONE;
    }

    switch(emu) {
        case EMU_NES:
            if(!nes_bridge_init(NES_AUDIO_SAMPLE_RATE)) return false;
            ret = nes_bridge_load_rom(actualPath);
            if (ret == 0) updateNESPalette();
            break;
        case EMU_GB:
            if(!gb_bridge_init(GB_AUDIO_SAMPLE_RATE)) return false;
            ret = gb_bridge_load_rom(actualPath);
            break;
        case EMU_SMS: case EMU_GG:
            if(!sms_bridge_init(SMS_AUDIO_SAMPLE_RATE)) return false;
            ret = sms_bridge_load_rom(actualPath);
            break;
        case EMU_PCE:
            if(!pce_bridge_init(PCE_AUDIO_SAMPLE_RATE)) return false;
            ret = pce_bridge_load_rom(actualPath);
            break;
        case EMU_SNES:
            if(!snes_bridge_init(SNES_AUDIO_SAMPLE_RATE)) return false;
            ret = snes_bridge_load_rom(actualPath);
            break;
        case EMU_LYNX:
            if(!lynx_bridge_init(LYNX_AUDIO_SAMPLE_RATE)) return false;
            ret = lynx_bridge_load_rom(actualPath);
            lynxRotated = false;
            break;
        case EMU_SG1000: case EMU_COLECO:
            if(!sms_bridge_init(SMS_AUDIO_SAMPLE_RATE)) return false;
            ret = sms_bridge_load_rom(actualPath);
            break;
        case EMU_GENESIS:
            if(!genesis_bridge_init(GENESIS_AUDIO_SAMPLE_RATE)) return false;
            ret = genesis_bridge_load_rom(actualPath);
            break;
        default: return false;
    }

    if (ret != 0) { DBG_ERROR("ROM", "Load failed (ret=%d): %s", ret, actualPath); return false; }
    currentEmu = emu;
    strncpy(currentRomPath, path, 255);
    currentRomPath[255] = '\0';
    turboMode    = false;
    frameCounter = 0;
    return true;
}

// ============================================================================
// EMULATOR TASK
// FIX: stack en PSRAM (32KB), creado con xTaskCreateStaticPinnedToCore
// FIX: genesis run_frame movido ANTES del blit
// ============================================================================
void EmulatorTask(void *pvParameters) {
    while (1) {
        if (appState == STATE_EMULATING) {
            uint32_t rawBtns = readButtons();
            emu_type_t emu = currentEmu;
            uint32_t btns = (emu == EMU_LYNX && lynxRotated) ? readButtonsRotated() : rawBtns;

            // Frameskip: fskip=0 (Auto) → renderiza frames pares para emuladores lentos
            // que no llegan a 60fps, sin pantalla negra. fskip=N → salta N frames entre renders.
            bool shouldRender;
            if (fskip == 0) {
                // Auto: renderiza 1 de cada 2 frames para PCE/SNES (lentos),
                // prácticamente todos para NES/GB/SMS (rápidos)
                shouldRender = (frameCounter % 2 == 0);
            } else {
                shouldRender = (frameCounter % (fskip + 1) == 0);
            }

            static uint32_t tEmuUs=0, tBlitUs=0, tFrames=0, tReport=0;
            uint32_t t0 = micros(), t1, t2;

            switch (emu) {
                case EMU_NES: {
                    nes_bridge_set_input(toNES(btns));
                    t1 = micros();
                    uint8_t *fb = nes_bridge_run_frame(shouldRender);
                    t2 = micros(); tEmuUs += t2 - t1;
                    if (shouldRender) { blitNES(fb); tBlitUs += micros() - t2; }
                    int nSamples = 0;
                    int16_t *aBuf = nes_bridge_get_audio(&nSamples);
                    if (aBuf && nSamples > 0) audioFeedSamples(aBuf, nSamples);
                    break;
                }
                case EMU_GB: {
                    gb_bridge_set_input(toGB(btns));
                    t1 = micros();
                    gb_bridge_run_frame(shouldRender);
                    t2 = micros(); tEmuUs += t2 - t1;
                    if (shouldRender) { blitGB(gb_bridge_get_framebuffer()); tBlitUs += micros() - t2; }
                    int nSamples = 0;
                    int16_t *aBuf = gb_bridge_get_audio(&nSamples);
                    if (aBuf && nSamples > 0) audioFeedSamples(aBuf, nSamples);
                    break;
                }
                case EMU_SMS: case EMU_GG: case EMU_SG1000: case EMU_COLECO: {
                    sms_bridge_set_input(toSMS(btns));
                    sms_bridge_run_frame(shouldRender);
                    if (shouldRender) {
                        int w, h;
                        uint8_t  *fb  = sms_bridge_get_framebuffer(&w, &h);
                        uint16_t *pal = sms_bridge_get_palette();
                        blitSMS(fb, pal, w, h);
                    }
                    int16_t *aL, *aR; int nSamples;
                    sms_bridge_get_audio(&aL, &aR, &nSamples);
                    if (aL && aR && nSamples > 0) audioFeedStereoMixed(aL, aR, nSamples);
                    break;
                }
                case EMU_PCE: {
                    pce_bridge_set_input(toPCE(btns));
                    t1 = micros();
                    pce_bridge_run_frame();
                    t2 = micros(); tEmuUs += t2 - t1;
                    if (shouldRender) {
                        int w, h;
                        uint8_t  *fb  = pce_bridge_get_framebuffer(&w, &h);
                        uint16_t *pal = pce_bridge_get_palette();
                        blitPCE(fb, pal, w, h);
                        tBlitUs += micros() - t2;
                    }
                    int nSamples = 0;
                    int16_t *aBuf = pce_bridge_get_audio(&nSamples);
                    if (aBuf && nSamples > 0) audioFeedSamples(aBuf, nSamples);
                    break;
                }
                case EMU_SNES: {
                    snes_bridge_set_input(toSNES(btns));
                    snes_bridge_run_frame(shouldRender);
                    if (shouldRender) {
                        int w, h;
                        uint16_t *fb = snes_bridge_get_framebuffer(&w, &h);
                        blitSNES(fb, w, h);
                    }
                    int nSamples = 0;
                    int16_t *aBuf = snes_bridge_get_audio(&nSamples);
                    if (aBuf && nSamples > 0) audioFeedSamples(aBuf, nSamples);
                    break;
                }
                case EMU_LYNX: {
                    uint32_t lynxBtns = lynxRotated ? toLynx(readButtonsRotated()) : toLynx(rawBtns);
                    lynx_bridge_set_input(lynxBtns);
                    lynx_bridge_run_frame(shouldRender);
                    if (shouldRender) blitLynx(lynx_bridge_get_framebuffer());
                    int nSamples = 0;
                    int16_t *aBuf = lynx_bridge_get_audio(&nSamples);
                    if (aBuf && nSamples > 0) audioFeedSamples(aBuf, nSamples);
                    break;
                }
                case EMU_GENESIS: {
                    genesis_bridge_set_input(toGenesis(btns));
                    // FIX: run_frame ANTES del blit (en el original era al revés)
                    genesis_bridge_run_frame(shouldRender);
                    if (shouldRender) {
                        int w, h;
                        uint8_t *fb = genesis_bridge_get_framebuffer(&w, &h);
                        blitGenesis(fb, w, h);
                    }
                    int nSamples = 0;
                    int16_t *aBuf = genesis_bridge_get_audio(&nSamples);
                    if (aBuf && nSamples > 0) audioFeedSamples(aBuf, nSamples);
                    break;
                }
                default: break;
            }

            tFrames++;
            if (micros() - tReport > 5000000) {
                Serial.printf("[PERF] frames=%lu emu=%luus/f blit=%luus/f total=%luus/f fps=%.1f\n",
                    tFrames,
                    tFrames ? tEmuUs/tFrames : 0,
                    tFrames ? tBlitUs/tFrames : 0,
                    tFrames ? (micros()-tReport)/tFrames : 0,
                    tFrames * 1000000.0f / (micros() - tReport));
                tEmuUs = tBlitUs = tFrames = 0;
                tReport = micros();
            }

            frameCounter++;

            // Abrir menú en juego: botón HOME/SYSTEM del gamepad o ESC de teclado
            {
                bool openMenu = false;
#ifdef TECLADO
                if (Kbd && Kbd->isVKDown(fabgl::VK_ESCAPE)) openMenu = true;
#endif
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
                static uint8_t lastMiscIngame = 0xff;
                static uint32_t lastBtIngame = 0;
                for (int _gi = 0; _gi < BP32_MAX_GAMEPADS; _gi++) {
                    ControllerPtr _ctl = bp32_gamepads[_gi];
                    if (_ctl && _ctl->isConnected()) {
                        uint8_t curMisc = _ctl->miscButtons();
                        uint32_t curBt  = _ctl->buttons();
                        // Abrir menú con SELECT+START simultáneos (flanco)
                        bool selStart = (curBt & 0x0030) == 0x0030; // bits típicos de Select+Start
                        // O con cualquier misc button en flanco de subida
                        uint8_t menuBits = MISC_BUTTON_SYSTEM | MISC_BUTTON_BACK;
                        bool miscEdge = (curMisc & menuBits) && !(lastMiscIngame & menuBits) && lastMiscIngame != 0xff;
                        if (miscEdge || selStart) openMenu = true;
                        lastMiscIngame = curMisc;
                        lastBtIngame   = curBt;
                    }
                }
#endif
                if (openMenu) {
                    vTaskDelay(pdMS_TO_TICKS(200));
                    igMenuSel = 0;
                    appState  = STATE_INGAME_MENU;
                    drawInGameMenu();
                }
            }

            //vTaskDelay(pdMS_TO_TICKS(1));

        } else {
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }
}
// ============================================================================
// TAREA FREERTOS: PROCESAMIENTO DEL MANDO (EJECUTA EN CORE 0)
// ============================================================================
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
void BluepadTask(void *pvParameters) {
    Serial.println("[FreeRTOS] BluepadTask iniciada en Core 0");

    while (1) {
        // Actualiza el estado del stack Bluepad32
        BP32.update();

        uint32_t current_buttons = 0;

        // Recorremos los mandos registrados
        for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
            ControllerPtr ctl = bp32_gamepads[i];
            if (!ctl || !ctl->isConnected()) continue;

            // Cruceta Digital (D-Pad)
            uint8_t dpad = ctl->dpad();
            if (dpad & DPAD_UP)    current_buttons |= BTN_UP;
            if (dpad & DPAD_DOWN)  current_buttons |= BTN_DOWN;
            if (dpad & DPAD_LEFT)  current_buttons |= BTN_LEFT;
            if (dpad & DPAD_RIGHT) current_buttons |= BTN_RIGHT;

            // Stick Analógico Izquierdo
            int16_t lx = ctl->axisX();
            int16_t ly = ctl->axisY();
            if (ly < -GP_AXIS_THRESHOLD) current_buttons |= BTN_UP;
            if (ly >  GP_AXIS_THRESHOLD) current_buttons |= BTN_DOWN;
            if (lx < -GP_AXIS_THRESHOLD) current_buttons |= BTN_LEFT;
            if (lx >  GP_AXIS_THRESHOLD) current_buttons |= BTN_RIGHT;

            // Botones de acción principales
            if (ctl->a()) current_buttons |= BTN_A;
            if (ctl->b()) current_buttons |= BTN_B;
            if (ctl->x()) current_buttons |= BTN_1;
            if (ctl->y()) current_buttons |= BTN_2;

            // Botones del sistema / menú
            uint16_t misc = ctl->miscButtons();
            if (misc & MISC_BUTTON_BACK)   current_buttons |= BTN_SEL;
            if (misc & MISC_BUTTON_HOME)   current_buttons |= BTN_START;
            if (misc & MISC_BUTTON_SYSTEM) current_buttons |= BTN_START;

            // Gatillos / Supriores (L1, R1)
            if (ctl->l1()) current_buttons |= BTN_3;
            if (ctl->r1()) current_buttons |= BTN_4;
        }

        // Actualización atómica de la variable global de estado
        gp_buttons = current_buttons;

        // Muestreo constante cada 10 ms (100 Hz)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif
// ============================================================================
// SETUP
// NOTA: El stack de tareas FreeRTOS en ESP32/Xtensa DEBE estar en DRAM interna.
// PSRAM no funciona como stack (StoreProhibited en accesos de la CPU al stack).
// Usamos xTaskCreatePinnedToCore con un tamaño justo y sin malgastar DRAM.
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(300);
    DBG_INFO("Boot", "=== RETRO-GAMER v2.0 ===");

    // PRIMERO: verificar PSRAM antes de cualquier alloc
    if (!ESP.getPsramSize()) {
        Serial.println("[BOOT] FATAL: PSRAM not found! Halting.");
        while(1) delay(1000);
    }


    // TERCERO: alloc de todas las variables grandes en PSRAM
    #define PSRAM_ALLOC(ptr, type, count) \
        ptr = (type*)heap_caps_calloc(count, sizeof(type), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); \
        if (!ptr) { Serial.printf("[BOOT] PSRAM alloc failed: %s\n", #ptr); while(1) delay(1000); }

    PSRAM_ALLOC(nesPalette222,       uint8_t,   256);
    PSRAM_ALLOC(nesPaletteRawPixels, uint8_t,    64);
    PSRAM_ALLOC(smsPaletteRawPixels, uint8_t,    32);
    PSRAM_ALLOC(smsLastPal,          uint16_t,   32);
    PSRAM_ALLOC(cramRaw,             uint8_t,    64);
    PSRAM_ALLOC(lastCRAM565,         uint16_t,   64);
    PSRAM_ALLOC(romFiles,            char*,      MAX_ROMS);
    PSRAM_ALLOC(currentRomPath,      char,       256);
    PSRAM_ALLOC(favoritesList,       char*,      MAX_FAVORITES);
    PSRAM_ALLOC(recentList,          char*,      MAX_RECENT);

    // Los buffers de audio s_audioDmaBuf se allocan lazy en audioFeedSafe
    // usando heap_caps con MALLOC_CAP_INTERNAL la primera vez que se necesitan.
    #undef PSRAM_ALLOC
BaseType_t taskOk = xTaskCreatePinnedToCore(
      EmulatorTask,
      "EmulatorTask",
      4096,
      NULL,
      5,
      NULL,
      1
  );
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
    initBluePad32();
#endif
    initVGA();
#ifdef TECLADO
    initPS2();
#endif
xTaskCreatePinnedToCore(
        BluepadTask,     // Función de la tarea
        "BluepadTask",   // Nombre identificativo
        3072,            // Tamaño de stack (bytes/palabras según RTOS)
        NULL,            // Parámetros de entrada
        3,               // Prioridad
        NULL,            // Handle de la tarea
        0                // CORE 0
    );
    bool sdOk = initSD();
    if (sdOk) { createAllSaveDirectories(); ensureSaveDir(); }

    updateNESPalette();
    audioInit();
    drawBoot();

    if (sdOk) { scanAllRoms(); }
    if (sdOk) { loadFavorites(); }
    if (sdOk) { loadRecent(); }

  
  Serial.printf("[BOOT] EmulatorTask: %s\n", taskOk == pdPASS ? "OK" : "FALLO");

    appState = STATE_FILE_SELECT;
    drawMenu();
}

// ============================================================================
// MAIN LOOP
// FIX: variables bool inicializadas siempre a false antes de los #ifdef
//      (antes podían quedar sin inicializar → UB → StoreProhibited)
// ============================================================================
void loop() {
    switch (appState) {

        case STATE_FILE_SELECT: {
            static unsigned long lastInput = 0;
            unsigned long now = millis();

            // FIX: inicializar a false siempre
            bool inputAvail = false;

#ifdef TECLADO
            inputAvail = (Kbd != nullptr);
#endif
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
            //BP32.update();
            if (!inputAvail) inputAvail = bluepad32_has_gamepad();
#endif
            if (!inputAvail || now - lastInput < 150) { delay(10); break; }

            // FIX: NO llamar bluepad32_update() aquí — readButtons() ya lo hace internamente
            uint32_t menuBtns = readButtons();
            bool redraw = false;

            if ((menuBtns & BTN_UP) && selectedRom > 0) {
                selectedRom--;
                if (selectedRom < scrollOffset) scrollOffset = selectedRom;
                redraw = true; lastInput = now;
            }
            // Calcular fCount para el límite correcto según la tab activa
            int curFCount = romCount;
            if (viewMode == VIEW_FAVORITES) {
                curFCount = 0;
                for (int i = 0; i < romCount; i++) if (isFavorite(romFiles[i])) curFCount++;
            } else if (viewMode == VIEW_RECENT) {
                curFCount = recentCount;
            } else if (viewMode >= VIEW_NES) {
                const emu_type_t emuMap[] = { EMU_NES, EMU_GB, EMU_SMS, EMU_PCE, EMU_GENESIS, EMU_SNES, EMU_LYNX };
                emu_type_t filterEmu = emuMap[viewMode - VIEW_NES];
                curFCount = 0;
                for (int i = 0; i < romCount; i++)
                    if (getEmuType(romFiles[i]) == filterEmu) curFCount++;
            }
            if ((menuBtns & BTN_DOWN) && selectedRom < curFCount - 1) {
                selectedRom++;
                if (selectedRom >= scrollOffset + 18) scrollOffset = selectedRom - 17;
                redraw = true; lastInput = now;
            }
            if ((menuBtns & BTN_START) && romCount > 0) {
                int realRomIndex = -1;

                if (viewMode == VIEW_FAVORITES) {
                    int fCount = 0;
                    for (int i = 0; i < romCount && fCount < MAX_ROMS; i++) {
                        if (isFavorite(romFiles[i])) {
                            if (fCount == selectedRom) { realRomIndex = i; break; }
                            fCount++;
                        }
                    }
                } else if (viewMode == VIEW_RECENT) {
                    int fCount = 0;
                    for (int r = 0; r < recentCount && fCount < MAX_ROMS; r++) {
                        for (int i = 0; i < romCount; i++) {
                            if (strcmp(romFiles[i], recentList[r]) == 0) {
                                if (fCount == selectedRom) { realRomIndex = i; break; }
                                fCount++; break;
                            }
                        }
                        if (realRomIndex != -1) break;
                    }
                } else if (viewMode >= VIEW_NES) {
                    const emu_type_t emuMap[] = { EMU_NES, EMU_GB, EMU_SMS, EMU_PCE, EMU_GENESIS, EMU_SNES, EMU_LYNX };
                    emu_type_t filterEmu = emuMap[viewMode - VIEW_NES];
                    int fCount = 0;
                    for (int i = 0; i < romCount && fCount < MAX_ROMS; i++) {
                        if (getEmuType(romFiles[i]) == filterEmu) {
                            if (fCount == selectedRom) { realRomIndex = i; break; }
                            fCount++;
                        }
                    }
                } else {
                    realRomIndex = selectedRom;
                }

                if (realRomIndex < 0 || realRomIndex >= romCount) {
                    auto c = new fabgl::Canvas(&DisplayController);
                    c->setPenColor(C_BRIGHT_RED);
                    c->drawText(80, 130, "ROM not found!");
                    delete c;
                    delay(2000);
                    break;
                }

                auto cv = new fabgl::Canvas(&DisplayController);
                cv->setBrushColor(C_BLACK); cv->clear();
                cv->setPenColor(C_BRIGHT_YELLOW);
                cv->drawTextFmt(60, 110, "Loading %s...", emuName(getEmuType(romFiles[realRomIndex])));
                delete cv;

                if (loadROM(romFiles[realRomIndex])) {
                    addToRecent(romFiles[realRomIndex]);
                    for (int y = 0; y < 240; y++) clearScanline(y);
                    appState = STATE_EMULATING;
                } else {
                    auto c = new fabgl::Canvas(&DisplayController);
                    c->setPenColor(C_BRIGHT_RED);
                    c->drawText(80, 130, "Load failed!");
                    delete c;
                    delay(2000);
                    redraw = true;
                }
                lastInput = now;
            }

            // FIX: escPressed inicializado a false siempre (antes: redeclaración con UB)
            bool escPressed = false;
#ifdef TECLADO
            escPressed = (Kbd && Kbd->isVKDown(fabgl::VK_ESCAPE));
#endif
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
            if (!escPressed) {
                for (int _gi = 0; _gi < BP32_MAX_GAMEPADS; _gi++) {
                    ControllerPtr _ctl = bp32_gamepads[_gi];
                    if (_ctl && _ctl->isConnected() && (_ctl->miscButtons() & MISC_BUTTON_SYSTEM))
                        escPressed = true;
                }
            }
#endif
            if (escPressed) { refreshRomList(); redraw = true; lastInput = now; }

            // FIX: favPressed inicializado a false siempre
            bool favPressed = false;
#ifdef TECLADO
            favPressed = (Kbd && Kbd->isVKDown(fabgl::VK_f));
#endif
            if (!favPressed && (menuBtns & BTN_3)) favPressed = true;
            if (favPressed && romCount > 0) {
                toggleFavorite(romFiles[selectedRom]);
                redraw = true; lastInput = now;
            }

            // FIX: tabPressed inicializado a false siempre
            bool tabPressed = false;
#ifdef TECLADO
            tabPressed = (Kbd && Kbd->isVKDown(fabgl::VK_TAB));
#endif
            if (!tabPressed && (menuBtns & BTN_4)) tabPressed = true;
            if (tabPressed) {
                viewMode = (view_mode_t)((viewMode + 1) % VIEW_COUNT);
                selectedRom = 0; scrollOffset = 0;
                selectedRom = 0; scrollOffset = 0;
                redraw = true; lastInput = now;
            }

            if (redraw) drawMenu();
            delay(16);
            break;
        }

        case STATE_EMULATING: {
            delay(16);
            break;
        }

        case STATE_INGAME_MENU: {
            static unsigned long lastMenuInput = 0;
            unsigned long now = millis();

            // FIX: igInputAvail inicializado a false siempre
            bool igInputAvail = false;
#ifdef TECLADO
            igInputAvail = (Kbd != nullptr);
#endif
#ifdef BT_GAMEPAD_INPUT_BLUEPAD
            if (!igInputAvail) igInputAvail = bluepad32_has_gamepad();
#endif
            if (!igInputAvail || now - lastMenuInput < 180) { delay(10); break; }

            // FIX: NO llamar bluepad32_update() aquí — readButtons() ya lo hace
            uint32_t igBtns = readButtons();

            bool redrawIG = false;
            int  max_items = IGMENU_COUNT;
            if (currentEmu != EMU_GB  && currentEmu != EMU_NES)  max_items--;
            if (currentEmu != EMU_LYNX)                           max_items--;
            if (igMenuSel >= max_items) igMenuSel = max_items - 1;
            if (igMenuSel < 0)          igMenuSel = 0;

            if (igBtns & BTN_UP) {
                igMenuSel = (igMenuSel - 1 + max_items) % max_items;
                redrawIG = true; lastMenuInput = now;
            }
            if (igBtns & BTN_DOWN) {
                igMenuSel = (igMenuSel + 1) % max_items;
                redrawIG = true; lastMenuInput = now;
            }

            // Calcular item real (saltando los NULL)
            int real_item = -1;
            int counter   = 0;
            for (int item = 0; item < IGMENU_COUNT; item++) {
                if (item == IGMENU_PALETTE && currentEmu != EMU_GB && currentEmu != EMU_NES) continue;
                if (item == IGMENU_ROTATE  && currentEmu != EMU_LYNX) continue;
                if (counter == igMenuSel) { real_item = item; break; }
                counter++;
            }

            if (igBtns & BTN_LEFT) {
                if      (real_item == IGMENU_SAVE_STATE || real_item == IGMENU_LOAD_STATE)
                    saveSlot = (saveSlot - 1 + 4) % 4;
                else if (real_item == IGMENU_TURBO) {
                    fskip = (fskip + 1) % 4;
                } else if (real_item == IGMENU_SOUND) {
                    osd_sound_toggle();
                } else if (real_item == IGMENU_PALETTE) {
                    if (currentEmu == EMU_GB) {
                        int p = (gb_bridge_get_palette() - 1 + gb_bridge_get_palette_count()) % gb_bridge_get_palette_count();
                        gb_bridge_set_palette(p);
                    } else if (currentEmu == EMU_NES) {
                        nesCurrentPalette = (nesCurrentPalette - 1 + NES_PALETTE_COUNT) % NES_PALETTE_COUNT;
                        updateNESPalette();
                    }
                } else if (real_item == IGMENU_ROTATE && currentEmu == EMU_LYNX) {
                    lynxRotated = !lynxRotated;
                }
                redrawIG = true; lastMenuInput = now;
            }

            if (igBtns & BTN_RIGHT) {
                if      (real_item == IGMENU_SAVE_STATE || real_item == IGMENU_LOAD_STATE)
                    saveSlot = (saveSlot + 1) % 4;
                else if (real_item == IGMENU_TURBO) {
                    fskip = (fskip + 1) % 4;
                } else if (real_item == IGMENU_SOUND) {
                    osd_sound_toggle();
                } else if (real_item == IGMENU_PALETTE) {
                    if (currentEmu == EMU_GB) {
                        int p = (gb_bridge_get_palette() + 1) % gb_bridge_get_palette_count();
                        gb_bridge_set_palette(p);
                    } else if (currentEmu == EMU_NES) {
                        nesCurrentPalette = (nesCurrentPalette + 1) % NES_PALETTE_COUNT;
                        updateNESPalette();
                    }
                } else if (real_item == IGMENU_ROTATE && currentEmu == EMU_LYNX) {
                    lynxRotated = !lynxRotated;
                }
                redrawIG = true; lastMenuInput = now;
            }

            if (igBtns & BTN_B) {
                appState = STATE_EMULATING; lastMenuInput = now; break;
            }
#ifdef TECLADO
            if (Kbd && Kbd->isVKDown(fabgl::VK_ESCAPE)) {
                appState = STATE_EMULATING; lastMenuInput = now; break;
            }
#endif
            if (igBtns & BTN_A) {
                lastMenuInput = now;
                switch(real_item) {
                    case IGMENU_RESUME:
                        appState = STATE_EMULATING;
                        break;

                    case IGMENU_SAVE_STATE: {
                        auto cv = new fabgl::Canvas(&DisplayController);
                        cv->setBrushColor(C_BLACK); cv->fillRectangle(90, 110, 250, 130);
                        cv->setPenColor(C_BRIGHT_YELLOW); cv->drawText(100, 115, "Saving..."); delete cv;
                        bool ok = saveStateToSD(saveSlot);
                        cv = new fabgl::Canvas(&DisplayController);
                        cv->setBrushColor(C_BLACK); cv->fillRectangle(90, 110, 250, 130);
                        cv->setPenColor(ok ? C_BRIGHT_GREEN : C_BRIGHT_RED);
                        cv->drawText(100, 115, ok ? "Saved OK!" : "Save FAILED!"); delete cv;
                        delay(1000); redrawIG = true;
                        break;
                    }

                    case IGMENU_LOAD_STATE: {
                        auto cv = new fabgl::Canvas(&DisplayController);
                        cv->setBrushColor(C_BLACK); cv->fillRectangle(90, 110, 250, 130);
                        cv->setPenColor(C_BRIGHT_YELLOW); cv->drawText(100, 115, "Loading..."); delete cv;
                        bool ok = loadStateFromSD(saveSlot);
                        cv = new fabgl::Canvas(&DisplayController);
                        cv->setBrushColor(C_BLACK); cv->fillRectangle(90, 110, 250, 130);
                        cv->setPenColor(ok ? C_BRIGHT_GREEN : C_BRIGHT_RED);
                        cv->drawText(100, 115, ok ? "Loaded OK!" : "No save found!"); delete cv;
                        delay(1000);
                        if (ok) appState = STATE_EMULATING;
                        else    redrawIG = true;
                        break;
                    }

                    case IGMENU_TURBO:
                        fskip = (fskip + 1) % 4; redrawIG = true;
                        break;

                    case IGMENU_SOUND:
                        osd_sound_toggle(); redrawIG = true;
                        break;

                    case IGMENU_PALETTE:
                        if (currentEmu == EMU_GB) {
                            int p = (gb_bridge_get_palette() + 1) % gb_bridge_get_palette_count();
                            gb_bridge_set_palette(p);
                        } else if (currentEmu == EMU_NES) {
                            nesCurrentPalette = (nesCurrentPalette + 1) % NES_PALETTE_COUNT;
                            updateNESPalette();
                        }
                        redrawIG = true;
                        break;

                    case IGMENU_ROTATE:
                        if (currentEmu == EMU_LYNX) { lynxRotated = !lynxRotated; redrawIG = true; }
                        break;

                    case IGMENU_RESET:
                        if (currentRomPath[0] != '\0' && currentEmu != EMU_NONE) {
                            auto cv = new fabgl::Canvas(&DisplayController);
                            cv->setBrushColor(C_BLACK); cv->clear(); delete cv;
                            appState = STATE_EMULATING;
                            switch(currentEmu) {
                                case EMU_NES:     nes_bridge_shutdown();     break;
                                case EMU_GB:      gb_bridge_shutdown();      break;
                                case EMU_SMS: case EMU_GG: sms_bridge_shutdown(); break;
                                case EMU_PCE:     pce_bridge_shutdown();     break;
                                case EMU_SNES:    snes_bridge_shutdown();    break;
                                case EMU_LYNX:    lynx_bridge_shutdown();    break;
                                case EMU_SG1000: case EMU_COLECO: sms_bridge_shutdown(); break;
                                case EMU_GENESIS: genesis_bridge_shutdown(); break;
                                default: break;
                            }
                            currentEmu = EMU_NONE;
                            auto cv2 = new fabgl::Canvas(&DisplayController);
                            cv2->setBrushColor(C_BLACK); cv2->clear();
                            cv2->setPenColor(C_BRIGHT_YELLOW); cv2->drawText(100, 115, "Resetting..."); delete cv2;
                            delay(100);
                            loadROM(currentRomPath);
                        } else {
                            cleanExitToMenu(); appState = STATE_FILE_SELECT; drawMenu();
                        }
                        break;

                    case IGMENU_QUIT:
                        cleanExitToMenu();
                        appState  = STATE_FILE_SELECT;
                        selectedRom = 0; scrollOffset = 0; viewMode = VIEW_ALL;
                        drawMenu();
                        break;

                    default: break;
                }
            }

            if (redrawIG) drawInGameMenu();
            delay(16);
            break;
        }

        default: delay(1000); break;
    }
}
