#ifdef APP_TARGET_M5PAPER
// M5Paper: use M5Unified which has built-in IT8951 e-ink panel support via LovyanGFX
#include <M5Unified.h>
#include "M5Paper_Config.h"
#else
#include <M5Unified.h>
#endif
#if defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
#include <M5Cardputer.h>
#endif
#include <Preferences.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "GroveNFC.h"

using namespace grove_nfc;

namespace {
#if defined(APP_TARGET_STICKS3)
constexpr int kSdaPin = 9;
constexpr int kSclPin = 10;
constexpr uint8_t kEmuMenuVisibleCount = 5;
#elif defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
constexpr int kSdaPin = 2;
constexpr int kSclPin = 1;
constexpr uint8_t kEmuMenuVisibleCount = 5;
#elif defined(APP_TARGET_STICKCPLUS)
constexpr int kSdaPin = 32;
constexpr int kSclPin = 33;
constexpr uint8_t kEmuMenuVisibleCount = 5;
#elif defined(APP_TARGET_M5PAPER)
constexpr int kSdaPin = 25;
constexpr int kSclPin = 32;
constexpr uint8_t kEmuMenuVisibleCount = 5;
#else
constexpr int kSdaPin = 2;
constexpr int kSclPin = 1;
constexpr uint8_t kEmuMenuVisibleCount = 4;
#endif
constexpr uint32_t kI2CFreq = 400000;
#if defined(APP_TARGET_STICKS3)
constexpr uint32_t kPollIntervalMs = 120;
#elif defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
constexpr uint32_t kPollIntervalMs = 120;
#elif defined(APP_TARGET_STICKCPLUS)
constexpr uint32_t kPollIntervalMs = 140;
#elif defined(APP_TARGET_M5PAPER)
constexpr uint32_t kPollIntervalMs = 280;
#else
constexpr uint32_t kPollIntervalMs = 220;
#endif
#if defined(APP_TARGET_STICKS3)
constexpr uint32_t kReaderHoldCheckMs = 650;
#elif defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
constexpr uint32_t kReaderHoldCheckMs = 650;
#elif defined(APP_TARGET_STICKCPLUS)
constexpr uint32_t kReaderHoldCheckMs = 700;
#elif defined(APP_TARGET_M5PAPER)
constexpr uint32_t kReaderHoldCheckMs = 1000;
#else
constexpr uint32_t kReaderHoldCheckMs = 900;
#endif
constexpr uint32_t kHeartbeatMs = 2000;
constexpr uint32_t kNfcHealthCheckMs = 3000;
constexpr uint32_t kNfcReconnectMs = 1500;
#if defined(APP_TARGET_STICKS3)
constexpr uint32_t kNdefAutoPollMs = 520;
#elif defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
constexpr uint32_t kNdefAutoPollMs = 520;
#elif defined(APP_TARGET_STICKCPLUS)
constexpr uint32_t kNdefAutoPollMs = 650;
#elif defined(APP_TARGET_M5PAPER)
constexpr uint32_t kNdefAutoPollMs = 1200;
#else
constexpr uint32_t kNdefAutoPollMs = 900;
#endif
#if defined(APP_TARGET_M5PAPER)
constexpr uint32_t kReaderRecoverMs = 8000;
#else
constexpr uint32_t kReaderRecoverMs = 6000;
#endif
constexpr uint32_t kRecoverCooldownMs = 1500;
constexpr uint32_t kDiagScrollMs = 800;
constexpr uint32_t kUiScrollMs = 260;
constexpr uint32_t kReaderAnimStepUs = 8000;
constexpr uint32_t kReaderSweepUs = 820000;
// Keep the compact system-style boot sequence on button-driven displays.
// M5Paper goes directly to its touch home screen without a Diagnose splash.
constexpr bool kAutoBootDebug =
#if defined(APP_TARGET_M5PAPER)
  false
#else
  true
#endif
;
constexpr uint32_t kBootDebugShowMs = 2500;
constexpr uint8_t kSpeakerVolume = 160;
constexpr uint8_t kEmuActionCount = 2;
constexpr uint8_t kEmuDumpMaxFiles = 32;
constexpr size_t kEmuDumpMaxBytes = 4096;
constexpr size_t kIso15DumpMaxBytes = 256;
constexpr uint8_t kEmuDumpMenuBaseItems = 2;  // Load Dump + Exit
constexpr uint32_t kDumpEmuPopupMs = 1700;
constexpr const char* kEmuDumpDir = "/dumps";
constexpr const char* kEmuApSsid = "GroveNFC-Dump";
constexpr const char* kEmuApPass = "";
constexpr const char* kEmuDumpPrefNs = "emu_dump";
#if defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
constexpr uint32_t kHoldPressMs = 320;
constexpr uint32_t kNfcBootPowerSettleMs = 220;
constexpr uint8_t kNfcBootRetryCount = 5;
constexpr uint32_t kNfcBootRetryDelayMs = 140;
#elif defined(APP_TARGET_STICKS3)
constexpr uint32_t kHoldPressMs = 380;
constexpr uint32_t kNfcBootPowerSettleMs = 220;
constexpr uint8_t kNfcBootRetryCount = 5;
constexpr uint32_t kNfcBootRetryDelayMs = 140;
#elif defined(APP_TARGET_M5PAPER)
constexpr uint32_t kHoldPressMs = 500;
constexpr uint32_t kNfcBootPowerSettleMs = 220;
constexpr uint8_t kNfcBootRetryCount = 5;
constexpr uint32_t kNfcBootRetryDelayMs = 140;
#else
constexpr uint32_t kHoldPressMs = 700;
constexpr uint32_t kNfcBootPowerSettleMs = 80;
constexpr uint8_t kNfcBootRetryCount = 2;
constexpr uint32_t kNfcBootRetryDelayMs = 90;
#endif

enum class MenuPage : uint8_t {
  Reader = 0,
  ReadNDEF,
  Emulator,
  WebFiles,
  Diagnose,
  Piano,
  About,
  Count
};

enum class EmuType : uint8_t {
  MF1K = 0,
  MF4K,
  N213,
  N215,
  N216,
  ISO14B,
  Felica,
  ISO15,
  Count
};

enum class EmuConfigStage : uint8_t {
  None = 0,
  TypeMenu,
};

enum class DumpsStage : uint8_t {
  Browse = 0,
  Menu,
  Preview,
  PortalQr,
};

enum class PianoStage : uint8_t {
  Menu = 0,
  Play,
  Config,
};

constexpr uint8_t kPianoNoteCount = 8;
constexpr uint16_t kPianoFreq[kPianoNoteCount] = {523, 587, 659, 698, 784, 880, 988, 1047};
constexpr const char* kPianoNoteName[kPianoNoteCount] = {"1 Do", "2 Re", "3 Mi", "4 Fa", "5 Sol", "6 La", "7 Ti", "1' Do"};
constexpr uint32_t kPianoPollMs = 140;
constexpr uint16_t kPianoSustainToneMs = 170;
constexpr uint16_t kPianoSustainRetriggerMs = 110;
constexpr const char* kPianoPrefNs = "piano";

GroveNFC nfc(Wire);
int active_sda_pin = kSdaPin;
int active_scl_pin = kSclPin;
// Off-screen canvas for flicker-free rendering (all drawing targets this sprite, then a
// single pushSprite() DMA-blits the complete frame to the physical display).
LGFX_Sprite g_canvas;
bool in_home = true;
#ifdef APP_TARGET_M5PAPER
// E-ink refresh state
static bool s_epd_ready = false;
static int g_eink_partial_count = 0;
static uint32_t g_last_eink_full_ms = 0;
constexpr uint32_t kEinkMinFullIntervalMs = 60000;

static void drawM5PaperStatusBar() {
  const int w = g_canvas.width();
  const int bar_h = kStatusBarHeight;
  const int32_t battery = M5.Power.getBatteryLevel();

  g_canvas.fillRect(0, 0, w, bar_h, TFT_WHITE);
  g_canvas.drawLine(0, bar_h - 1, w, bar_h - 1, TFT_BLACK);
  g_canvas.setFont(&fonts::Font0);
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setTextSize(2);
  const int text_y = (bar_h - g_canvas.fontHeight()) / 2;
  g_canvas.setCursor(12, text_y);
  g_canvas.print(in_home ? "GroveNFC" : "< Back");
  if (!in_home) {
    const String brand = "GroveNFC";
    const int brand_w = g_canvas.textWidth(brand);
    g_canvas.setCursor((w - brand_w) / 2, text_y);
    g_canvas.print(brand);
  }

  const int battery_w = 34;
  const int battery_h = 18;
  const int battery_x = w - battery_w - 18;
  const int battery_y = (bar_h - battery_h) / 2;
  g_canvas.drawRect(battery_x, battery_y, battery_w, battery_h, TFT_BLACK);
  g_canvas.fillRect(battery_x + battery_w, battery_y + 5, 3, 6, TFT_BLACK);
  if (battery >= 0) {
    const int fill_w = ((battery_w - 4) * constrain(battery, 0, 100)) / 100;
    if (fill_w > 0) g_canvas.fillRect(battery_x + 2, battery_y + 2, fill_w, battery_h - 4, TFT_BLACK);
  }
}

// Push canvas to M5Paper e-ink display via M5Unified's LovyanGFX IT8951 driver
// Mirrors the minimal test that is confirmed to work: pushSprite then display().
static void pushCanvasToEink(bool force_full = false) {
  if (!s_epd_ready) return;
  drawM5PaperStatusBar();
  g_canvas.pushSprite(&M5.Display, 0, 0);
  M5.Display.display();
}
#endif

MenuPage menu_page = MenuPage::Reader;
EmuType emu_type = EmuType::N213;
EmuType emu_menu_type = EmuType::N213;
EmuConfigStage emu_config_stage = EmuConfigStage::None;
#ifdef APP_TARGET_M5PAPER
int home_index = -1;  // No tile selected until the first touch.
#else
int home_index = 0;   // The initial visible page is Reader (home item 0).
#endif
int emu_menu_index = 0;
uint8_t emu_type_cursor = 0;
uint8_t emu_type_scroll = 0;
grove_nfc::CardInfo last_card;
uint32_t last_poll_ms = 0;
bool nfc_ready = false;
bool emu_started = false;
bool emu_user_stopped = false;
uint32_t emu_last_start_retry_ms = 0;
String diagnose_report;
bool diagnose_ok = false;
uint16_t hw_ver = 0;
uint16_t fw_ver = 0;
String ndef_text;
String ndef_detail;
bool ndef_is_wifi = false;

bool wifi_popup = false;
String wifi_ssid;
String wifi_pass;
String wifi_type;
String wifi_status;
bool boot_debug_running = false;
uint32_t last_heartbeat_ms = 0;
int32_t menu_scroll_px = 0;
uint32_t last_nfc_health_ms = 0;
uint32_t last_nfc_reconnect_ms = 0;
uint32_t last_ndef_auto_ms = 0;
uint8_t ndef_fail_count = 0;
uint32_t last_reader_success_ms = 0;
uint32_t last_recover_ms = 0;
uint32_t last_diag_scroll_ms = 0;
uint32_t last_ui_scroll_ms = 0;
uint32_t last_reader_anim_us = 0;
bool btn_hold_latched = false;
bool ignore_click_after_hold = false;
String boot_notice_line;
bool emu_show_menu = false;
bool littlefs_ready = false;
String emu_dump_status = "No dump";
String emu_dump_files[kEmuDumpMaxFiles];
String emu_dump_uid_raw[kEmuDumpMaxFiles];
String emu_dump_uid_view[kEmuDumpMaxFiles];
uint8_t emu_dump_count = 0;
bool emu_type_anim_active = false;
uint32_t emu_type_anim_start_ms = 0;
uint16_t emu_type_anim_duration_ms = 240;
int8_t emu_type_anim_dir = 1;
EmuType emu_type_anim_from = EmuType::N213;
EmuType emu_type_anim_to = EmuType::N213;
uint32_t last_emu_anim_ms = 0;
bool emu_switch_apply_pending = false;

// Home page carousel animation (same style as Emulator type carousel)
bool home_anim_active = false;
uint32_t home_anim_start_ms = 0;
uint16_t home_anim_duration_ms = 240;
int8_t home_anim_dir = 1;
MenuPage home_anim_from = MenuPage::Reader;
MenuPage home_anim_to = MenuPage::Reader;

// Arrow flash: indicates which button was pressed on the home screen
bool home_arrow_flash_right = false;  // flashes when A (next) pressed
bool home_arrow_flash_left = false;   // flashes when B/prev pressed
uint32_t home_arrow_flash_until_ms = 0;
constexpr uint32_t kHomeArrowFlashMs = 90;
bool emu_ap_active = false;
WebServer emu_ap_server(80);
DNSServer emu_ap_dns;
File emu_ap_upload_file;
bool emu_ap_upload_session_active = false;
String emu_ap_uploaded_paths[kEmuDumpMaxFiles];
uint8_t emu_ap_uploaded_count = 0;
uint8_t dump_file_index = 0;
DumpsStage dumps_stage = DumpsStage::Browse;
uint8_t dumps_menu_index = 0;
bool dumps_pick_for_emu = false;
String dumps_preview_text;
size_t dumps_preview_offset = 0;
size_t m5paper_emu_hex_offset = 0;
uint8_t dumps_preview_font_level = 1;
String emu_ap_last_upload_path;
String dumps_qr_payload;
bool dumps_qr_wifi = true;
bool dumps_emu_popup_active = false;
bool m5paper_dump_actions = false;
bool m5paper_dump_delete_confirm = false;
uint32_t dumps_emu_popup_until_ms = 0;
String dumps_emu_popup_line1;
String dumps_emu_popup_line2;
String dumps_emu_popup_type;
bool ui_marquee_active = false;
bool reader_need_first_tone = false;
bool reader_14b_only = false;
uint32_t reader_last_hold_log_ms = 0;
uint8_t reader_fail_streak = 0;
String nfc_module_name = "GroveNFC";
#ifdef APP_TARGET_M5PAPER
String nfc_port_name = "Port A";
#endif
Preferences prefs;
bool emu_dump_restore_checked[static_cast<uint8_t>(EmuType::Count)] = {false};
String emu_dump_loaded_path[static_cast<uint8_t>(EmuType::Count)];
String emu_dump_loaded_uid[static_cast<uint8_t>(EmuType::Count)];
PianoStage piano_stage = PianoStage::Menu;
uint8_t piano_menu_index = 0;
uint8_t piano_config_step = 0;
String piano_card_map[kPianoNoteCount];
String piano_status = "Not configured";
String piano_last_note = "-";
int8_t about_page_idx = 0;
String piano_active_card_key;
int8_t piano_active_note_idx = -1;
uint32_t piano_last_sustain_ms = 0;

// ---- NFC Worker Task (Core 0) Infrastructure ----
// NFC I2C operations run on a dedicated FreeRTOS task on Core 0,
// so the UI animation loop on Core 1 never blocks on I2C.

SemaphoreHandle_t nfc_mutex = nullptr;
TaskHandle_t nfc_task_handle = nullptr;

// Command queue: UI thread -> NFC worker
enum class NfcCmd : uint8_t {
  None = 0,
  StartEmulation,
  StopRF,
  RunDiagnose,
  ScanNdefNow,
  Recover,
};
QueueHandle_t nfc_cmd_queue = nullptr;

// Results from NFC worker -> UI thread (protected by nfc_mutex)
struct NfcReaderResult {
  CardInfo card;
  bool got_card = false;
  bool new_result = false;  // set by worker, cleared by UI
};
NfcReaderResult nfc_reader_result;

struct NfcNdefResult {
  String text;
  String detail;
  bool ok = false;
  bool new_result = false;
};
NfcNdefResult nfc_ndef_result;

struct NfcPianoResult {
  CardInfo card;
  bool got_card = false;
  bool new_result = false;
};
NfcPianoResult nfc_piano_result;

struct NfcHealthResult {
  bool lost_connection = false;
  bool reconnected = false;
  uint16_t new_hw_ver = 0;
  uint16_t new_fw_ver = 0;
  bool need_recover = false;
  bool new_result = false;
};
NfcHealthResult nfc_health_result;

struct NfcCmdResult {
  bool ok = false;
  String report;   // for diagnose
  uint16_t hw = 0;
  uint16_t fw = 0;
  bool done = false;
};
NfcCmdResult nfc_cmd_result;

// Snapshot of UI state read by the NFC worker (set by UI, read by worker)
bool nfc_w_reader_14b_only = false;
bool nfc_w_is_reader_page = false;
bool nfc_w_is_ndef_page = false;
bool nfc_w_is_piano_page = false;
bool nfc_w_card_valid = false;
bool nfc_w_wifi_popup = false;
bool nfc_w_in_home = true;
EmuType nfc_w_emu_type = EmuType::N213;
PianoStage nfc_w_piano_stage = PianoStage::Menu;

inline bool mainButtonClicked() {
#ifdef APP_TARGET_M5PAPER
  return M5.BtnB.wasClicked();
#else
  return M5.BtnA.wasClicked();
#endif
}

inline bool mainButtonPressedFor(uint32_t ms) {
#ifdef APP_TARGET_M5PAPER
  return M5.BtnB.pressedFor(ms);
#else
  return M5.BtnA.pressedFor(ms);
#endif
}

inline bool mainButtonReleased() {
#ifdef APP_TARGET_M5PAPER
  return M5.BtnB.wasReleased();
#else
  return M5.BtnA.wasReleased();
#endif
}

struct KeyNavState {
  bool next = false;
  bool prev = false;
  bool confirm = false;
  bool back = false;
  bool cancel = false;
  bool key_b = false;
};

#if defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
KeyNavState readKeyNavState() {
  static uint32_t b_pressed_since_ms = 0;
  static bool b_hold_latched = false;

  KeyNavState nav;
  const bool changed = M5Cardputer.Keyboard.isChange();
  const bool pressed = M5Cardputer.Keyboard.isPressed();
  auto& ks = M5Cardputer.Keyboard.keysState();

  bool has_b = false;
  if (pressed) {
    for (const auto c : ks.word) {
      if (c == 'b' || c == 'B') {
        has_b = true;
        break;
      }
    }
  }

 
  if (has_b) {
    if (b_pressed_since_ms == 0) {
      b_pressed_since_ms = millis();
      b_hold_latched = false;
    } else if (!b_hold_latched && (millis() - b_pressed_since_ms >= kHoldPressMs)) {
      nav.back = true;
      nav.cancel = true;
      b_hold_latched = true;
    }

    if (changed && !b_hold_latched) {
      nav.key_b = true;
    }
  } else {
    b_pressed_since_ms = 0;
    b_hold_latched = false;
  }

  if (!(changed && pressed)) {
    return nav;
  }

  if (ks.enter) nav.confirm = true;
  if (ks.del) {
    nav.back = true;
    nav.cancel = true;
  }

  for (const auto c : ks.word) {
    switch (c) {
      case 'w':
      case 'W':
      case 'a':
      case 'A':
      case 'h':
      case 'H':
      case 'i':
      case 'I':
      case 'k':
      case 'K':
      case ';':
      case ',':
        nav.prev = true;
        break;
      case 's':
      case 'S':
      case 'd':
      case 'D':
      case 'j':
      case 'J':
      case 'l':
      case 'L':
      case '.':
      case '/':
        nav.next = true;
        break;
      case 0x1B:
      case '`':
      case '~':
        nav.back = true;
        nav.cancel = true;
        break;
      case 'b':
      case 'B':
        nav.key_b = true;
        break;
      default:
        break;
    }
  }

  return nav;
}
#elif defined(APP_TARGET_M5PAPER)
KeyNavState readKeyNavState() {
  KeyNavState nav;
  if (M5.BtnC.wasClicked())    nav.prev = true;
  if (M5.BtnA.wasClicked())    nav.next = true;
  if (M5.BtnC.pressedFor(kHoldPressMs)) { nav.back = true; nav.cancel = true; }
  return nav;
}
#else
KeyNavState readKeyNavState() {
  static bool b_down = false;
  static bool b_hold_latched = false;
  static uint32_t b_down_ms = 0;
  static uint32_t b_last_tap_ms = 0;
  constexpr uint32_t kBTapDebounceMs = 90;

  KeyNavState nav;
  bool b_pressed_now = false;
#if defined(APP_TARGET_STICKS3)
  b_pressed_now = M5.BtnPWR.isPressed() || M5.BtnB.isPressed();
#endif
#if defined(APP_TARGET_STICKCPLUS)
  b_pressed_now = M5.BtnB.isPressed();
#endif

  const uint32_t now = millis();

  if (b_pressed_now) {
    if (!b_down) {
      b_down = true;
      b_down_ms = now;
      b_hold_latched = false;
    } else if (!b_hold_latched && (now - b_down_ms >= kHoldPressMs)) {
      nav.back = true;
      nav.cancel = true;
      b_hold_latched = true;
    }
    return nav;
  }

  if (!b_down) return nav;

  const uint32_t held_ms = now - b_down_ms;
  if (!b_hold_latched && held_ms >= kBTapDebounceMs && (now - b_last_tap_ms >= kBTapDebounceMs)) {
    nav.key_b = true;
    b_last_tap_ms = now;
  }

  b_down = false;
  b_hold_latched = false;
  return nav;
}
#endif

void playTone(uint16_t freq, uint16_t ms) {
  M5.Speaker.tone(freq, ms);
}

void playSuccessTone() {
  playTone(1760, 60);
  delay(10);
  playTone(2093, 80);
}

void playCardTone(const String& protocol) {
  if (protocol == "ISO14443A" || protocol.startsWith("MFC") || protocol.startsWith("MFP")
      || protocol.startsWith("NTAG") || protocol.startsWith("MFUL") || protocol == "DESFire") {
    playTone(1480, 70);
  } else if (protocol == "ISO14443B") {
    playTone(1318, 70);
  } else if (protocol == "ISO15693") {
    playTone(1174, 90);
  } else if (protocol == "FeliCa") {
    playTone(1568, 90);
  } else {
    playTone(1047, 80);
  }
}

void playNdefTone(bool is_wifi) {
  if (is_wifi) {
    playTone(1760, 60);
    delay(8);
    playTone(1976, 60);
    delay(8);
    playTone(2349, 90);
    return;
  }
  playSuccessTone();
}

// ---------- Speaker GPIO 5V control (M5StickS3 PY32PMIC) ----------
static void setSpeaker5V(bool enable) {
#if defined(APP_TARGET_STICKS3)
  if (enable) {
    M5.In_I2C.bitOn(0x6E, 0x11, 0b00001000, 100000);
  } else {
    M5.In_I2C.bitOff(0x6E, 0x11, 0b00001000, 100000);
  }
#else
  (void)enable;
#endif
}

const MenuPage kHomeOrder[] = {MenuPage::Reader, MenuPage::Emulator, MenuPage::WebFiles, MenuPage::Piano, MenuPage::About};
const MenuPage kHomeOrderNfcUnit[] = {MenuPage::Reader, MenuPage::Emulator, MenuPage::WebFiles, MenuPage::Piano, MenuPage::About};
// Keep the same accent palette for both GroveNFC and Unit NFC to avoid UI mismatch.

inline bool isNfcUnitMode() { return nfc_module_name == "M5 Unit NFC"; }
inline bool isEmuTypeSupportedCurrentMode(EmuType type) {
  if (isNfcUnitMode()) {
    return type == EmuType::N213 || type == EmuType::N215 || type == EmuType::N216 || type == EmuType::Felica;
  }
  // Grove module does not support Felica card emulation.
  return type != EmuType::Felica;
}
inline int  homePageCount() { return isNfcUnitMode() ? static_cast<int>(sizeof(kHomeOrderNfcUnit) / sizeof(kHomeOrderNfcUnit[0])) : static_cast<int>(sizeof(kHomeOrder) / sizeof(kHomeOrder[0])); }
inline MenuPage homePageAt(int i) { return isNfcUnitMode() ? kHomeOrderNfcUnit[i] : kHomeOrder[i]; }

MenuPage homePageWithOffset(int offset) {
  const int count = homePageCount();
  if (count == 0) return MenuPage::Reader;
  const int raw = (home_index + offset) % count;
  const int idx = raw < 0 ? raw + count : raw;
  return homePageAt(idx);
}

const char* homePageName(MenuPage page) {
  switch (page) {
    case MenuPage::Reader:   return "Reader";
    case MenuPage::ReadNDEF: return "Read NDEF";
    case MenuPage::Emulator: return "Emulator";
    case MenuPage::WebFiles: return "Dumps";
    case MenuPage::Piano:    return "Piano";
    case MenuPage::About:    return "About";
    case MenuPage::Diagnose: return "Diagnose";
    default:                 return "";
  }
}

// Forward declarations for home icon drawers (defined later in drawScreen section)
void drawHomeIconReader(lgfx::v1::LGFXBase& d, int cx, int cy, uint16_t accent);
void drawHomeIconEmulator(lgfx::v1::LGFXBase& d, int cx, int cy, uint16_t accent);
void drawHomeIconDumps(lgfx::v1::LGFXBase& d, int cx, int cy, uint16_t accent);

void drawHomeIconForPage(lgfx::v1::LGFXBase& d, int cx, int cy, MenuPage page, uint16_t accent) {
  if (page == MenuPage::About) {
    d.drawCircle(cx, cy, 16, accent);
    d.drawCircle(cx, cy, 15, accent);
    d.fillRect(cx - 2, cy - 8, 4, 4, accent);
    d.fillRect(cx - 2, cy - 1, 4, 11, accent);
  } else if (page == MenuPage::Reader) {
    drawHomeIconReader(d, cx, cy, accent);
  } else if (page == MenuPage::ReadNDEF) {
    d.fillRect(cx - 16, cy - 18, 30, 34, accent);
    d.fillRect(cx - 12, cy - 14, 22, 26, TFT_BLACK);
    d.fillRect(cx + 8, cy - 18, 6, 6, TFT_BLACK);
    d.fillRect(cx + 4, cy - 14, 4, 2, accent);
    d.fillRect(cx - 8, cy - 6, 14, 3, accent);
    d.fillRect(cx - 8, cy, 14, 3, accent);
    d.fillRect(cx - 8, cy + 6, 10, 3, accent);
  } else if (page == MenuPage::WebFiles) {
    drawHomeIconDumps(d, cx, cy, accent);
  } else if (page == MenuPage::Emulator) {
    drawHomeIconEmulator(d, cx, cy, accent);
  } else if (page == MenuPage::Piano) {
    d.fillRect(cx - 18, cy - 18, 36, 30, accent);
    d.fillRect(cx - 14, cy - 14, 28, 22, TFT_BLACK);
    d.fillRect(cx - 10, cy - 10, 3, 18, accent);
    d.fillRect(cx - 4, cy - 10, 3, 18, accent);
    d.fillRect(cx + 2, cy - 10, 3, 18, accent);
    d.fillRect(cx + 8, cy - 10, 3, 18, accent);
  }
}

const char* pageName(MenuPage page) {
  switch (page) {
    case MenuPage::Reader:
      return "Reader";
    case MenuPage::Emulator:
      return "Emulator";
    case MenuPage::WebFiles:
      return "Dumps";
    case MenuPage::Diagnose:
      return "Diagnose";
    case MenuPage::ReadNDEF:
      return "Read NDEF";
    case MenuPage::Piano:
      return "Piano";
    case MenuPage::About:
      return "About";
    default:
      return "Unknown";
  }
}

#ifdef APP_TARGET_M5PAPER
void enterCurrentFeature();
void drawScreen(bool popup_only = false);
void goHome();
void startCurrentEmulation();
void switchEmuType(int8_t dir);
void selectEmuType(EmuType type);
void stopAllModes();
void refreshDumpFiles(bool filter_by_current_type);
void drawM5PaperEmulatorHexTable(lgfx::v1::LGFXBase& d);
void refreshM5PaperEmulatorHexTable();
bool loadDumpIntoEmulator(const String& path);
String buildDumpPreview(const String& path);
String dumpTypeLabel(const String& path);
bool mapTypeHintToEmuType(String hint, EmuType& out);
String loadLastDumpForType(EmuType type);
void saveLastDumpForType(EmuType type, const String& path);

struct M5PaperHomeTile {
  int x;
  int y;
  int w;
  int h;
  int icon_cx;
  int icon_cy;
};

static M5PaperHomeTile getM5PaperHomeTile(int index, int screen_w, int screen_h) {
  const int status_h = kStatusBarHeight;
  const int margin_x = 14;
  const int margin_top = 12;
  const int margin_bottom = 18;
  const int gap_x = 12;
  const int gap_y = 14;
  const int cols = 2;
  const int count = homePageCount();
  const int rows = max(1, (count + cols - 1) / cols);
  const int grid_x = margin_x;
  const int grid_y = status_h + margin_top;
  const int grid_w = screen_w - margin_x * 2;
  const int grid_h = screen_h - grid_y - margin_bottom;
  const int cell_w = (grid_w - gap_x) / cols;
  const int cell_h = (grid_h - gap_y * (rows - 1)) / rows;
  const int row = index / cols;
  const int col = index % cols;

  M5PaperHomeTile tile{};
  tile.x = grid_x + col * (cell_w + gap_x);
  tile.y = grid_y + row * (cell_h + gap_y);
  tile.w = cell_w;
  tile.h = cell_h;
  tile.icon_cx = tile.x + tile.w / 2;
  tile.icon_cy = tile.y + tile.h / 2 - 18;
  return tile;
}

static int findM5PaperHomeTileIndex(int x, int y) {
  const int count = homePageCount();
  const int screen_w = M5.Display.width();
  const int screen_h = M5.Display.height();
  for (int i = 0; i < count; ++i) {
    const auto tile = getM5PaperHomeTile(i, screen_w, screen_h);
    if (x >= tile.x && x < tile.x + tile.w && y >= tile.y && y < tile.y + tile.h) {
      return i;
    }
  }
  return -1;
}

static bool handleM5PaperHomeTouch() {
  if (!in_home || M5.Touch.getCount() == 0) return false;

  const auto touch = M5.Touch.getDetail();
  const int tile_index = findM5PaperHomeTileIndex(touch.x, touch.y);
  if (tile_index < 0) return false;

  if (touch.wasClicked()) {
    home_index = tile_index;
    menu_page = homePageAt(home_index);
    enterCurrentFeature();
    return true;
  }

  return false;
}

static bool handleM5PaperStatusTouch() {
  if (in_home || M5.Touch.getCount() == 0) return false;
  const auto touch = M5.Touch.getDetail();
  if (touch.wasClicked() && touch.y < kStatusBarHeight && touch.x < 150) {
    if (menu_page == MenuPage::WebFiles && dumps_stage == DumpsStage::Preview) {
      dumps_stage = DumpsStage::Browse;
      m5paper_dump_actions = false;
      drawScreen();
    } else if (menu_page == MenuPage::WebFiles && dumps_pick_for_emu) {
      dumps_pick_for_emu = false;
      menu_page = MenuPage::Emulator;
      drawScreen();
    } else {
      goHome();
    }
    return true;
  }
  return false;
}

static bool m5PaperHit(const m5::touch_detail_t& t, int x, int y, int w, int h) {
  return t.x >= x && t.x < x + w && t.y >= y && t.y < y + h;
}

static bool handleM5PaperFeatureTouch() {
  if (in_home || M5.Touch.getCount() == 0) return false;
  const auto touch = M5.Touch.getDetail();
  if (!touch.wasClicked()) return false;
  const int w = M5.Display.width();

  if (menu_page == MenuPage::Reader) {
    if (m5PaperHit(touch, 24, 820, w - 48, 88)) {
      last_card.valid = false;
      last_card.protocol = "None";
      last_card.uid = "";
      last_card.detail = "Scanning now...";
      last_poll_ms = 0;
      drawScreen();
      return true;
    }
  } else if (menu_page == MenuPage::Emulator) {
    constexpr int grid_x = 24, grid_y = 174, cell_w = 117, cell_h = 76, gap = 8;
    for (uint8_t i = 0; i < static_cast<uint8_t>(EmuType::Count); ++i) {
      const int col = i % 4, row = i / 4;
      if (m5PaperHit(touch, grid_x + col * (cell_w + gap),
                     grid_y + row * (cell_h + gap), cell_w, cell_h)) {
        const EmuType selected = static_cast<EmuType>(i);
        if (isEmuTypeSupportedCurrentMode(selected)) {
          m5paper_emu_hex_offset = 0;
          selectEmuType(selected);
        }
        return true;
      }
    }
    if (m5PaperHit(touch, 354, 384, 144, 64)) {
      dumps_pick_for_emu = true;
      dumps_stage = DumpsStage::Browse;
      dump_file_index = 0;
      menu_page = MenuPage::WebFiles;
      refreshDumpFiles(true);
      drawScreen();
      return true;
    }
    constexpr int hex_x = 24, hex_y = 558, hex_w = 492, hex_h = 382;
    const uint8_t type_idx = static_cast<uint8_t>(emu_type);
    const String dump_path = type_idx < static_cast<uint8_t>(EmuType::Count)
                                 ? emu_dump_loaded_path[type_idx]
                                 : String();
    if (!dump_path.isEmpty() && littlefs_ready && LittleFS.exists(dump_path)) {
      const bool two_columns = emu_type == EmuType::N213 || emu_type == EmuType::N215 ||
                               emu_type == EmuType::N216 || emu_type == EmuType::ISO15;
      const size_t page_items = two_columns ? 20 : 10;
      const String preview = buildDumpPreview(dump_path);
      size_t start = 0;
      for (int header = 0; header < 3 && start < preview.length(); ++header) {
        const int nl = preview.indexOf('\n', start);
        start = nl < 0 ? preview.length() : static_cast<size_t>(nl + 1);
      }
      size_t total = 0;
      while (start < preview.length()) {
        ++total;
        const int nl = preview.indexOf('\n', start);
        start = nl < 0 ? preview.length() : static_cast<size_t>(nl + 1);
      }
      if (m5PaperHit(touch, hex_x, hex_y, hex_w, hex_h / 2)) {
        if (total > page_items) {
          if (m5paper_emu_hex_offset >= page_items) m5paper_emu_hex_offset -= page_items;
          else m5paper_emu_hex_offset = ((total - 1) / page_items) * page_items;
        }
        refreshM5PaperEmulatorHexTable();
        return true;
      }
      if (m5PaperHit(touch, hex_x, hex_y + hex_h / 2, hex_w, hex_h - hex_h / 2)) {
        if (total > page_items) {
          m5paper_emu_hex_offset += page_items;
          if (m5paper_emu_hex_offset >= total) m5paper_emu_hex_offset = 0;
        }
        refreshM5PaperEmulatorHexTable();
        return true;
      }
    }
  } else if (menu_page == MenuPage::WebFiles && dumps_stage == DumpsStage::Browse) {
    if (m5paper_dump_delete_confirm) {
      if (m5PaperHit(touch, 70, 620, 180, 86)) {
        if (dump_file_index < emu_dump_count) {
          const String deleted_path = emu_dump_files[dump_file_index];
          LittleFS.remove(deleted_path);
          for (uint8_t i = 0; i < static_cast<uint8_t>(EmuType::Count); ++i) {
            if (emu_dump_loaded_path[i] == deleted_path || loadLastDumpForType(static_cast<EmuType>(i)) == deleted_path) {
              emu_dump_loaded_path[i] = "";
              emu_dump_loaded_uid[i] = "";
              saveLastDumpForType(static_cast<EmuType>(i), "");
            }
          }
        }
        m5paper_dump_delete_confirm = false;
        m5paper_dump_actions = false;
        refreshDumpFiles(false);
        drawScreen();
        return true;
      }
      if (m5PaperHit(touch, 290, 620, 180, 86)) {
        m5paper_dump_delete_confirm = false;
        drawScreen();
        return true;
      }
      return true;
    }
    if (m5paper_dump_actions) {
      if (m5PaperHit(touch, 60, 560, 420, 76)) {
        dumps_preview_text = buildDumpPreview(emu_dump_files[dump_file_index]);
        dumps_preview_offset = 0;
        dumps_stage = DumpsStage::Preview;
        m5paper_dump_actions = false;
        drawScreen();
        return true;
      }
      if (m5PaperHit(touch, 60, 650, 420, 76)) {
        EmuType inferred = emu_type;
        if (mapTypeHintToEmuType(dumpTypeLabel(emu_dump_files[dump_file_index]), inferred)) emu_type = inferred;
        emu_user_stopped = false;
        if (loadDumpIntoEmulator(emu_dump_files[dump_file_index])) {
          menu_page = MenuPage::Emulator;
          dumps_pick_for_emu = false;
        }
        m5paper_dump_actions = false;
        drawScreen();
        return true;
      }
      if (m5PaperHit(touch, 60, 740, 420, 76)) {
        m5paper_dump_delete_confirm = true;
        drawScreen();
        return true;
      }
      if (m5PaperHit(touch, 60, 830, 420, 76)) {
        m5paper_dump_actions = false;
        drawScreen();
        return true;
      }
      return true;
    }
    const int list_y = 170;
    const int row_h = 76;
    const int visible = 7;
    int first = 0;
    if (dump_file_index >= visible) first = dump_file_index - visible + 1;
    for (int row = 0; row < visible; ++row) {
      const int idx = first + row;
      if (idx >= emu_dump_count) break;
      if (m5PaperHit(touch, 20, list_y + row * row_h, w - 40, row_h - 8)) {
        dump_file_index = static_cast<uint8_t>(idx);
        m5paper_dump_actions = true;
        drawScreen();
        return true;
      }
    }
  } else if (menu_page == MenuPage::WebFiles && dumps_stage == DumpsStage::Preview) {
    constexpr size_t rows_per_page = 17;
    size_t data_lines = 0;
    int newline_count = 0;
    for (size_t i = 0; i < dumps_preview_text.length(); ++i) {
      if (dumps_preview_text[i] == '\n') {
        ++newline_count;
        if (newline_count > 3) ++data_lines;
      }
    }
    if (data_lines > rows_per_page) {
      dumps_preview_offset += rows_per_page;
      if (dumps_preview_offset >= data_lines) dumps_preview_offset = 0;
      drawScreen();
    }
    return true;
  } else if (menu_page == MenuPage::Piano) {
    if (piano_stage == PianoStage::Menu) {
      if (m5PaperHit(touch, 24, 260, w - 48, 110)) {
        setSpeaker5V(true);
        piano_stage = PianoStage::Play;
        piano_last_note = "-";
        piano_status = "Place a mapped card to play";
        drawScreen();
        return true;
      }
      if (m5PaperHit(touch, 24, 400, w - 48, 110)) {
        piano_stage = PianoStage::Config;
        piano_config_step = 0;
        for (uint8_t i = 0; i < kPianoNoteCount; ++i) piano_card_map[i] = "";
        piano_status = String("Scan ") + kPianoNoteName[0];
        drawScreen();
        return true;
      }
    }
  }
  return false;
}
#else
void drawScreen(bool popup_only = false);
#endif

const char* emuName(EmuType type) {
  switch (type) {
    case EmuType::MF1K:
      return "MFC1K";
    case EmuType::MF4K:
      return "MFC4K";
    case EmuType::N213:
      return "NTAG213";
    case EmuType::N215:
      return "NTAG215";
    case EmuType::N216:
      return "NTAG216";
    case EmuType::ISO14B:
      return "ISO14B";
    case EmuType::Felica:
      return "Felica";
    case EmuType::ISO15:
      return "ISO15";
    default:
      return "Unknown";
  }
}

const char* emuActionName(uint8_t idx) {
  switch (idx) {
    case 0:
      return "Load Dump";
    case 1:
      return "Exit";
    default:
      return "";
  }
}

String shortDumpName(const String& path, size_t limit = 18) {
  String name = path;
  const int slash = name.lastIndexOf('/');
  if (slash >= 0 && slash + 1 < static_cast<int>(name.length())) {
    name = name.substring(static_cast<size_t>(slash + 1));
  }
  if (name.length() <= limit) return name;
  if (limit <= 2) return name.substring(0, limit);
  return name.substring(0, limit - 2) + "..";
}

String dumpFileName(const String& path) {
  const int slash = path.lastIndexOf('/');
  return (slash >= 0 && slash + 1 < static_cast<int>(path.length()))
             ? path.substring(static_cast<size_t>(slash + 1))
             : path;
}

uint8_t dumpMenuCount() {
  return kEmuDumpMenuBaseItems;
}

uint8_t dumpsMenuCount() {
  return dumps_pick_for_emu ? 1 : 4;
}

bool dumpsMenuItemDisabled(uint8_t idx) {
  return false;
}

String dumpDisplayName(const String& path, size_t limit = 18);

String typeMenuLabel(uint8_t idx) {
  if (idx == 0) return "Load Dump";
  if (idx == 1) return "Exit";
  const uint8_t file_idx = static_cast<uint8_t>(idx - kEmuDumpMenuBaseItems);
  if (file_idx < emu_dump_count) return dumpDisplayName(emu_dump_files[file_idx]);
  return "";
}

void startCurrentEmulation();
void refreshDumpFiles(bool filter_by_current_type = true);
bool loadDumpIntoEmulator(const String& path);
bool applyDumpToCurrentType(const String& path, bool remember_selection, bool auto_restore, bool silent_fail = false);
void removeLegacyExampleDumps();
uint8_t dumpsBrowseCount();
bool dumpsSelectionIsBack();
bool dumpsHasSelectedFile();
uint8_t dumpsSelectedFileIndex();
void handleEmuApPortal();
bool startEmuApPortal();
void stopEmuApPortal();
bool parseWifiNdef(const String& input, String& ssid, String& pass, String& auth);
bool mapTypeHintToEmuType(String hint, EmuType& out);
bool mapPm3FileTypeToEmuType(String file_type, EmuType& out);
bool inferEmuTypeByLength(size_t len, EmuType& out);
bool extractJsonValue(const String& src, const String& key, String& out);
bool parseHexBytes(const String& raw, uint8_t* out, size_t& out_len);
bool parseByteArray(const String& raw, uint8_t* out, size_t& out_len);
bool injectPm3Iso15UidHeader(const String& json, uint8_t* dump, size_t dump_len);
bool parseJsonBlocksObject(const String& blocks_obj,
                           uint8_t* out,
                           size_t max_bytes,
                           uint8_t block_size_hint,
                           size_t& out_len,
                           size_t& out_blocks);
String normalizeHexText(String raw);
String deriveEmuUidFromDump(EmuType type, const uint8_t* dump, size_t dump_len);
void readDumpMetaForWeb(const String& path,
                        String& type_label,
                        String& uid,
                        String& sak,
                        String& atqa,
                        String& source_label);
String dumpUidForList(const String& path);
void rebuildDumpUidViewLabels();
void showDumpsEmuPopup(const String& uid, const String& type_name);
bool normalizeDumpFileInPlace(const String& path, String& status_text, const String& bin_type_hint = "auto");
void stopAllModes();
bool recoverNfc(const char* reason, bool rebegin);
bool initNfcAtBoot();
bool sendNfcCmdAndWait(NfcCmd cmd, uint32_t timeout_ms = 2000);
void goHome();
void enterCurrentFeature();
void runDiagnose();
void handlePiano();
void loadPianoConfig();
void savePianoConfig();
uint8_t pianoMappedCount();
int8_t findPianoNoteByCard(const String& card_key);
String buildPianoCardKey(const grove_nfc::CardInfo& card);
void drawPianoPlayPartial();

const char* protocolFull(const String& protocol) {
  if (protocol == "ISO14443A") return "ISO14443A";
  if (protocol == "ISO14443B") return "ISO14443B";
  if (protocol == "ISO15693") return "ISO15693";
  if (protocol == "FeliCa") return "Felica";
  if (protocol == "MFC1K") return "MFC 1K";
  if (protocol == "MFC4K") return "MFC 4K";
  if (protocol == "MFCMini") return "MFC Mini";
  if (protocol == "MFPlus2K") return "MF+ 2K";
  if (protocol == "MFPlus4K") return "MF+ 4K";
  if (protocol == "NTAG213") return "NTAG213";
  if (protocol == "NTAG215") return "NTAG215";
  if (protocol == "NTAG216") return "NTAG216";
  if (protocol == "NTAG203") return "NTAG203";
  if (protocol == "NTAG") return "NTAG";
  if (protocol == "MFUL11") return "MFUL11";
  if (protocol == "MFUL21") return "MFUL21";
  if (protocol == "MFUL") return "MFUL";
  if (protocol == "MFUL-C") return "MFUL-C";
  if (protocol == "DESFire") return "DESFire";
  return "Unknown";
}

const char* emuTypeShort(EmuType type) {
  switch (type) {
    case EmuType::MF1K:
      return "MF1K";
    case EmuType::MF4K:
      return "MF4K";
    case EmuType::N213:
      return "N213";
    case EmuType::N215:
      return "N215";
    case EmuType::N216:
      return "N216";
    case EmuType::ISO14B:
      return "14B";
    case EmuType::Felica:
      return "FLC";
    case EmuType::ISO15:
      return "15";
    default:
      return "---";
  }
}

String emulatorDisplayId(EmuType type, uint8_t slot) {
  String base;
  if (type == EmuType::MF1K) {
    base = "6117C420";
  } else if (type == EmuType::MF4K) {
    base = "041219C3CC9802";
  } else if (type == EmuType::N213 || type == EmuType::N215 || type == EmuType::N216) {
    base = "04311D01174503";
  } else if (type == EmuType::ISO14B) {
    base = "11223344";
  } else if (type == EmuType::Felica) {
    base = "02FE123456789ABC";
  } else {
    base = "E0070050B902C6C1";
  }

  const uint8_t tweak = slot & 0x0F;
  const char lut[] = "0123456789ABCDEF";
  const char c = base[base.length() - 1];
  uint8_t v = 0;
  if (c >= '0' && c <= '9') v = static_cast<uint8_t>(c - '0');
  else if (c >= 'A' && c <= 'F') v = static_cast<uint8_t>(10 + c - 'A');
  v = (v + tweak) & 0x0F;
  base.setCharAt(base.length() - 1, lut[v]);
  return base;
}

String activeEmulatorDisplayId(EmuType type) {
  const uint8_t idx = static_cast<uint8_t>(type);
  if (idx < static_cast<uint8_t>(EmuType::Count) && !emu_dump_loaded_uid[idx].isEmpty()) {
    return emu_dump_loaded_uid[idx];
  }
  return emulatorDisplayId(type, 0);
}

void updateEmulatorSourceStatus() {
  const uint8_t idx = static_cast<uint8_t>(emu_type);
  if (idx < static_cast<uint8_t>(EmuType::Count) && !emu_dump_loaded_path[idx].isEmpty()) {
    String name = emu_dump_loaded_path[idx];
    const int slash = name.lastIndexOf('/');
    if (slash >= 0) name = name.substring(static_cast<size_t>(slash + 1));
    emu_dump_status = "Loaded " + name;
  } else {
    emu_dump_status = String("Built-in ") + emuName(emu_type) + " demo data";
  }
}

String marqueeText(const String& text, size_t visible_chars, uint32_t speed_ms = 180) {
  if (text.length() <= visible_chars) return text;
  const size_t n = text.length();
  const size_t padding = 4;
  const size_t cycle = n + padding + visible_chars;
  const size_t phase = (millis() / speed_ms) % cycle;

  size_t start = 0;
  if (phase > visible_chars) {
    start = phase - visible_chars;
    if (start > n) start = n;
  }

  if (start + visible_chars > n) {
    String tail = text.substring(start);
    while (tail.length() < visible_chars) tail += " ";
    return tail;
  }
  return text.substring(start, start + visible_chars);
}

static String ellipsizeCanvasText(const String& text, int max_width) {
  if (max_width <= 0 || g_canvas.textWidth(text) <= max_width) return text;
  const String suffix = "...";
  String fitted = text;
  while (!fitted.isEmpty() && g_canvas.textWidth(fitted + suffix) > max_width) {
    fitted.remove(fitted.length() - 1);
  }
  return fitted + suffix;
}

String upperText(String s) {
  s.toUpperCase();
  return s;
}

String formatUidDisplay(const String& input) {
  String hex;
  hex.reserve(input.length());
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
      hex += static_cast<char>((c >= 'a' && c <= 'f') ? c - ('a' - 'A') : c);
    }
  }
  String out;
  out.reserve(hex.length() + hex.length() / 2);
  for (size_t i = 0; i < hex.length(); ++i) {
    if (i > 0 && (i & 1u) == 0) out += ':';
    out += hex[i];
  }
  return out;
}

String formatIdText(String s) {
  s.toUpperCase();
  s.replace(":", "");
  return s;
}

String readerBodyText(const grove_nfc::CardInfo& card) {
  if (!card.valid) return "";
  if (card.protocol == "FeliCa" && !card.detail.isEmpty()) {
    return card.detail;
  }
  return formatUidDisplay(card.uid);
}

String protocolTag(const String& protocol) {
  if (protocol == "ISO14443A") return "14A";
  if (protocol == "ISO14443B") return "14B";
  if (protocol == "ISO15693") return "15";
  if (protocol == "FeliCa") return "FEL";
  if (protocol.startsWith("MFC") || protocol.startsWith("MFP")) return "14A";
  if (protocol.startsWith("NTAG") || protocol.startsWith("MFUL")) return "14A";
  if (protocol == "DESFire") return "14A";
  return protocol;
}

String normalizeDetailForLog(String detail) {
  detail.replace("\r", " ");
  detail.replace("\n", " | ");
  while (detail.indexOf("  ") >= 0) detail.replace("  ", " ");
  detail.trim();
  return detail;
}

String formatCardLogLine(const grove_nfc::CardInfo& card) {
  String line = "[CARD][" + protocolTag(card.protocol) + "] UID=" + card.uid;
  const String detail = normalizeDetailForLog(card.detail);
  if (!detail.isEmpty()) {
    line += " | " + detail;
  }
  return line;
}

uint8_t pianoMappedCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < kPianoNoteCount; ++i) {
    if (!piano_card_map[i].isEmpty()) ++count;
  }
  return count;
}

int8_t findPianoNoteByCard(const String& card_key) {
  if (card_key.isEmpty()) return -1;
  for (uint8_t i = 0; i < kPianoNoteCount; ++i) {
    if (piano_card_map[i] == card_key) return static_cast<int8_t>(i);
  }
  return -1;
}

String buildPianoCardKey(const grove_nfc::CardInfo& card) {
  if (!card.valid) return "";
  String uid = card.uid;
  uid.toUpperCase();
  uid.replace(":", "");
  return card.protocol + "|" + uid;
}

void loadPianoConfig() {
  if (!prefs.begin(kPianoPrefNs, true)) {
    piano_status = "Config load fail";
    return;
  }
  for (uint8_t i = 0; i < kPianoNoteCount; ++i) {
    String key = "n" + String(i);
    piano_card_map[i] = prefs.getString(key.c_str(), "");
  }
  prefs.end();

  const uint8_t mapped = pianoMappedCount();
  if (mapped == kPianoNoteCount) piano_status = "Configured 8/8";
  else piano_status = "Configured " + String(mapped) + "/8";
}

void savePianoConfig() {
  if (!prefs.begin(kPianoPrefNs, false)) {
    piano_status = "Config save fail";
    return;
  }
  for (uint8_t i = 0; i < kPianoNoteCount; ++i) {
    String key = "n" + String(i);
    prefs.putString(key.c_str(), piano_card_map[i]);
  }
  prefs.end();
}

bool isDumpFilePath(const String& path) {
  String p = path;
  p.toLowerCase();
  return p.endsWith(".json") || p.endsWith(".bin");
}

bool dumpLengthMatchesType(EmuType type, size_t len) {
  switch (type) {
    case EmuType::MF1K:
      return len == 1024;
    case EmuType::MF4K:
      return len == 4096;
    case EmuType::N213:
      return len == 180;
    case EmuType::N215:
      return len == 540;
    case EmuType::N216:
      return len == 924;
    case EmuType::ISO14B:
      return len > 0 && len <= kEmuDumpMaxBytes;
    case EmuType::Felica:
      return len == 448;
    case EmuType::ISO15:
      return len > 0 && len <= kIso15DumpMaxBytes && (len % 4u) == 0;
    default:
      return false;
  }
}

bool inferTypeFromPathHint(const String& path, EmuType& out) {
  String hint = path;
  hint.toLowerCase();
  if (hint.indexOf("ntag216") >= 0 || hint.indexOf("n216") >= 0) {
    out = EmuType::N216;
    return true;
  }
  if (hint.indexOf("ntag215") >= 0 || hint.indexOf("n215") >= 0) {
    out = EmuType::N215;
    return true;
  }
  if (hint.indexOf("ntag213") >= 0 || hint.indexOf("n213") >= 0) {
    out = EmuType::N213;
    return true;
  }
  if (hint.indexOf("iso15693") >= 0 || hint.indexOf("iso15") >= 0) {
    out = EmuType::ISO15;
    return true;
  }
  if (hint.indexOf("iso14443b") >= 0 || hint.indexOf("14b") >= 0) {
    out = EmuType::ISO14B;
    return true;
  }
  if (hint.indexOf("felica") >= 0 || hint.indexOf("feli") >= 0) {
    out = EmuType::Felica;
    return true;
  }
  if (hint.indexOf("mf4k") >= 0 || hint.indexOf("mfc4k") >= 0 || hint.indexOf("4k") >= 0) {
    out = EmuType::MF4K;
    return true;
  }
  if (hint.indexOf("mifare") >= 0 || hint.indexOf("mfc") >= 0 || hint.indexOf("mf1k") >= 0 ||
      hint.indexOf("1k") >= 0) {
    out = EmuType::MF1K;
    return true;
  }
  return false;
}

bool dumpFileLikelyMatchesType(const String& path, EmuType target_type, bool strict_match = false) {
  if (!isDumpFilePath(path)) return false;

  String lower = path;
  lower.toLowerCase();

  if (lower.endsWith(".bin")) {
    File f = LittleFS.open(path, "r");
    if (!f || f.isDirectory()) return false;
    const size_t len = static_cast<size_t>(f.size());
    return dumpLengthMatchesType(target_type, len);
  }

  File f = LittleFS.open(path, "r");
  if (!f || f.isDirectory()) return false;
  String json;
  json.reserve(static_cast<size_t>(f.size()) + 1);
  while (f.available()) json += static_cast<char>(f.read());

  String file_type;
  EmuType hinted_type;
  if (extractJsonValue(json, "FileType", file_type) &&
      mapPm3FileTypeToEmuType(file_type, hinted_type)) {
    if (hinted_type == target_type) return true;
  } else if ((extractJsonValue(json, "type", file_type) ||
              extractJsonValue(json, "tagType", file_type) ||
              extractJsonValue(json, "protocol", file_type) ||
              extractJsonValue(json, "emuType", file_type)) &&
             mapTypeHintToEmuType(file_type, hinted_type)) {
    if (hinted_type == target_type) return true;
  }

  String blocks_obj;
  if (extractJsonValue(json, "blocks", blocks_obj) && blocks_obj.startsWith("{")) {
    size_t parsed_len = 0;
    size_t parsed_blocks = 0;
    uint8_t block_hint = 0;
    switch (target_type) {
      case EmuType::MF1K:
      case EmuType::MF4K: block_hint = 16; break;
      case EmuType::N213:
      case EmuType::N215:
      case EmuType::N216:
      case EmuType::Felica:
      case EmuType::ISO15: block_hint = 4; break;
      default: block_hint = 0; break;
    }
    if (parseJsonBlocksObject(blocks_obj, nullptr, kEmuDumpMaxBytes, block_hint, parsed_len, parsed_blocks)) {
      return dumpLengthMatchesType(target_type, parsed_len);
    }
  }

  EmuType path_hint;
  if (inferTypeFromPathHint(path, path_hint)) {
    return path_hint == target_type;
  }
  if (strict_match) return false;
  return true;
}

void dumpsPreviewApplyFont(lgfx::v1::LGFXBase& d, uint8_t level, int& line_h_out) {
  switch (level & 0x03u) {
    case 0:
      d.setFont(&fonts::Font0);
      d.setTextSize(1);
      break;
    case 1:
      d.setFont(&fonts::Font2);
      d.setTextSize(1);
      break;
    case 2:
      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      break;
    default:
      d.setFont(&fonts::Font2);
      d.setTextSize(2);
      break;
  }
  line_h_out = d.fontHeight() + 1;
}

size_t dumpsPreviewPageLines(int height, int line_h) {
  return static_cast<size_t>(max(1, height / max(1, line_h)));
}

size_t dumpsPreviewCountLines(const String& text) {
  if (text.isEmpty()) return 0;
  size_t lines = 1;
  for (size_t i = 0; i < text.length(); ++i) {
    if (text[i] == '\n') ++lines;
  }
  return lines;
}

bool dumpsPreviewParseBlockLine(const String& line, size_t& block_index, size_t& hex_start, size_t& byte_count) {
  block_index = 0;
  hex_start = 0;
  byte_count = 0;

  const int colon = line.indexOf(':');
  if (colon <= 0) return false;

  unsigned long id = 0;
  for (int i = 0; i < colon; ++i) {
    const char c = line[static_cast<size_t>(i)];
    if (c < '0' || c > '9') return false;
    id = id * 10ul + static_cast<unsigned long>(c - '0');
    if (id > 65535ul) return false;
  }
  if (id == 0) return false;

  hex_start = static_cast<size_t>(colon + 1);
  while (hex_start < line.length() && line[hex_start] == ' ') ++hex_start;
  if (hex_start >= line.length()) return false;

  const size_t hex_len = line.length() - hex_start;
  if (hex_len == 0 || (hex_len % 2u) != 0u) return false;

  for (size_t i = hex_start; i < line.length(); ++i) {
    const char c = line[i];
    const bool is_hex = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
    if (!is_hex) return false;
  }

  block_index = static_cast<size_t>(id - 1ul);
  byte_count = hex_len / 2u;
  return true;
}

uint16_t dumpsPreviewByteColor(const String& type_label, size_t block_index, size_t byte_index, size_t bytes_per_block) {
  constexpr uint16_t kColUid = 0x07FF;     // cyan
  constexpr uint16_t kColBcc = 0xFD20;     // orange
  constexpr uint16_t kColKeyA = 0x07E0;    // green
  constexpr uint16_t kColAccess = 0xFFE0;  // yellow
  constexpr uint16_t kColKeyB = 0xF81F;    // magenta

  const bool is_mfc = type_label.startsWith("MFC");

  if (is_mfc && bytes_per_block >= 16u) {
    if (block_index == 0u) {
      if (byte_index <= 3u) return kColUid;
      if (byte_index == 4u) return kColBcc;
    }
    if (((block_index + 1u) % 4u) == 0u) {
      if (byte_index < 6u) return kColKeyA;
      if (byte_index < 10u) return kColAccess;
      return kColKeyB;
    }
    return TFT_WHITE;
  }

  if (bytes_per_block == 4u) {
    if (block_index == 0u) {
      if (byte_index <= 2u) return kColUid;
      if (byte_index == 3u) return kColBcc;
    } else if (block_index == 1u && byte_index <= 3u) {
      return kColUid;
    } else if (block_index == 2u && byte_index == 0u) {
      return kColBcc;
    }
    return TFT_WHITE;
  }

  if (block_index == 0u && byte_index < min(static_cast<size_t>(8), bytes_per_block)) {
    return kColUid;
  }
  return TFT_WHITE;
}

void drawDumpPreviewLine(lgfx::v1::LGFXBase& d,
                         int x,
                         int y,
                         const String& line,
                         const String& type_label,
                         uint16_t accent) {
  size_t block_index = 0;
  size_t hex_start = 0;
  size_t byte_count = 0;
  if (!dumpsPreviewParseBlockLine(line, block_index, hex_start, byte_count)) {
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setCursor(x, y);
    d.print(line);
    return;
  }

  const String prefix = line.substring(0, hex_start);
  d.setTextColor(accent, TFT_BLACK);
  d.setCursor(x, y);
  d.print(prefix);

  const int char_w = max(1, d.textWidth("0"));
  const int byte_w = char_w * 2;
  int cx = x + d.textWidth(prefix);

  for (size_t i = 0; i < byte_count; ++i) {
    const size_t begin = hex_start + i * 2u;
    const String hex = line.substring(begin, begin + 2u);
    d.setTextColor(dumpsPreviewByteColor(type_label, block_index, i, byte_count), TFT_BLACK);
    d.setCursor(cx, y);
    d.print(hex);
    cx += byte_w;
  }
}

bool mapPm3FileTypeToEmuType(String file_type, EmuType& out) {
  file_type.toLowerCase();
  if (file_type.indexOf("mfc") >= 0 || file_type.indexOf("mifare") >= 0) {
    out = EmuType::MF1K;
    return true;
  }
  if (file_type.indexOf("mfu") >= 0 || file_type.indexOf("ntag") >= 0 || file_type.indexOf("ul") >= 0) {
    out = EmuType::N213;
    return true;
  }
  if (file_type.indexOf("14b") >= 0 || file_type.indexOf("iso14443b") >= 0) {
    out = EmuType::ISO14B;
    return true;
  }
  if (file_type.indexOf("15") >= 0 || file_type.indexOf("iso15693") >= 0) {
    out = EmuType::ISO15;
    return true;
  }
  if (file_type.indexOf("felica") >= 0 || file_type.indexOf("feli") >= 0) {
    out = EmuType::Felica;
    return true;
  }
  return false;
}

bool isPm3MfuFamilyFileType(const String& file_type) {
  String lower = file_type;
  lower.toLowerCase();
  return lower.indexOf("mfu") >= 0 || lower.indexOf("ntag") >= 0 || lower.indexOf("ul") >= 0;
}

bool parseJsonBlocksObject(const String& blocks_obj,
                           uint8_t* out,
                           size_t max_bytes,
                           uint8_t block_size_hint,
                           size_t& out_len,
                           size_t& out_blocks) {
  out_len = 0;
  out_blocks = 0;
  const int n = static_cast<int>(blocks_obj.length());
  int pos = blocks_obj.indexOf('{');
  if (pos < 0) return false;
  ++pos;

  while (pos < n) {
    while (pos < n && (blocks_obj[pos] == ' ' || blocks_obj[pos] == '\r' || blocks_obj[pos] == '\n' || blocks_obj[pos] == '\t' || blocks_obj[pos] == ',')) ++pos;
    if (pos >= n || blocks_obj[pos] == '}') break;
    if (blocks_obj[pos] != '"') return false;

    ++pos;
    const int key_start = pos;
    while (pos < n && blocks_obj[pos] != '"') ++pos;
    if (pos >= n) return false;
    const String key = blocks_obj.substring(static_cast<size_t>(key_start), static_cast<size_t>(pos));
    ++pos;
    while (pos < n && (blocks_obj[pos] == ' ' || blocks_obj[pos] == '\r' || blocks_obj[pos] == '\n' || blocks_obj[pos] == '\t')) ++pos;
    if (pos >= n || blocks_obj[pos] != ':') return false;
    ++pos;
    while (pos < n && (blocks_obj[pos] == ' ' || blocks_obj[pos] == '\r' || blocks_obj[pos] == '\n' || blocks_obj[pos] == '\t')) ++pos;
    if (pos >= n) return false;

    String token;
    if (blocks_obj[pos] == '"') {
      ++pos;
      const int start = pos;
      while (pos < n) {
        if (blocks_obj[pos] == '"' && blocks_obj[pos - 1] != '\\') break;
        ++pos;
      }
      if (pos >= n) return false;
      token = blocks_obj.substring(static_cast<size_t>(start), static_cast<size_t>(pos));
      ++pos;
    } else if (blocks_obj[pos] == '[') {
      int depth = 1;
      const int start = pos;
      ++pos;
      while (pos < n && depth > 0) {
        if (blocks_obj[pos] == '[') ++depth;
        else if (blocks_obj[pos] == ']') --depth;
        ++pos;
      }
      if (depth != 0) return false;
      token = blocks_obj.substring(static_cast<size_t>(start), static_cast<size_t>(pos));
    } else {
      const int start = pos;
      while (pos < n && blocks_obj[pos] != ',' && blocks_obj[pos] != '}') ++pos;
      token = blocks_obj.substring(static_cast<size_t>(start), static_cast<size_t>(pos));
    }

    token.trim();
    if (token.isEmpty()) continue;

    char* end_ptr = nullptr;
    strtol(key.c_str(), &end_ptr, 10);
    if (end_ptr == key.c_str()) continue;

    uint8_t block[32] = {0};
    size_t block_len = 0;
    bool ok = token.startsWith("[") ? parseByteArray(token, block, block_len) : parseHexBytes(token, block, block_len);
    if (!ok || block_len == 0) continue;
    if (block_size_hint > 0) {
      if (block_len < block_size_hint) return false;
      block_len = block_size_hint;
    }
    if (out_len + block_len > max_bytes) return false;
    if (out != nullptr) {
      memcpy(out + out_len, block, block_len);
    }
    out_len += block_len;
    ++out_blocks;
  }

  return out_blocks > 0;
}

String dumpTypeLabel(const String& path) {
  String lower = path;
  lower.toLowerCase();

  if (lower.endsWith(".bin")) {
    File f = LittleFS.open(path, "r");
    if (!f || f.isDirectory()) return "Unknown";
    const size_t len = static_cast<size_t>(f.size());
    EmuType type;
    if (inferEmuTypeByLength(len, type)) {
      return emuName(type);
    }
    if (len > 0 && len <= kIso15DumpMaxBytes && (len % 4u) == 0) return "ISO15693";
    return "Unknown";
  }

  File f = LittleFS.open(path, "r");
  if (!f || f.isDirectory()) return "Unknown";
  String json;
  json.reserve(static_cast<size_t>(f.size()) + 1);
  while (f.available()) json += static_cast<char>(f.read());

  String file_type;
  EmuType type = EmuType::MF1K;
  bool has_file_type = extractJsonValue(json, "FileType", file_type) && mapPm3FileTypeToEmuType(file_type, type);

  String blocks_obj;
  size_t parsed_len = 0;
  size_t parsed_blocks = 0;
  if (extractJsonValue(json, "blocks", blocks_obj) && blocks_obj.startsWith("{")) {
    uint8_t block_hint = 0;
    if (has_file_type) {
      if (type == EmuType::MF1K || type == EmuType::MF4K) block_hint = 16;
      else if (type == EmuType::N213 || type == EmuType::N215 || type == EmuType::N216 || type == EmuType::ISO15 || type == EmuType::Felica) block_hint = 4;
    }
    parseJsonBlocksObject(blocks_obj, nullptr, kEmuDumpMaxBytes, block_hint, parsed_len, parsed_blocks);
  }

  if (has_file_type) {
    if (type == EmuType::MF1K || type == EmuType::MF4K) {
      if (parsed_len == 4096 || parsed_blocks >= 256) return "MFC4K";
      if (parsed_len == 1024 || parsed_blocks >= 64) return "MFC1K";
      return "MFC";
    }
    if (type == EmuType::N213) {
      if (parsed_len == 924 || parsed_blocks == 231) return "NTAG216";
      if (parsed_len == 540 || parsed_blocks == 135) return "NTAG215";
      if (parsed_len == 180 || parsed_blocks == 45) return "NTAG213";
      return "MFU";
    }
    return emuName(type);
  }

  String type_hint;
  if ((extractJsonValue(json, "type", type_hint) ||
       extractJsonValue(json, "tagType", type_hint) ||
       extractJsonValue(json, "protocol", type_hint) ||
       extractJsonValue(json, "emuType", type_hint)) &&
      mapTypeHintToEmuType(type_hint, type)) {
    return emuName(type);
  }

  if (parsed_len > 0) {
    EmuType inferred;
    if (inferEmuTypeByLength(parsed_len, inferred)) return emuName(inferred);
    if (parsed_len <= kIso15DumpMaxBytes && (parsed_len % 4u) == 0) return "ISO15693";
  }
  return "Unknown";
}

String dumpDisplayName(const String& path, size_t limit) {
  String name = path;
  const int slash = name.lastIndexOf('/');
  if (slash >= 0 && slash + 1 < static_cast<int>(name.length())) {
    name = name.substring(static_cast<size_t>(slash + 1));
  }

  String lower = path;
  lower.toLowerCase();
  if (lower.endsWith(".json")) {
    File f = LittleFS.open(path, "r");
    if (f && !f.isDirectory()) {
      String json;
      const size_t limit_bytes = min(static_cast<size_t>(f.size()), static_cast<size_t>(2048));
      json.reserve(limit_bytes + 1);
      for (size_t i = 0; i < limit_bytes && f.available(); ++i) {
        json += static_cast<char>(f.read());
      }
      String dump_name;
      if ((extractJsonValue(json, "name", dump_name) || extractJsonValue(json, "Name", dump_name)) && !dump_name.isEmpty()) {
        dump_name.trim();
        if (!dump_name.isEmpty()) {
          name = dump_name;
        }
      }
    }
  }

  if (name.length() <= limit) return name;
  if (limit <= 2) return name.substring(0, limit);
  return name.substring(0, limit - 2) + "..";
}

String buildDumpPreview(const String& path) {
  File f = LittleFS.open(path, "r");
  if (!f || f.isDirectory()) return "Open failed";

  String preview = dumpDisplayName(path, 24) + "\n";
  preview += dumpTypeLabel(path) + "  " + String(static_cast<size_t>(f.size())) + " bytes\n";

  auto appendBlockLines = [&](const uint8_t* data, size_t len, size_t block_size, bool include_ascii) {
    if (data == nullptr || len == 0) {
      preview += "No block data\n";
      return;
    }
    if (block_size == 0) block_size = 4;

    const size_t block_count = (len + block_size - 1) / block_size;
    preview += "Blocks: " + String(block_count) + "\n";

    const char* hex_chars = "0123456789ABCDEF";
    for (size_t b = 0; b < block_count; ++b) {
      preview += String(static_cast<unsigned>(b + 1));
      preview += ": ";

      const size_t begin = b * block_size;
      const size_t end = min(len, begin + block_size);
      for (size_t i = begin; i < end; ++i) {
        preview += hex_chars[(data[i] >> 4) & 0x0F];
        preview += hex_chars[data[i] & 0x0F];
      }
      if (include_ascii) {
        preview += " | ";
        for (size_t i = begin; i < end; ++i) {
          const uint8_t c = data[i];
          preview += (c >= 0x20 && c <= 0x7E) ? static_cast<char>(c) : '.';
        }
      }
      if (b + 1 < block_count) preview += '\n';
    }
  };

  String lower = path;
  lower.toLowerCase();
  if (lower.endsWith(".json")) {
    String json;
    json.reserve(static_cast<size_t>(f.size()) + 1);
    while (f.available()) json += static_cast<char>(f.read());

    EmuType type = EmuType::N213;
    String type_hint;
    bool has_type_hint = false;
    if (extractJsonValue(json, "FileType", type_hint) && mapPm3FileTypeToEmuType(type_hint, type)) {
      has_type_hint = true;
    } else if ((extractJsonValue(json, "type", type_hint) ||
                extractJsonValue(json, "tagType", type_hint) ||
                extractJsonValue(json, "protocol", type_hint) ||
                extractJsonValue(json, "emuType", type_hint)) &&
               mapTypeHintToEmuType(type_hint, type)) {
      has_type_hint = true;
    }

    size_t block_size_hint = 0;
    if (has_type_hint) {
      block_size_hint = (type == EmuType::MF1K || type == EmuType::MF4K) ? 16 : 4;
    }

    String blocks_obj;
    if (extractJsonValue(json, "blocks", blocks_obj) && blocks_obj.startsWith("{")) {
      uint8_t dump[kEmuDumpMaxBytes] = {0};
      size_t out_len = 0;
      size_t out_blocks = 0;
      if (parseJsonBlocksObject(blocks_obj,
                                dump,
                                sizeof(dump),
                                static_cast<uint8_t>(block_size_hint),
                                out_len,
                                out_blocks) && out_len > 0) {
        size_t block_size = block_size_hint;
        if (block_size == 0) {
          if ((out_len % 16u) == 0u && out_len >= 64u) block_size = 16;
          else block_size = 4;
        }
        const String preview_type = dumpTypeLabel(path);
        appendBlockLines(dump, out_len, block_size,
                         preview_type == "ISO15693" || preview_type.startsWith("NTAG"));
        return preview;
      }
    }

    String payload;
    if (extractJsonValue(json, "data", payload) || extractJsonValue(json, "dump", payload) ||
        extractJsonValue(json, "bytes", payload) || extractJsonValue(json, "hex", payload) ||
        extractJsonValue(json, "bin", payload)) {
      uint8_t dump[kEmuDumpMaxBytes] = {0};
      size_t out_len = 0;
      const bool ok = payload.startsWith("[") ? parseByteArray(payload, dump, out_len)
                                               : parseHexBytes(payload, dump, out_len);
      if (ok && out_len > 0) {
        size_t block_size = block_size_hint;
        if (block_size == 0) {
          if ((out_len % 16u) == 0u && out_len >= 64u) block_size = 16;
          else block_size = 4;
        }
        const String preview_type = dumpTypeLabel(path);
        appendBlockLines(dump, out_len, block_size,
                         preview_type == "ISO15693" || preview_type.startsWith("NTAG"));
        return preview;
      }
    }

    preview += "No parsable block data";
    return preview;
  }

  uint8_t dump[kEmuDumpMaxBytes] = {0};
  const size_t got = f.read(dump, sizeof(dump));
  if (got == 0) return preview + "No block data";

  size_t block_size = 4;
  const String type_label = dumpTypeLabel(path);
  if (type_label.startsWith("MFC")) block_size = 16;
  else if (type_label == "Unknown" && (got % 16u) == 0u && got >= 64u) block_size = 16;

  appendBlockLines(dump, got, block_size,
                   type_label == "ISO15693" || type_label.startsWith("NTAG"));
  return preview;
}

String normalizeReportNewlines(String text) {
  text.replace("\\r\\n", "\n");
  text.replace("\\n", "\n");
  text.replace("\\r", "\r");
  return text;
}

String dumpUidForList(const String& path) {
  String type_label;
  String uid;
  String sak;
  String atqa;
  String source_label;
  readDumpMetaForWeb(path, type_label, uid, sak, atqa, source_label);
  uid = normalizeHexText(uid);
  if (!uid.isEmpty() && uid != "-") {
    return uid;
  }

  String lower = path;
  lower.toLowerCase();
  if (!lower.endsWith(".bin")) return "NOUID";

  File f = LittleFS.open(path, "r");
  if (!f || f.isDirectory()) return "NOUID";

  EmuType guessed = EmuType::N213;
  const size_t file_len = static_cast<size_t>(f.size());
  if (!inferEmuTypeByLength(file_len, guessed)) return "NOUID";

  uint8_t head[8] = {0};
  const size_t got = f.read(head, sizeof(head));
  if (got == 0) return "NOUID";

  String derived = deriveEmuUidFromDump(guessed, head, got);
  derived = normalizeHexText(derived);
  if (derived.isEmpty()) return "NOUID";
  return derived;
}

void rebuildDumpUidViewLabels() {
  for (uint8_t i = 0; i < emu_dump_count; ++i) {
    emu_dump_uid_raw[i] = dumpUidForList(emu_dump_files[i]);
    emu_dump_uid_view[i] = emu_dump_uid_raw[i];
  }

  for (uint8_t i = 0; i < emu_dump_count; ++i) {
    const String uid = emu_dump_uid_raw[i];
    if (uid.isEmpty() || uid == "NOUID") continue;

    uint8_t total = 0;
    uint8_t seq = 0;
    for (uint8_t j = 0; j < emu_dump_count; ++j) {
      if (emu_dump_uid_raw[j] == uid) {
        ++total;
        if (j <= i) ++seq;
      }
    }
    if (total > 1) {
      emu_dump_uid_view[i] = uid + "(" + String(seq) + ")";
    }
  }
}

void showDumpsEmuPopup(const String& uid, const String& type_name) {
  dumps_emu_popup_line1 = "Emulating";
  dumps_emu_popup_line2 = uid;
  dumps_emu_popup_type = type_name;
  dumps_emu_popup_type.replace("\r", " ");
  dumps_emu_popup_type.replace("\n", " ");
  dumps_emu_popup_type.trim();
  dumps_emu_popup_active = true;
  dumps_emu_popup_until_ms = millis() + kDumpEmuPopupMs;
}

void scanDumpDir(const String& dir, uint8_t depth = 0) {
  if (!littlefs_ready || depth > 4 || emu_dump_count >= kEmuDumpMaxFiles) return;

  File root = LittleFS.open(dir);
  if (!root || !root.isDirectory()) return;

  File entry = root.openNextFile();
  while (entry && emu_dump_count < kEmuDumpMaxFiles) {
    String path = String(entry.name());
    if (!path.startsWith("/")) {
      if (dir == "/") {
        path = "/" + path;
      } else {
        path = dir + "/" + path;
      }
    }
    if (entry.isDirectory()) {
      scanDumpDir(path, depth + 1);
    } else if (isDumpFilePath(path)) {
      emu_dump_files[emu_dump_count++] = path;
    }
    entry = root.openNextFile();
  }
}

void refreshDumpFiles(bool filter_by_current_type) {
  emu_dump_count = 0;
  emu_type_scroll = 0;
  emu_type_cursor = 0;

  if (!littlefs_ready) {
    emu_dump_status = "LittleFS unavailable";
    return;
  }

  if (LittleFS.exists(kEmuDumpDir)) {
    scanDumpDir(kEmuDumpDir);
  } else {
    scanDumpDir("/");
  }
  if (emu_dump_count == 0) {
    emu_dump_status = "No dumps (use AP)";
    return;
  }

  // Stable lexicographic sort for deterministic menu order.
  for (uint8_t i = 0; i + 1 < emu_dump_count; ++i) {
    for (uint8_t j = static_cast<uint8_t>(i + 1); j < emu_dump_count; ++j) {
      if (emu_dump_files[j] < emu_dump_files[i]) {
        String tmp = emu_dump_files[i];
        emu_dump_files[i] = emu_dump_files[j];
        emu_dump_files[j] = tmp;
      }
    }
  }

  if (filter_by_current_type) {
    const bool strict_match = dumps_pick_for_emu;
    uint8_t keep = 0;
    for (uint8_t i = 0; i < emu_dump_count; ++i) {
      if (dumpFileLikelyMatchesType(emu_dump_files[i], emu_type, strict_match)) {
        emu_dump_files[keep++] = emu_dump_files[i];
      }
    }
    emu_dump_count = keep;
  }

  rebuildDumpUidViewLabels();

  if (dumps_pick_for_emu) {
    if (dump_file_index > emu_dump_count) dump_file_index = emu_dump_count;
  } else {
    if (dump_file_index >= emu_dump_count) dump_file_index = emu_dump_count > 0 ? static_cast<uint8_t>(emu_dump_count - 1) : 0;
  }

  if (emu_dump_count == 0) {
    emu_dump_status = filter_by_current_type ? String("No ") + emuName(emu_type) + " dumps" : String("No dumps");
  } else {
    emu_dump_status = filter_by_current_type ? String("Dumps(") + emuName(emu_type) + "): " + String(emu_dump_count) : String("Dumps: ") + String(emu_dump_count);
  }
}

String sanitizeDumpFilename(String raw_name) {
  raw_name.trim();
  raw_name.replace("\\", "/");
  const int slash = raw_name.lastIndexOf('/');
  if (slash >= 0 && slash + 1 < static_cast<int>(raw_name.length())) {
    raw_name = raw_name.substring(static_cast<size_t>(slash + 1));
  }
  raw_name.replace(" ", "_");
  if (raw_name.indexOf("..") >= 0 || raw_name.indexOf('/') >= 0 || raw_name.indexOf('\\') >= 0) {
    return "";
  }
  if (!isDumpFilePath(raw_name)) return "";
  return raw_name;
}

String stripDumpExtension(String name) {
  String lower = name;
  lower.toLowerCase();
  if (lower.endsWith(".json")) {
    name.remove(name.length() - 5);
  } else if (lower.endsWith(".bin")) {
    name.remove(name.length() - 4);
  }
  return name;
}

String jsonEscape(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\r", "");
  s.replace("\n", "\\n");
  return s;
}

String urlEncode(String s) {
  static const char* hex = "0123456789ABCDEF";
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(s[i]);
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
      out += static_cast<char>(c);
    } else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }
  return out;
}

bool upsertJsonStringField(String& json, const String& key, const String& value) {
  const String key_tag = "\"" + key + "\"";
  const int key_pos = json.indexOf(key_tag);
  if (key_pos >= 0) {
    int pos = json.indexOf(':', key_pos + key_tag.length());
    if (pos < 0) return false;
    ++pos;
    while (pos < static_cast<int>(json.length()) && (json[pos] == ' ' || json[pos] == '\r' || json[pos] == '\n' || json[pos] == '\t')) ++pos;
    if (pos >= static_cast<int>(json.length()) || json[pos] != '"') return false;
    const int value_start = pos + 1;
    int value_end = value_start;
    while (value_end < static_cast<int>(json.length())) {
      if (json[value_end] == '"' && json[value_end - 1] != '\\') break;
      ++value_end;
    }
    if (value_end >= static_cast<int>(json.length())) return false;
    json = json.substring(0, value_start) + jsonEscape(value) + json.substring(value_end);
    return true;
  }

  const int insert_at = json.indexOf('{');
  if (insert_at < 0) return false;
  json = json.substring(0, insert_at + 1) + "\n  \"" + key + "\": \"" + jsonEscape(value) + "\"," + json.substring(insert_at + 1);
  return true;
}

bool ensureDumpDirExists() {
  if (!littlefs_ready) return false;
  if (LittleFS.exists(kEmuDumpDir)) return true;
  return LittleFS.mkdir(kEmuDumpDir);
}

String bytesToHexUpper(const uint8_t* data, size_t len) {
  static const char* kHex = "0123456789ABCDEF";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out += kHex[(data[i] >> 4) & 0x0F];
    out += kHex[data[i] & 0x0F];
  }
  return out;
}

String deriveEmuUidFromDump(EmuType type, const uint8_t* dump, size_t dump_len) {
  if (!dump || dump_len == 0) return "";

  switch (type) {
    case EmuType::MF1K:
      if (dump_len >= 4) return bytesToHexUpper(dump, 4);
      break;
    case EmuType::MF4K:
      return "041219C3CC9802";
    case EmuType::N213:
    case EmuType::N215:
    case EmuType::N216:
      if (dump_len >= 8) {
        uint8_t uid7[7] = {
          dump[0], dump[1], dump[2],
          dump[4], dump[5], dump[6], dump[7],
        };
        return bytesToHexUpper(uid7, sizeof(uid7));
      }
      break;
    case EmuType::ISO14B:
      if (dump_len >= 4) return bytesToHexUpper(dump, 4);
      break;
    case EmuType::Felica:
      if (dump_len >= 8) return bytesToHexUpper(dump, 8);
      break;
    case EmuType::ISO15:
      if (dump_len >= 8) return bytesToHexUpper(dump, 8);
      break;
    default:
      break;
  }

  return "";
}

String normalizeHexText(String raw) {
  String out;
  out.reserve(raw.length());
  for (size_t i = 0; i < raw.length(); ++i) {
    const char c = raw[i];
    if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
      out += static_cast<char>(toupper(static_cast<unsigned char>(c)));
    }
  }
  return out;
}

String managedTypeLabelForDump(EmuType type, size_t len) {
  return String(emuName(type));
}

String defaultAtqaForType(EmuType type) {
  switch (type) {
    case EmuType::MF1K:
      return "0400";
    case EmuType::MF4K:
      return "0200";
    case EmuType::N213:
    case EmuType::N215:
    case EmuType::N216:
      return "0044";
    default:
      return "";
  }
}

String defaultSakForType(EmuType type) {
  switch (type) {
    case EmuType::MF1K:
      return "08";
    case EmuType::MF4K:
      return "18";
    case EmuType::N213:
    case EmuType::N215:
    case EmuType::N216:
      return "00";
    default:
      return "";
  }
}

bool parseBinTypeHint(String hint, EmuType& out, bool& has_hint) {
  has_hint = false;
  hint.trim();
  if (hint.isEmpty()) return true;
  String lower = hint;
  lower.toLowerCase();
  if (lower == "auto" || lower == "detect") return true;
  has_hint = mapTypeHintToEmuType(lower, out);
  return has_hint;
}

bool writePm3JsonHeader(File& f, const String& file_type, const String& uid, const String& atqa, const String& sak) {
  if (f.print("{\n  \"Created\": \"proxmark3\",\n") == 0) return false;
  if (f.print("  \"FileType\": \"") == 0) return false;
  if (f.print(file_type) == 0) return false;
  if (f.print("\",\n  \"Card\": {\n") == 0) return false;
  if (f.print("    \"UID\": \"") == 0) return false;
  if (f.print(uid) == 0) return false;
  if (f.print("\"") == 0) return false;
  if (!atqa.isEmpty()) {
    if (f.print(",\n    \"ATQA\": \"") == 0) return false;
    if (f.print(atqa) == 0) return false;
    if (f.print("\"") == 0) return false;
  }
  if (!sak.isEmpty()) {
    if (f.print(",\n    \"SAK\": \"") == 0) return false;
    if (f.print(sak) == 0) return false;
    if (f.print("\"") == 0) return false;
  }
  return f.print("\n  },\n  \"blocks\": {\n") > 0;
}

bool writePm3JsonFooter(File& f) {
  return f.print("  }\n}\n") > 0;
}

bool writeManagedJsonHeader(File& f,
                            const String& name,
                            const String& type_label,
                            const String& source_kind,
                            const String& file_type,
                            const String& uid,
                            const String& atqa,
                            const String& sak,
                            const String& uid_source,
                            const String& atqa_source,
                            const String& sak_source) {
  if (f.print("{\n  \"name\": \"") == 0) return false;
  if (f.print(jsonEscape(name)) == 0) return false;
  if (f.print("\",\n  \"format\": \"grovenfc-dump-v1\",\n  \"type\": \"") == 0) return false;
  if (f.print(type_label) == 0) return false;
  if (f.print("\",\n  \"source\": \"") == 0) return false;
  if (f.print(source_kind) == 0) return false;
  if (f.print("\",\n  \"Created\": \"GroveNFC\",\n  \"FileType\": \"") == 0) return false;
  if (f.print(file_type) == 0) return false;
  if (f.print("\",\n  \"Card\": {\n    \"UID\": \"") == 0) return false;
  if (f.print(uid) == 0) return false;
  if (f.print("\"") == 0) return false;
  if (!atqa.isEmpty()) {
    if (f.print(",\n    \"ATQA\": \"") == 0) return false;
    if (f.print(atqa) == 0) return false;
    if (f.print("\"") == 0) return false;
  }
  if (!sak.isEmpty()) {
    if (f.print(",\n    \"SAK\": \"") == 0) return false;
    if (f.print(sak) == 0) return false;
    if (f.print("\"") == 0) return false;
  }
  if (f.print("\n  },\n  \"FieldSource\": {\n") == 0) return false;
  if (f.print("    \"UID\": \"") == 0) return false;
  if (f.print(uid_source) == 0) return false;
  if (f.print("\",\n    \"ATQA\": \"") == 0) return false;
  if (f.print(atqa_source) == 0) return false;
  if (f.print("\",\n    \"SAK\": \"") == 0) return false;
  if (f.print(sak_source) == 0) return false;
  return f.print("\"\n  },\n  \"blocks\": {\n") > 0;
}

bool writePm3BlockLine(File& f, size_t index, const uint8_t* block, size_t block_size, bool with_comma) {
  if (f.print("    \"") == 0) return false;
  if (f.print(index) == 0) return false;
  if (f.print("\": \"") == 0) return false;
  if (f.print(bytesToHexUpper(block, block_size)) == 0) return false;
  if (with_comma) {
    if (f.print("\",\n") == 0) return false;
  } else {
    if (f.print("\"\n") == 0) return false;
  }
  return true;
}

String uidFromDumpBytes(EmuType type, const uint8_t* data, size_t len) {
  if (!data || len == 0) return "11223344";
  if ((type == EmuType::N213 || type == EmuType::N215 || type == EmuType::N216) && len >= 8) {
    uint8_t uid[7] = {data[0], data[1], data[2], data[4], data[5], data[6], data[7]};
    return bytesToHexUpper(uid, sizeof(uid));
  }
  if ((type == EmuType::MF1K || type == EmuType::MF4K) && len >= 4) return bytesToHexUpper(data, 4);
  if (type == EmuType::ISO15 && len >= 8) return bytesToHexUpper(data, 8);
  return "11223344";
}

String managedFileTypeForDump(EmuType type, size_t len, bool mfu_family) {
  switch (type) {
    case EmuType::MF1K:
      return "mfc v2";
    case EmuType::MF4K:
      return "mfc v3";
    case EmuType::N213:
    case EmuType::N215:
    case EmuType::N216:
      return mfu_family ? "mfu" : "mfu";
    case EmuType::ISO14B:
      return "14b v2";
    case EmuType::Felica:
      return "felica";
    case EmuType::ISO15:
      return "15 v4";
    default:
      return "raw";
  }
}

bool writeManagedDumpJson(const String& path,
                          const String& name,
                          EmuType type,
                          const uint8_t* data,
                          size_t len,
                          const String& uid_hint,
                          const String& atqa,
                          const String& sak,
                          bool mfu_family,
                          const String& source_kind,
                          const String& uid_source,
                          const String& atqa_source,
                          const String& sak_source) {
  if (!data || len == 0) return false;
  const size_t block_size = (type == EmuType::MF1K || type == EmuType::MF4K) ? 16u : 4u;
  if ((len % block_size) != 0) return false;

  File f = LittleFS.open(path, "w");
  if (!f) return false;
  const String uid = uid_hint.isEmpty() ? uidFromDumpBytes(type, data, len) : normalizeHexText(uid_hint);
  const String file_type = managedFileTypeForDump(type, len, mfu_family);
  const String type_label = managedTypeLabelForDump(type, len);
  if (!writeManagedJsonHeader(f,
                              name,
                              type_label,
                              source_kind,
                              file_type,
                              uid,
                              normalizeHexText(atqa),
                              normalizeHexText(sak),
                              uid_source,
                              atqa_source,
                              sak_source)) {
    f.close();
    return false;
  }
  const size_t blocks = len / block_size;
  for (size_t i = 0; i < blocks; ++i) {
    if (!writePm3BlockLine(f, i, data + i * block_size, block_size, i + 1 < blocks)) {
      f.close();
      return false;
    }
  }
  const bool ok = writePm3JsonFooter(f);
  f.close();
  return ok;
}

bool normalizeDumpFileInPlace(const String& path, String& status_text, const String& bin_type_hint) {
  status_text = "Kept raw";
  String lower = path;
  lower.toLowerCase();

  if (lower.endsWith(".bin")) {
    File bf = LittleFS.open(path, "r");
    if (!bf || bf.isDirectory()) {
      status_text = "BIN open failed";
      return false;
    }
    const size_t data_len = static_cast<size_t>(bf.size());
    if (data_len == 0 || data_len > kEmuDumpMaxBytes) {
      status_text = "BIN size unsupported";
      return false;
    }
    uint8_t data[kEmuDumpMaxBytes] = {0};
    const size_t got = bf.read(data, data_len);
    bf.close();
    if (got != data_len) {
      status_text = "BIN read failed";
      return false;
    }

    EmuType type = EmuType::N213;
    bool has_hint = false;
    parseBinTypeHint(bin_type_hint, type, has_hint);
    if (!has_hint) {
      if (!inferEmuTypeByLength(data_len, type)) {
        if (data_len <= kIso15DumpMaxBytes && (data_len % 4u) == 0) {
          type = EmuType::ISO15;
        } else {
          status_text = "BIN type unknown; choose type";
          return false;
        }
      }
    }

    if (!dumpLengthMatchesType(type, data_len)) {
      status_text = "BIN length/type mismatch";
      return false;
    }

    String name = path;
    const int slash = name.lastIndexOf('/');
    if (slash >= 0) name = name.substring(static_cast<size_t>(slash + 1));
    name = stripDumpExtension(name);
    String uid = uidFromDumpBytes(type, data, data_len);
    String atqa = defaultAtqaForType(type);
    String sak = defaultSakForType(type);

    String target_path = path;
    if (target_path.endsWith(".bin")) {
      target_path = target_path.substring(0, target_path.length() - 4) + ".json";
    }
    if (!writeManagedDumpJson(target_path,
                              name,
                              type,
                              data,
                              data_len,
                              uid,
                              atqa,
                              sak,
                              false,
                              "raw-bin",
                              "computed-from-data",
                              atqa.isEmpty() ? "none" : "type-default",
                              sak.isEmpty() ? "none" : "type-default")) {
      status_text = "BIN normalize failed";
      return false;
    }
    if (target_path != path) {
      LittleFS.remove(path);
    }
    status_text = String("BIN imported ") + managedTypeLabelForDump(type, data_len);
    return true;
  }

  if (!lower.endsWith(".json")) return true;

  File f = LittleFS.open(path, "r");
  if (!f || f.isDirectory()) {
    status_text = "Open failed";
    return false;
  }
  String json;
  json.reserve(static_cast<size_t>(f.size()) + 1);
  while (f.available()) json += static_cast<char>(f.read());
  f.close();

  String payload;
  String source_kind = "json";
  String created_by;
  if (extractJsonValue(json, "format", payload) && payload == "grovenfc-dump-v1") {
    source_kind = "grovenfc-json";
  } else if (extractJsonValue(json, "Created", created_by)) {
    String created_lower = created_by;
    created_lower.toLowerCase();
    source_kind = created_lower.indexOf("proxmark") >= 0 ? "pm3-json" : "json";
  }

  String file_type_hint;
  String type_hint;
  EmuType hinted_type = EmuType::N213;
  bool has_hint = false;
  bool mfu_family = false;
  if (extractJsonValue(json, "FileType", file_type_hint) && mapPm3FileTypeToEmuType(file_type_hint, hinted_type)) {
    has_hint = true;
    mfu_family = isPm3MfuFamilyFileType(file_type_hint);
  } else if ((extractJsonValue(json, "type", type_hint) || extractJsonValue(json, "tagType", type_hint) ||
              extractJsonValue(json, "protocol", type_hint) || extractJsonValue(json, "emuType", type_hint)) &&
             mapTypeHintToEmuType(type_hint, hinted_type)) {
    has_hint = true;
  }

  uint8_t data[kEmuDumpMaxBytes] = {0};
  size_t data_len = 0;
  size_t parsed_blocks = 0;
  bool parsed = false;
  if (extractJsonValue(json, "blocks", payload) && payload.startsWith("{")) {
    uint8_t block_hint = 0;
    if (has_hint) {
      block_hint = (hinted_type == EmuType::MF1K || hinted_type == EmuType::MF4K) ? 16 : 4;
    }
    parsed = parseJsonBlocksObject(payload, data, sizeof(data), block_hint, data_len, parsed_blocks);
    if (parsed && hinted_type == EmuType::ISO15) {
      (void)injectPm3Iso15UidHeader(json, data, data_len);
    }
  } else if (extractJsonValue(json, "data", payload) || extractJsonValue(json, "dump", payload) ||
             extractJsonValue(json, "bytes", payload) || extractJsonValue(json, "hex", payload) || extractJsonValue(json, "bin", payload)) {
    parsed = payload.startsWith("[") ? parseByteArray(payload, data, data_len) : parseHexBytes(payload, data, data_len);
  }
  if (!parsed || data_len == 0) {
    status_text = "Import kept raw";
    return true;
  }

  EmuType type = hinted_type;
  if (!has_hint || mfu_family) {
    if (!inferEmuTypeByLength(data_len, type)) {
      if (data_len <= kIso15DumpMaxBytes && (data_len % 4u) == 0) type = EmuType::ISO15;
      else if (has_hint) type = hinted_type;
      else {
        status_text = "Unknown dump length";
        return true;
      }
    }
  }

  const bool type_ok = (type == EmuType::MF1K && data_len == 1024) ||
                       (type == EmuType::MF4K && data_len == 4096) ||
                       (type == EmuType::N213 && data_len == 180) ||
                       (type == EmuType::N215 && data_len == 540) ||
                       (type == EmuType::N216 && data_len == 924) ||
                       (type == EmuType::Felica && data_len == 448) ||
                       (type == EmuType::ISO15 && data_len > 0 && data_len <= kIso15DumpMaxBytes && (data_len % 4u) == 0) ||
                       (type == EmuType::ISO14B && data_len > 0 && (data_len % 4u) == 0);
  if (!type_ok) {
    status_text = "Length/type mismatch";
    return true;
  }

  String name;
  extractJsonValue(json, "name", name) || extractJsonValue(json, "Name", name);
  name.trim();
  if (name.isEmpty()) {
    String filename = path;
    const int slash = filename.lastIndexOf('/');
    if (slash >= 0) filename = filename.substring(static_cast<size_t>(slash + 1));
    name = stripDumpExtension(filename);
  }
  String uid;
  const bool has_uid = extractJsonValue(json, "UID", uid) || extractJsonValue(json, "uid", uid);
  uid = normalizeHexText(uid);
  if (uid.isEmpty()) uid = uidFromDumpBytes(type, data, data_len);

  String atqa;
  String sak;
  const bool has_atqa = extractJsonValue(json, "ATQA", atqa) || extractJsonValue(json, "atqa", atqa);
  const bool has_sak = extractJsonValue(json, "SAK", sak) || extractJsonValue(json, "sak", sak);
  atqa = normalizeHexText(atqa);
  sak = normalizeHexText(sak);
  if (atqa.isEmpty()) atqa = defaultAtqaForType(type);
  if (sak.isEmpty()) sak = defaultSakForType(type);

  const String uid_source = has_uid ? "json-card" : "computed-from-data";
  const String atqa_source = has_atqa ? "json-card" : (atqa.isEmpty() ? "none" : "type-default");
  const String sak_source = has_sak ? "json-card" : (sak.isEmpty() ? "none" : "type-default");

  if (!writeManagedDumpJson(path,
                            name,
                            type,
                            data,
                            data_len,
                            uid,
                            atqa,
                            sak,
                            mfu_family,
                            source_kind,
                            uid_source,
                            atqa_source,
                            sak_source)) {
    status_text = "Normalize failed";
    return false;
  }
  status_text = String("Imported ") + managedTypeLabelForDump(type, data_len);
  return true;
}

bool saveDumpRawJson(const String& path, String content, String name, String& status_text) {
  status_text = "Save failed";
  if (!path.endsWith(".json")) {
    status_text = "JSON only";
    return false;
  }
  content.trim();
  if (!content.startsWith("{")) {
    status_text = "Invalid JSON";
    return false;
  }
  if (!name.isEmpty()) {
    if (!upsertJsonStringField(content, "name", name)) {
      status_text = "Name update failed";
      return false;
    }
  }
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  const bool wrote = f.print(content) > 0;
  f.close();
  if (!wrote) return false;
  normalizeDumpFileInPlace(path, status_text, "auto");
  return true;
}

void removeLegacyExampleDumps() {
  if (!littlefs_ready) return;
  if (!ensureDumpDirExists()) return;

  LittleFS.remove(String(kEmuDumpDir) + "/example_mfc1k.bin");
  LittleFS.remove(String(kEmuDumpDir) + "/example_mfc4k.bin");
  LittleFS.remove(String(kEmuDumpDir) + "/example_ntag213.bin");
  LittleFS.remove(String(kEmuDumpDir) + "/example_ntag215.bin");
  LittleFS.remove(String(kEmuDumpDir) + "/example_ntag216.bin");
  LittleFS.remove(String(kEmuDumpDir) + "/example_felica.bin");
  LittleFS.remove(String(kEmuDumpDir) + "/example_mfc1k.json");
  LittleFS.remove(String(kEmuDumpDir) + "/example_mfc4k.json");
  LittleFS.remove(String(kEmuDumpDir) + "/example_ntag213.json");
  LittleFS.remove(String(kEmuDumpDir) + "/example_ntag215.json");
  LittleFS.remove(String(kEmuDumpDir) + "/example_ntag216.json");
  LittleFS.remove(String(kEmuDumpDir) + "/example_felica.json");
  LittleFS.remove(String(kEmuDumpDir) + "/example_iso14b.json");
  LittleFS.remove(String(kEmuDumpDir) + "/example_iso15693.json");
}

uint8_t dumpsBrowseCount() {
  return dumps_pick_for_emu ? static_cast<uint8_t>(emu_dump_count + 1) : emu_dump_count;
}

bool dumpsSelectionIsBack() {
  return dumps_pick_for_emu && dump_file_index == 0;
}

bool dumpsHasSelectedFile() {
  if (!dumps_pick_for_emu) {
    return dump_file_index < emu_dump_count;
  }
  return dump_file_index > 0 && dump_file_index <= emu_dump_count;
}

uint8_t dumpsSelectedFileIndex() {
  if (!dumps_pick_for_emu) return dump_file_index;
  if (dump_file_index == 0) return 0;
  return static_cast<uint8_t>(dump_file_index - 1);
}

String htmlEscape(String s) {
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  return s;
}

String detectDumpSourceLabel(const String& json_text) {
  String source;
  if (extractJsonValue(json_text, "source", source) && !source.isEmpty()) {
    return source;
  }
  String format;
  if (extractJsonValue(json_text, "format", format) && format == "grovenfc-dump-v1") {
    return "grovenfc-json";
  }
  String created;
  if (extractJsonValue(json_text, "Created", created)) {
    String lower = created;
    lower.toLowerCase();
    if (lower.indexOf("proxmark") >= 0) return "pm3-json";
  }
  return "json";
}

void readDumpMetaForWeb(const String& path,
                        String& type_label,
                        String& uid,
                        String& sak,
                        String& atqa,
                        String& source_label) {
  type_label = dumpTypeLabel(path);
  uid = "-";
  sak = "-";
  atqa = "-";
  source_label = "raw-bin";

  String lower = path;
  lower.toLowerCase();
  if (!lower.endsWith(".json")) return;

  File f = LittleFS.open(path, "r");
  if (!f || f.isDirectory()) return;

  String json;
  const size_t read_limit = min(static_cast<size_t>(f.size()), static_cast<size_t>(4096));
  json.reserve(read_limit + 1);
  for (size_t i = 0; i < read_limit && f.available(); ++i) {
    json += static_cast<char>(f.read());
  }

  String value;
  if (extractJsonValue(json, "UID", value) || extractJsonValue(json, "uid", value)) {
    value = normalizeHexText(value);
    if (!value.isEmpty()) uid = value;
  }
  if (extractJsonValue(json, "SAK", value) || extractJsonValue(json, "sak", value)) {
    value = normalizeHexText(value);
    if (!value.isEmpty()) sak = value;
  }
  if (extractJsonValue(json, "ATQA", value) || extractJsonValue(json, "atqa", value)) {
    value = normalizeHexText(value);
    if (!value.isEmpty()) atqa = value;
  }
  source_label = detectDumpSourceLabel(json);
}

void handleEmuApPortal() {
  if (emu_ap_active) {
    emu_ap_dns.processNextRequest();
    emu_ap_server.handleClient();
  }
}

void stopEmuApPortal() {
  if (!emu_ap_active) return;
  if (emu_ap_upload_file) {
    emu_ap_upload_file.close();
  }
  emu_ap_upload_session_active = false;
  emu_ap_uploaded_count = 0;
  emu_ap_server.stop();
  emu_ap_dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  emu_ap_active = false;
  refreshDumpFiles(dumps_pick_for_emu);
  if (emu_dump_count == 0) {
    emu_dump_status = "AP stopped | no dumps";
  } else {
    emu_dump_status = String("AP stopped | dumps: ") + String(emu_dump_count);
  }
}

bool startEmuApPortal() {
  if (emu_ap_active) return true;
  if (!littlefs_ready) {
    emu_dump_status = "LittleFS unavailable";
    return false;
  }
  if (!ensureDumpDirExists()) {
    emu_dump_status = "Create /dumps failed";
    return false;
  }

  WiFi.mode(WIFI_AP);
  bool ap_ok = false;
  if (strlen(kEmuApPass) == 0) {
    ap_ok = WiFi.softAP(kEmuApSsid);
  } else {
    ap_ok = WiFi.softAP(kEmuApSsid, kEmuApPass);
  }
  if (!ap_ok) {
    emu_dump_status = "AP start failed";
    return false;
  }

  static bool routes_ready = false;
  if (!routes_ready) {
    emu_ap_server.on("/", HTTP_GET, []() {
      refreshDumpFiles(false);
      String html =
        "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>GroveNFC Dump Library</title><style>body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,monospace;margin:0;padding:14px;background:#101214;color:#eee}main{max-width:1240px;margin:0 auto}a{color:#68d8ff;text-decoration:none}button{margin:4px;padding:7px 12px;background:#1f8a70;color:#fff;border:0;border-radius:4px}input,textarea,select{background:#181c20;color:#eee;border:1px solid #38424a;border-radius:4px;padding:7px}table{border-collapse:collapse;width:100%;min-width:680px}td,th{border-bottom:1px solid #30363d;padding:7px;text-align:left;white-space:nowrap}.table-wrap{width:100%;overflow-x:auto;-webkit-overflow-scrolling:touch}.muted{color:#aab}.pill{display:inline-block;padding:2px 7px;border:1px solid #446;border-radius:999px;color:#cde}.actions a{margin-right:10px}.meta{font-size:12px;color:#9fb}.filebtn{cursor:pointer;display:inline-block;padding:7px 14px;background:#1a2d40;color:#68d8ff;border:1px solid #2a4a6a;border-radius:4px;vertical-align:middle;max-width:220px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:14px}.up-row{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-bottom:6px}.up-row>*{margin:0}@media (max-width:768px){body{padding:10px}button{padding:7px 10px}td,th{padding:6px;font-size:12px}.filebtn{max-width:100%;width:100%;box-sizing:border-box}.up-row{flex-direction:column;align-items:stretch}.up-row select,.up-row button{width:100%;box-sizing:border-box}}</style></head>"
        "<body><main><h2>GroveNFC Dump Library</h2>"
        "<p class='muted'>Manage dumps from web. PM3 JSON and BIN imports are normalized into GroveNFC managed JSON (name/type/source/UID/SAK/ATQA/blocks).</p>"
        "<form method='POST' action='/upload' enctype='multipart/form-data'>"
        "<div class='up-row'>"
        "<label class='filebtn' id='fl' for='di'>Choose dumps...</label>"
        "<input type='file' id='di' name='dump' accept='.json,.bin' multiple style='display:none' onchange=\"var n=this.files.length;document.getElementById('fl').textContent=n>1?n+' files selected':n?this.files[0].name:'Choose dumps...'\">"
        "<select name='bin_type'><option value='auto' selected>BIN: Auto detect</option><option value='mfc1k'>BIN: Mifare Classic 1K/4K</option><option value='ntag213'>BIN: NTAG213</option><option value='ntag215'>BIN: NTAG215</option><option value='ntag216'>BIN: NTAG216</option><option value='iso14b'>BIN: ISO14443B</option><option value='iso15'>BIN: ISO15693</option><option value='felica'>BIN: Felica</option></select>"
        "<button type='submit'>Import dumps</button></div></form>"
        "<p class='meta'>BIN type can be selected manually. Auto mode detects by length first.</p>"
        "<form method='POST' action='/delete-selected'><div class='table-wrap'><table><tr><th></th><th>Dump</th><th>Type</th><th>UID</th><th>Source</th><th>Size</th><th>File</th><th>Actions</th></tr>";

      for (uint8_t i = 0; i < emu_dump_count; ++i) {
        String full = emu_dump_files[i];
        String name = full;
        const int slash = name.lastIndexOf('/');
        if (slash >= 0 && slash + 1 < static_cast<int>(name.length())) {
          name = name.substring(static_cast<size_t>(slash + 1));
        }
        const String display_name = dumpDisplayName(full, 32);
        File f = LittleFS.open(full, "r");
        const size_t size = f ? static_cast<size_t>(f.size()) : 0;
        String type_label;
        String uid;
        String sak;
        String atqa;
        String source_label;
        readDumpMetaForWeb(full, type_label, uid, sak, atqa, source_label);

        html += "<tr><td><input type='checkbox' name='name' value='" + htmlEscape(name) + "'></td>";
        html += "<td>" + htmlEscape(display_name) + "</td>";
        html += "<td><span class='pill'>" + htmlEscape(type_label) + "</span></td>";
        html += "<td>" + htmlEscape(uid) + "</td>";
        html += "<td>" + htmlEscape(source_label) + "</td><td>" + String(size) + "</td><td title='" + htmlEscape(name) + "'>" + htmlEscape(name) + "</td>";
        html += "<td class='actions'><a href='/edit?name=" + urlEncode(name) + "'>Edit</a><a href='/delete?name=" + urlEncode(name) + "' onclick='return confirm(\"Delete " + htmlEscape(name) + "?\")'>Delete</a></td></tr>";
      }
      if (emu_dump_count == 0) {
        html += "<tr><td colspan='8'>No dump files</td></tr>";
      }
      html += "</table></div><p><button type='submit'>Delete selected</button>";
      html += "<button formaction='/delete-all' onclick='return confirm(\"Delete all dumps?\")'>Delete all</button>";
      html += "<button formaction='/' formmethod='GET'>Refresh</button></p></form>";
      html += "</main></body></html>";
      emu_ap_server.send(200, "text/html", html);
    });

    emu_ap_server.on("/edit", HTTP_GET, []() {
      String name = sanitizeDumpFilename(emu_ap_server.arg("name"));
      if (name.isEmpty()) {
        emu_ap_server.send(400, "text/plain", "Invalid file name");
        return;
      }
      const String path = String(kEmuDumpDir) + "/" + name;
      File f = LittleFS.open(path, "r");
      if (!f || f.isDirectory()) {
        emu_ap_server.send(404, "text/plain", "Not found");
        return;
      }
      String content;
      String lower = name;
      lower.toLowerCase();
      if (lower.endsWith(".json")) {
        content.reserve(static_cast<size_t>(f.size()) + 1);
        while (f.available()) content += static_cast<char>(f.read());
      }
      const String display_name = dumpDisplayName(path, 48);
      String html =
        "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Edit Dump</title><style>body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,monospace;background:#101214;color:#eee;padding:14px}a{color:#68d8ff}input,textarea{box-sizing:border-box;width:100%;background:#181c20;color:#eee;border:1px solid #38424a;border-radius:4px;padding:8px}textarea{height:58vh;font-family:ui-monospace,Menlo,monospace;font-size:12px}button{margin-top:10px;padding:8px 14px;background:#1f8a70;color:#fff;border:0;border-radius:4px}.muted{color:#aab}.row{max-width:960px}</style></head><body><p><a href='/'>Back</a></p>";
      html += "<h2>" + htmlEscape(display_name) + "</h2><p class='muted'>" + htmlEscape(dumpTypeLabel(path)) + " | " + htmlEscape(name) + " | " + String(static_cast<size_t>(f.size())) + " bytes</p>";
      html += "<div class='row'><form method='POST' action='/save'><input type='hidden' name='name' value='" + htmlEscape(name) + "'>";
      html += "<label>Display name</label><input name='display' value='" + htmlEscape(display_name) + "'>";
      if (lower.endsWith(".json")) {
        html += "<p><label>Internal JSON data</label><textarea name='content'>" + htmlEscape(content) + "</textarea></p><button type='submit'>Save dump</button>";
      } else {
        html += "<p class='muted'>BIN files are normalized into managed JSON during import. Re-import from the list page if needed.</p>";
      }
      html += "</form></div></body></html>";
      emu_ap_server.send(200, "text/html", html);
    });

    emu_ap_server.on("/save", HTTP_POST, []() {
      String name = sanitizeDumpFilename(emu_ap_server.arg("name"));
      if (name.isEmpty()) {
        emu_ap_server.send(400, "text/plain", "Invalid file name");
        return;
      }
      const String path = String(kEmuDumpDir) + "/" + name;
      String status;
      if (!saveDumpRawJson(path, emu_ap_server.arg("content"), emu_ap_server.arg("display"), status)) {
        emu_ap_server.send(400, "text/plain", status);
        return;
      }
      refreshDumpFiles(false);
      emu_dump_status = status;
      emu_ap_server.sendHeader("Location", "/edit?name=" + urlEncode(name));
      emu_ap_server.send(303, "text/plain", status);
    });

    emu_ap_server.on("/delete", HTTP_GET, []() {
      String name = sanitizeDumpFilename(emu_ap_server.arg("name"));
      if (name.isEmpty()) {
        emu_ap_server.send(400, "text/plain", "Invalid file name");
        return;
      }
      const String path = String(kEmuDumpDir) + "/" + name;
      if (LittleFS.exists(path)) {
        LittleFS.remove(path);
      }
      refreshDumpFiles(false);
      emu_dump_status = String("Deleted ") + shortDumpName(path, 14);
      emu_ap_server.sendHeader("Location", "/");
      emu_ap_server.send(303, "text/plain", "Deleted");
    });

    emu_ap_server.on("/delete-selected", HTTP_POST, []() {
      const int count = emu_ap_server.args();
      uint8_t deleted = 0;
      for (int i = 0; i < count; ++i) {
        if (emu_ap_server.argName(i) != "name") continue;
        String name = sanitizeDumpFilename(emu_ap_server.arg(i));
        if (name.isEmpty()) continue;
        const String path = String(kEmuDumpDir) + "/" + name;
        if (LittleFS.exists(path) && LittleFS.remove(path)) ++deleted;
      }
      refreshDumpFiles(false);
      emu_dump_status = String("Deleted ") + String(deleted);
      emu_ap_server.sendHeader("Location", "/");
      emu_ap_server.send(303, "text/plain", "Deleted");
    });

    emu_ap_server.on("/delete-all", HTTP_POST, []() {
      refreshDumpFiles(false);
      uint8_t deleted = 0;
      for (uint8_t i = 0; i < emu_dump_count; ++i) {
        if (LittleFS.exists(emu_dump_files[i]) && LittleFS.remove(emu_dump_files[i])) ++deleted;
      }
      refreshDumpFiles(false);
      emu_dump_status = String("Deleted all ") + String(deleted);
      emu_ap_server.sendHeader("Location", "/");
      emu_ap_server.send(303, "text/plain", "Deleted");
    });

    emu_ap_server.on(
      "/upload",
      HTTP_POST,
      []() {
        const String bin_type = emu_ap_server.hasArg("bin_type") ? emu_ap_server.arg("bin_type") : String("auto");
        uint8_t imported = 0;
        uint8_t failed = 0;
        String last_status;
        for (uint8_t i = 0; i < emu_ap_uploaded_count; ++i) {
          if (emu_ap_uploaded_paths[i].isEmpty()) continue;
          String one_status;
          if (normalizeDumpFileInPlace(emu_ap_uploaded_paths[i], one_status, bin_type)) {
            ++imported;
          } else {
            ++failed;
          }
          if (!one_status.isEmpty()) last_status = one_status;
        }
        emu_ap_uploaded_count = 0;
        emu_ap_upload_session_active = false;
        refreshDumpFiles(false);
        if (failed == 0) {
          emu_dump_status = String("Imported ") + String(imported);
          if (!last_status.isEmpty()) emu_dump_status += String(" | ") + last_status;
        } else {
          emu_dump_status = String("Imported ") + String(imported) + String(", failed ") + String(failed);
        }
        emu_ap_server.sendHeader("Location", "/");
        emu_ap_server.send(303, "text/plain", "OK");
      },
      []() {
        HTTPUpload& upload = emu_ap_server.upload();
        if (upload.status == UPLOAD_FILE_START) {
          if (!emu_ap_upload_session_active) {
            emu_ap_upload_session_active = true;
            emu_ap_uploaded_count = 0;
          }
          emu_ap_last_upload_path = "";
          const String safe_name = sanitizeDumpFilename(upload.filename);
          if (safe_name.isEmpty() || !ensureDumpDirExists()) {
            emu_dump_status = "Upload reject (json/bin only)";
            return;
          }
          const String path = String(kEmuDumpDir) + "/" + safe_name;
          emu_ap_upload_file = LittleFS.open(path, "w");
          if (!emu_ap_upload_file) {
            emu_dump_status = "Upload open failed";
          } else {
            emu_ap_last_upload_path = path;
            emu_dump_status = String("Uploading ") + shortDumpName(path, 14);
          }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (emu_ap_upload_file) {
            emu_ap_upload_file.write(upload.buf, upload.currentSize);
          }
        } else if (upload.status == UPLOAD_FILE_END) {
          if (emu_ap_upload_file) {
            emu_ap_upload_file.close();
            if (!emu_ap_last_upload_path.isEmpty() && emu_ap_uploaded_count < kEmuDumpMaxFiles) {
              emu_ap_uploaded_paths[emu_ap_uploaded_count++] = emu_ap_last_upload_path;
            }
            emu_dump_status = String("Upload queued ") + String(emu_ap_uploaded_count);
          }
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
          if (emu_ap_upload_file) {
            emu_ap_upload_file.close();
          }
          emu_dump_status = "Upload aborted";
        }
      });

    emu_ap_server.onNotFound([]() {
      emu_ap_server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/");
      emu_ap_server.send(302, "text/plain", "GroveNFC Dumps");
    });

    routes_ready = true;
  }

  emu_ap_dns.start(53, "*", WiFi.softAPIP());
  emu_ap_server.begin();
  emu_ap_active = true;
  refreshDumpFiles(false);
  emu_dump_status = String("AP OPEN ") + WiFi.softAPIP().toString();
  Serial.printf("[AP] Dump portal: SSID=%s PASS=<open> IP=%s\n", kEmuApSsid, WiFi.softAPIP().toString().c_str());
  return true;
}

const char* emuDumpPrefKey(EmuType type) {
  switch (type) {
    case EmuType::MF1K:
      return "mf1k";
    case EmuType::MF4K:
      return "mf4k";
    case EmuType::N213:
      return "n213";
    case EmuType::N215:
      return "n215";
    case EmuType::N216:
      return "n216";
    case EmuType::ISO14B:
      return "iso14b";
    case EmuType::Felica:
      return "felica";
    case EmuType::ISO15:
      return "iso15";
    default:
      return "unknown";
  }
}

String loadLastDumpForType(EmuType type) {
  if (!prefs.begin(kEmuDumpPrefNs, true)) return "";
  const String path = prefs.getString(emuDumpPrefKey(type), "");
  prefs.end();
  return path;
}

void saveLastDumpForType(EmuType type, const String& path) {
  if (!prefs.begin(kEmuDumpPrefNs, false)) return;
  prefs.putString(emuDumpPrefKey(type), path);
  prefs.end();
}

bool mapTypeHintToEmuType(String hint, EmuType& out) {
  hint.toLowerCase();
  if (hint.indexOf("ntag216") >= 0 || hint.indexOf("n216") >= 0 || hint.indexOf("216") >= 0) {
    out = EmuType::N216;
    return true;
  }
  if (hint.indexOf("ntag215") >= 0 || hint.indexOf("n215") >= 0 || hint.indexOf("215") >= 0) {
    out = EmuType::N215;
    return true;
  }
  if (hint.indexOf("ntag213") >= 0 || hint.indexOf("n213") >= 0 || hint.indexOf("213") >= 0) {
    out = EmuType::N213;
    return true;
  }
  if (hint.indexOf("mf4k") >= 0 || hint.indexOf("mfc4k") >= 0 || hint.indexOf("4k") >= 0) {
    out = EmuType::MF4K;
    return true;
  }
  if (hint.indexOf("mifare") >= 0 || hint.indexOf("mf1k") >= 0 || hint.indexOf("mfc1k") >= 0 ||
      hint.indexOf("1k") >= 0) {
    out = EmuType::MF1K;
    return true;
  }
  if (hint.indexOf("iso15693") >= 0 || hint.indexOf("iso15") >= 0 || hint.indexOf("15") >= 0) {
    out = EmuType::ISO15;
    return true;
  }
  if (hint.indexOf("iso14443b") >= 0 || hint.indexOf("14b") >= 0) {
    out = EmuType::ISO14B;
    return true;
  }
  if (hint.indexOf("felica") >= 0 || hint.indexOf("feli") >= 0) {
    out = EmuType::Felica;
    return true;
  }
  return false;
}

bool inferEmuTypeByLength(size_t len, EmuType& out) {
  if (len == 180) {
    out = EmuType::N213;
    return true;
  }
  if (len == 540) {
    out = EmuType::N215;
    return true;
  }
  if (len == 924) {
    out = EmuType::N216;
    return true;
  }
  if (len == 1024) {
    out = EmuType::MF1K;
    return true;
  }
  if (len == 4096) {
    out = EmuType::MF4K;
    return true;
  }
  if (len == 448) {
    out = EmuType::Felica;
    return true;
  }
  if (len == 64) {
    out = EmuType::ISO15;
    return true;
  }
  return false;
}

bool mapEmuTypeToDumpType(EmuType in, DumpTagType& out) {
  switch (in) {
    case EmuType::MF1K:
      out = DumpTagType::Mifare1K;
      return true;
    case EmuType::MF4K:
      out = DumpTagType::Mifare4K;
      return true;
    case EmuType::N213:
      out = DumpTagType::Ntag213;
      return true;
    case EmuType::N215:
      out = DumpTagType::Ntag215;
      return true;
    case EmuType::N216:
      out = DumpTagType::Ntag216;
      return true;
    case EmuType::ISO14B:
      out = DumpTagType::ISO14B;
      return true;
    case EmuType::Felica:
      out = DumpTagType::Felica;
      return true;
    case EmuType::ISO15:
      out = DumpTagType::ISO15;
      return true;
    default:
      return false;
  }
}

bool parseHexBytes(const String& raw, uint8_t* out, size_t& out_len) {
  String hex;
  hex.reserve(raw.length());
  for (size_t i = 0; i < raw.length(); ++i) {
    const char c = raw[i];
    if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
      hex += c;
    }
  }

  if (hex.length() < 2) return false;
  if (hex.length() & 1u) hex.remove(hex.length() - 1);
  out_len = min(static_cast<size_t>(hex.length() / 2), static_cast<size_t>(kEmuDumpMaxBytes));

  for (size_t i = 0; i < out_len; ++i) {
    const char c1 = hex[i * 2];
    const char c2 = hex[i * 2 + 1];
    const uint8_t hi = static_cast<uint8_t>((c1 <= '9') ? (c1 - '0') : ((c1 & 0x5F) - 'A' + 10));
    const uint8_t lo = static_cast<uint8_t>((c2 <= '9') ? (c2 - '0') : ((c2 & 0x5F) - 'A' + 10));
    out[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return out_len > 0;
}

bool parseByteArray(const String& raw, uint8_t* out, size_t& out_len) {
  out_len = 0;
  String token;
  for (size_t i = 0; i <= raw.length(); ++i) {
    const char c = (i < raw.length()) ? raw[i] : ',';
    if (c == ',' || c == ']') {
      token.trim();
      if (!token.isEmpty()) {
        char* end_ptr = nullptr;
        const unsigned long v = strtoul(token.c_str(), &end_ptr, 0);
        if (end_ptr != token.c_str() && out_len < kEmuDumpMaxBytes) {
          out[out_len++] = static_cast<uint8_t>(v & 0xFFu);
        }
      }
      token = "";
      continue;
    }
    if (c != '[' && c != ' ' && c != '\r' && c != '\n' && c != '\t') {
      token += c;
    }
  }
  return out_len > 0;
}

bool parseIso15BlocksArray(const String& raw, uint8_t* out, size_t& out_len) {
  out_len = 0;
  const int n = static_cast<int>(raw.length());
  int pos = 0;
  while (pos < n && raw[pos] != '[') ++pos;
  if (pos >= n) return false;
  ++pos;

  while (pos < n) {
    while (pos < n && (raw[pos] == ' ' || raw[pos] == '\r' || raw[pos] == '\n' || raw[pos] == '\t' || raw[pos] == ',')) ++pos;
    if (pos >= n || raw[pos] == ']') break;

    String token;
    if (raw[pos] == '"') {
      ++pos;
      const int start = pos;
      while (pos < n) {
        if (raw[pos] == '"' && raw[pos - 1] != '\\') break;
        ++pos;
      }
      token = raw.substring(static_cast<size_t>(start), static_cast<size_t>(pos));
      if (pos < n && raw[pos] == '"') ++pos;
    } else if (raw[pos] == '[') {
      int depth = 1;
      const int start = pos;
      ++pos;
      while (pos < n && depth > 0) {
        if (raw[pos] == '[') ++depth;
        else if (raw[pos] == ']') --depth;
        ++pos;
      }
      token = raw.substring(static_cast<size_t>(start), static_cast<size_t>(pos));
    } else {
      const int start = pos;
      while (pos < n && raw[pos] != ',' && raw[pos] != ']') ++pos;
      token = raw.substring(static_cast<size_t>(start), static_cast<size_t>(pos));
    }

    token.trim();
    if (token.isEmpty()) continue;

    uint8_t block[32] = {0};
    size_t block_len = 0;
    bool ok = false;
    if (token.startsWith("[")) {
      ok = parseByteArray(token, block, block_len);
    } else {
      ok = parseHexBytes(token, block, block_len);
    }

    if (!ok || block_len < 4) return false;
    if (out_len + 4 > kIso15DumpMaxBytes) break;
    memcpy(out + out_len, block, 4);
    out_len += 4;
  }

  return out_len > 0;
}

bool injectPm3Iso15UidHeader(const String& json, uint8_t* dump, size_t dump_len) {
  if (!dump || dump_len < 8) return false;

  String created_by;
  if (!extractJsonValue(json, "Created", created_by)) return false;
  created_by.toLowerCase();
  if (created_by.indexOf("proxmark") < 0) return false;

  String uid_hex;
  if (!extractJsonValue(json, "UID", uid_hex) && !extractJsonValue(json, "uid", uid_hex)) return false;
  uid_hex = normalizeHexText(uid_hex);
  if (uid_hex.length() < 16) return false;

  uint8_t uid[16] = {0};
  size_t uid_len = 0;
  if (!parseHexBytes(uid_hex, uid, uid_len) || uid_len < 8) return false;

  // ISO15693 emulation image stores UID in LSB-first order.
  for (size_t i = 0; i < 8; ++i) {
    dump[i] = uid[7 - i];
  }
  return true;
}

bool extractJsonValue(const String& src, const String& key, String& out) {
  const String key_tag = "\"" + key + "\"";
  const int key_pos = src.indexOf(key_tag);
  if (key_pos < 0) return false;

  int pos = src.indexOf(':', key_pos + key_tag.length());
  if (pos < 0) return false;
  ++pos;
  while (pos < static_cast<int>(src.length()) && (src[pos] == ' ' || src[pos] == '\r' || src[pos] == '\n' || src[pos] == '\t')) {
    ++pos;
  }
  if (pos >= static_cast<int>(src.length())) return false;

  if (src[pos] == '"') {
    ++pos;
    int end = pos;
    while (end < static_cast<int>(src.length())) {
      if (src[end] == '"' && src[end - 1] != '\\') break;
      ++end;
    }
    if (end >= static_cast<int>(src.length())) return false;
    out = src.substring(static_cast<size_t>(pos), static_cast<size_t>(end));
    return true;
  }

  if (src[pos] == '[') {
    int depth = 1;
    int end = pos + 1;
    while (end < static_cast<int>(src.length()) && depth > 0) {
      if (src[end] == '[') ++depth;
      else if (src[end] == ']') --depth;
      ++end;
    }
    if (depth != 0) return false;
    out = src.substring(static_cast<size_t>(pos), static_cast<size_t>(end));
    return true;
  }

  if (src[pos] == '{') {
    int depth = 1;
    int end = pos + 1;
    while (end < static_cast<int>(src.length()) && depth > 0) {
      if (src[end] == '{') ++depth;
      else if (src[end] == '}') --depth;
      ++end;
    }
    if (depth != 0) return false;
    out = src.substring(static_cast<size_t>(pos), static_cast<size_t>(end));
    return true;
  }

  int end = pos;
  while (end < static_cast<int>(src.length()) && src[end] != ',' && src[end] != '}' && src[end] != '\n') ++end;
  out = src.substring(static_cast<size_t>(pos), static_cast<size_t>(end));
  out.trim();
  return !out.isEmpty();
}

bool readDumpForTargetType(const String& path,
                           EmuType target_type,
                           uint8_t* dump,
                           size_t& dump_len,
                           bool& truncated_from_4k,
                           String& error_text) {
  dump_len = 0;
  truncated_from_4k = false;
  error_text = "Dump parse failed";

  File f = LittleFS.open(path, "r");
  if (!f || f.isDirectory()) {
    error_text = "Open dump failed";
    return false;
  }

  String lower = path;
  lower.toLowerCase();
  if (lower.endsWith(".bin")) {
    const size_t file_len = static_cast<size_t>(f.size());
    if (!dumpLengthMatchesType(target_type, file_len)) {
      if (target_type == EmuType::MF1K || target_type == EmuType::MF4K) {
        error_text = target_type == EmuType::MF1K ? "MFC1K needs 1024B" : "MFC4K needs 4096B";
      } else if (target_type == EmuType::ISO15) {
        error_text = "ISO15 bin needs 4B blocks";
      } else {
        error_text = "Dump/type mismatch";
      }
      return false;
    }

    const size_t size = min(file_len, static_cast<size_t>(kEmuDumpMaxBytes));
    dump_len = f.read(dump, size);
    return dump_len > 0;
  }

  String json;
  json.reserve(static_cast<size_t>(f.size()) + 1);
  while (f.available()) {
    json += static_cast<char>(f.read());
  }

  String type_hint;
  String file_type_hint;
  EmuType hinted_type;
  bool has_hinted_type = false;
  const bool has_file_type_hint = extractJsonValue(json, "FileType", file_type_hint);
  if (has_file_type_hint && mapPm3FileTypeToEmuType(file_type_hint, hinted_type)) {
    has_hinted_type = true;
  } else if ((extractJsonValue(json, "type", type_hint) ||
              extractJsonValue(json, "tagType", type_hint) ||
              extractJsonValue(json, "protocol", type_hint) ||
              extractJsonValue(json, "emuType", type_hint)) &&
             mapTypeHintToEmuType(type_hint, hinted_type)) {
    has_hinted_type = true;
  }

  if (has_hinted_type && hinted_type != target_type) {
    const bool both_mfc = (target_type == EmuType::MF1K || target_type == EmuType::MF4K) &&
                          (hinted_type == EmuType::MF1K || hinted_type == EmuType::MF4K);
    const bool target_is_ntag = target_type == EmuType::N213 || target_type == EmuType::N215 || target_type == EmuType::N216;
    const bool allow_mfu_family = has_file_type_hint && target_is_ntag && hinted_type == EmuType::N213 && isPm3MfuFamilyFileType(file_type_hint);
    if (!allow_mfu_family && !both_mfc) {
      error_text = "Dump/type mismatch";
      return false;
    }
  }

  String payload;
  if (extractJsonValue(json, "blocks", payload) && payload.startsWith("{")) {
    uint8_t block_size_hint = 0;
    switch (target_type) {
      case EmuType::MF1K:
      case EmuType::MF4K: block_size_hint = 16; break;
      case EmuType::N213:
      case EmuType::N215:
      case EmuType::N216:
      case EmuType::Felica:
      case EmuType::ISO15: block_size_hint = 4; break;
      default: block_size_hint = 0; break;
    }

    size_t parsed_blocks = 0;
    if (!parseJsonBlocksObject(payload, dump, kEmuDumpMaxBytes, block_size_hint, dump_len, parsed_blocks) || dump_len == 0) {
      error_text = "Dump parse failed";
      return false;
    }
    if (target_type == EmuType::ISO15) {
      (void)injectPm3Iso15UidHeader(json, dump, dump_len);
    }
  } else if (target_type == EmuType::ISO15 &&
             (extractJsonValue(json, "blocks", payload) ||
              extractJsonValue(json, "blockData", payload) ||
              extractJsonValue(json, "iso15693Blocks", payload))) {
    if (!parseIso15BlocksArray(payload, dump, dump_len)) {
      error_text = "ISO15 block parse fail";
      return false;
    }
  } else {
    if (!(extractJsonValue(json, "data", payload) ||
          extractJsonValue(json, "dump", payload) ||
          extractJsonValue(json, "bytes", payload) ||
          extractJsonValue(json, "hex", payload) ||
          extractJsonValue(json, "bin", payload))) {
      error_text = "Dump parse failed";
      return false;
    }

    if (payload.startsWith("[")) {
      if (!parseByteArray(payload, dump, dump_len)) {
        error_text = "Dump parse failed";
        return false;
      }
    } else {
      if (!parseHexBytes(payload, dump, dump_len)) {
        error_text = "Dump parse failed";
        return false;
      }
    }
  }

  if (!dumpLengthMatchesType(target_type, dump_len)) {
    if (target_type == EmuType::MF1K || target_type == EmuType::MF4K) {
      error_text = target_type == EmuType::MF1K ? "MFC1K needs 1024B" : "MFC4K needs 4096B";
    } else if (target_type == EmuType::ISO15) {
      error_text = "ISO15 dump needs 4B blocks";
    } else {
      error_text = "Dump/type mismatch";
    }
    return false;
  }

  return true;
}

bool applyDumpToCurrentType(const String& path, bool remember_selection, bool auto_restore, bool silent_fail) {
  const String prev_status = emu_dump_status;
  uint8_t dump[kEmuDumpMaxBytes] = {0};
  size_t dump_len = 0;
  bool truncated_from_4k = false;
  String parse_error;

  if (!littlefs_ready) {
    if (!silent_fail) emu_dump_status = "LittleFS unavailable";
    return false;
  }

  if (!readDumpForTargetType(path, emu_type, dump, dump_len, truncated_from_4k, parse_error)) {
    if (!silent_fail) emu_dump_status = parse_error;
    else emu_dump_status = prev_status;
    return false;
  }

  DumpTagType dump_type;
  if (!mapEmuTypeToDumpType(emu_type, dump_type)) {
    if (!silent_fail) emu_dump_status = "Type unsupported";
    else emu_dump_status = prev_status;
    return false;
  }

  if (false) {}  // MF1K supported on Unit NFC

  if (!nfc_ready) {
    if (!silent_fail) emu_dump_status = "NFC not ready";
    else emu_dump_status = prev_status;
    return false;
  }

  bool upload_ok = false;
  if (xSemaphoreTake(nfc_mutex, pdMS_TO_TICKS(800)) == pdTRUE) {
    upload_ok = nfc.uploadEmulationDump(dump_type, dump, dump_len);
    xSemaphoreGive(nfc_mutex);
  }
  if (!upload_ok) {
    if (!silent_fail) emu_dump_status = "Upload failed";
    else emu_dump_status = prev_status;
    return false;
  }

  const uint8_t type_idx = static_cast<uint8_t>(emu_type);
  emu_dump_restore_checked[type_idx] = true;
  emu_dump_loaded_path[type_idx] = path;
  emu_dump_loaded_uid[type_idx] = deriveEmuUidFromDump(emu_type, dump, dump_len);
  if (remember_selection) {
    saveLastDumpForType(emu_type, path);
  }

  if (auto_restore) {
    emu_dump_status = "Auto " + dumpFileName(path);
  } else if (truncated_from_4k) {
    emu_dump_status = "Loaded 4K->1K " + dumpFileName(path);
  } else {
    emu_dump_status = "Loaded " + dumpFileName(path);
  }

  return true;
}

bool loadDumpIntoEmulator(const String& path) {
  if (!applyDumpToCurrentType(path, true, false)) {
    return false;
  }
  m5paper_emu_hex_offset = 0;
  emu_switch_apply_pending = false;
  startCurrentEmulation();
  return emu_started;
}

uint16_t scaleColor565(uint16_t color, uint8_t scale) {
  uint8_t r = (color >> 11) & 0x1F;
  uint8_t g = (color >> 5) & 0x3F;
  uint8_t b = color & 0x1F;
  r = static_cast<uint8_t>((r * scale) / 255);
  g = static_cast<uint8_t>((g * scale) / 255);
  b = static_cast<uint8_t>((b * scale) / 255);
  return (r << 11) | (g << 5) | b;
}

uint16_t breathingColor(uint16_t base, uint32_t period_ms = 1700) {
  const uint32_t phase = millis() % period_ms;
  const uint32_t half = period_ms / 2;
  uint32_t ramp = 0;
  if (phase < half) {
    ramp = (phase * 255) / half;
  } else {
    ramp = ((period_ms - phase) * 255) / half;
  }
  const uint8_t scale = static_cast<uint8_t>(160 + (ramp * 95) / 255);
  return scaleColor565(base, scale);
}

void drawWrappedText(lgfx::v1::LGFXBase& d,
                     int x,
                     int y,
                     int width,
                     int line_height,
                     int max_lines,
                     int char_width,
                     const String& text,
                     uint16_t fg,
                     uint16_t bg) {
  if (width <= 6 || max_lines <= 0) return;

  const int max_chars = (width / char_width) - 1;
  if (max_chars <= 0) return;

  size_t index = 0;
  int line = 0;
  while (line < max_lines && index < text.length()) {
    if (text[index] == '\n') {
      ++index;
      ++line;
      continue;
    }

    const int newline_pos = text.indexOf('\n', index);
    const size_t hard_end = newline_pos >= 0 ? static_cast<size_t>(newline_pos) : text.length();

    size_t end = index + max_chars;
    if (end > hard_end) end = hard_end;

    size_t split = end;
    if (end < hard_end) {
      for (size_t i = end; i > index; --i) {
        const char c = text[i - 1];
        if (c == ' ' || c == '/' || c == ':' || c == '_' || c == '-' || c == ';' || c == ',') {
          split = i;
          break;
        }
      }
      if (split == index) split = end;
    }

    String part = text.substring(index, split);
    part.trim();

    if (line == max_lines - 1 && split < text.length()) {
      if (part.length() > static_cast<size_t>(max_chars - 2)) {
        part = part.substring(0, max_chars - 2);
      }
      part += "..";
    }

    d.setTextColor(fg, bg);
    d.setCursor(x, y + line * line_height);
    d.print(part);

    index = split;
    while (index < hard_end && text[index] == ' ') {
      ++index;
    }
    if (index >= hard_end && newline_pos >= 0) {
      index = hard_end + 1;
    }
    ++line;
  }
}

const char* dumpsMenuLabel(uint8_t idx) {
  if (dumps_pick_for_emu) {
    return idx == 0 ? "Back" : "";
  }

  switch (idx) {
    case 0: return "Emulate";
    case 1: return "Preview";
    case 2: return "Web QR";
    case 3: return "Exit";
    default: return "";
  }
}

uint8_t dumpsMenuNextEnabledIndex(uint8_t current, int8_t step_dir) {
  const uint8_t count = dumpsMenuCount();
  if (count == 0) return 0;
  if (step_dir == 0) step_dir = 1;
  const int8_t step = step_dir > 0 ? 1 : -1;

  uint8_t idx = current % count;
  for (uint8_t guard = 0; guard < count; ++guard) {
    idx = static_cast<uint8_t>((idx + count + step) % count);
    if (!dumpsMenuItemDisabled(idx)) return idx;
  }
  return current % count;
}

String dumpsPortalUrl() {
  IPAddress ip = WiFi.softAPIP();
  if (ip == IPAddress(0, 0, 0, 0)) ip = IPAddress(192, 168, 4, 1);
  return String("http://") + ip.toString() + "/";
}

String dumpsWifiQrPayload() {
  return String("WIFI:T:nopass;S:") + kEmuApSsid + ";;";
}

void prepareDumpsQrPayload(bool wifi_payload) {
  dumps_qr_wifi = wifi_payload;
  dumps_qr_payload = wifi_payload ? dumpsWifiQrPayload() : dumpsPortalUrl();
}

void drawDumpsPage(lgfx::v1::LGFXBase& d, int x, int y, int w, int h, uint16_t accent) {
  d.fillRect(x, y, w, h, TFT_BLACK);
  d.setFont(&fonts::Font0);
  d.setTextSize(1);

  if (dumps_pick_for_emu && dumps_stage != DumpsStage::Browse) {
    dumps_stage = DumpsStage::Browse;
  }

  if (dumps_stage == DumpsStage::PortalQr) {
    if (dumps_qr_payload.isEmpty()) prepareDumpsQrPayload(true);
    const int margin = 1;
    const int border = 2;
    const int qr_size = min(w - (margin * 2) - (border * 2), h - (margin * 2) - (border * 2));
    if (qr_size < 24) {
      d.setTextColor(TFT_WHITE, TFT_BLACK);
      d.setCursor(x + 2, y + 2);
      d.print("QR area too small");
      return;
    }
    const int qr_x = x + (w - qr_size) / 2;
    const int qr_y = y + margin;
    d.fillRect(x, y, w, h, TFT_BLACK);
    d.fillRect(qr_x - border, qr_y - border, qr_size + border * 2, qr_size + border * 2, TFT_WHITE);
    d.qrcode(dumps_qr_payload, qr_x, qr_y, qr_size, 0);
    return;
  }

  if (dumps_stage == DumpsStage::Preview) {
    d.fillRect(x, y, w, h, TFT_BLACK);

    if (dumps_preview_text.isEmpty() && emu_dump_count > 0 && dump_file_index < emu_dump_count) {
      dumps_preview_text = buildDumpPreview(emu_dump_files[dump_file_index]);
      dumps_preview_offset = 0;
    }

    const int text_x = x + 2;
    const int text_y = y + 1;
    const int text_w = max(12, w - 4);
    int line_h = 10;
    dumpsPreviewApplyFont(d, dumps_preview_font_level, line_h);
    const int footer_h = d.fontHeight() + 1;
    const int text_h = max(20, h - footer_h - 2);
    d.setTextWrap(false);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    const size_t page_lines = dumpsPreviewPageLines(text_h, line_h);
    const size_t total_lines = dumpsPreviewCountLines(dumps_preview_text);
    if (total_lines > 0 && dumps_preview_offset >= total_lines) dumps_preview_offset = 0;
    const size_t page_start = (total_lines == 0) ? 0 : dumps_preview_offset;

    String preview_type = "Unknown";
    if (emu_dump_count > 0 && dump_file_index < emu_dump_count) {
      preview_type = dumpTypeLabel(emu_dump_files[dump_file_index]);
    }

    size_t line_idx = 0;
    size_t pos = 0;
    while (line_idx < page_start && pos < dumps_preview_text.length()) {
      const int newline = dumps_preview_text.indexOf('\n', pos);
      if (newline < 0) {
        pos = dumps_preview_text.length();
      } else {
        pos = static_cast<size_t>(newline + 1);
      }
      ++line_idx;
    }

    for (size_t i = 0; i < page_lines && pos <= dumps_preview_text.length(); ++i) {
      size_t end = dumps_preview_text.length();
      const int newline = (pos < dumps_preview_text.length()) ? dumps_preview_text.indexOf('\n', pos) : -1;
      if (newline >= 0) end = static_cast<size_t>(newline);
      const String line = dumps_preview_text.substring(pos, end);
      drawDumpPreviewLine(d,
                          text_x,
                          text_y + static_cast<int>(i) * line_h,
                          line,
                          preview_type,
                          accent);
      if (newline < 0) break;
      pos = end + 1u;
    }

    const size_t page_index = (total_lines == 0) ? 1 : (page_start / page_lines) + 1;
    const size_t page_total = (total_lines == 0) ? 1 : ((total_lines + page_lines - 1) / page_lines);
    const String footer = String("A:Next Hold:Back B:Font");
    const String page_text = String(page_index) + "/" + String(page_total);
    d.setTextColor(accent, TFT_BLACK);
    d.setCursor(x + 2, y + h - d.fontHeight() - 1);
    d.print(footer);
    const int page_x = max(x + 2, x + w - 2 - d.textWidth(page_text));
    d.setCursor(page_x, y + h - d.fontHeight() - 1);
    d.print(page_text);
    d.setTextWrap(true);
    return;
  }

  d.setTextColor(TFT_WHITE, TFT_BLACK);

  if (emu_dump_count == 0 && !dumps_pick_for_emu) {
    d.setTextSize(2);
    drawWrappedText(d, x + 2, y + 12, w - 4, 18, max(2, (h - 12) / 18), 12, "No dumps. Connect GroveNFC-Dump and upload json/bin.", TFT_WHITE, TFT_BLACK);
    d.setTextSize(1);
    return;
  }

  d.setTextSize(2);
  const int row_h = 20;
  const int list_y = y + 12;
  const uint8_t total_items = dumpsBrowseCount();
  const int visible = max(1, min(static_cast<int>(total_items), (h - 12) / row_h));
  int first = 0;
  if (dump_file_index >= visible) first = dump_file_index - visible + 1;
  const int type_w = min(max(46, w / 4), 72);
  d.setTextWrap(false);
  for (int i = 0; i < visible; ++i) {
    const uint8_t idx = static_cast<uint8_t>(first + i);
    if (idx >= total_items) break;
    const int row_y = list_y + i * row_h;
    const bool selected = idx == dump_file_index;
    if (selected) {
      d.fillRect(x + 1, row_y, w - 2, row_h - 1, accent);
      d.setTextColor(TFT_BLACK, accent);
    } else {
      d.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    if (dumps_pick_for_emu && idx == 0) {
      d.setCursor(x + 4, row_y + 2);
      d.print("Back");
    } else {
      const uint8_t file_idx = dumps_pick_for_emu ? static_cast<uint8_t>(idx - 1) : idx;
      const int name_w = w - type_w - 12;
      String name = emu_dump_uid_view[file_idx];
      name.replace("\r", " ");
      name.replace("\n", " ");
      while (name.length() > 2 && d.textWidth(name) > name_w) {
        name = name.substring(0, name.length() - 1);
      }
      d.setCursor(x + 4, row_y + 2);
      d.print(name);

      String type = dumpTypeLabel(emu_dump_files[file_idx]);
      type.replace("\r", " ");
      type.replace("\n", " ");
      if (type.length() > 10) type = type.substring(0, 10);

      d.setTextSize(1);
      const int type_text_w = d.textWidth(type);
      const int tag_w = min(type_w - 2, max(26, type_text_w + 8));
      const int tag_x = x + w - tag_w - 2;
      const int tag_y = row_y + 3;
      const int tag_h = row_h - 6;
      const uint16_t type_bg = selected ? static_cast<uint16_t>(0xFFE0) : static_cast<uint16_t>(0xCFF9);
      d.fillRect(tag_x, tag_y, tag_w, tag_h, type_bg);
      d.drawRect(tag_x, tag_y, tag_w, tag_h, selected ? TFT_BLACK : TFT_DARKGREY);
      d.setTextColor(TFT_BLACK, type_bg);
      const int text_h = d.fontHeight();
      const int text_y = tag_y + max(0, (tag_h - text_h) / 2);
      d.setCursor(tag_x + 4, text_y);
      d.print(type);
      d.setTextSize(2);
    }
  }
  d.setTextWrap(true);
  d.setTextSize(1);

  if (dumps_stage == DumpsStage::Menu) {
    const uint8_t count = dumpsMenuCount();
    const int row_h = 22;
    const int box_w = min(w - 8, 156);
    const int box_h = count * row_h + 8;
    const int box_x = x + (w - box_w) / 2;
    const int box_y = y + (h - box_h) / 2;
    const int frame_radius = 10;
    d.fillRoundRect(box_x - 3, box_y - 3, box_w + 6, box_h + 6, frame_radius + 3, TFT_BLACK);  // shadow ring
    d.fillRoundRect(box_x, box_y, box_w, box_h, frame_radius, TFT_BLACK);
    d.drawRoundRect(box_x, box_y, box_w, box_h, frame_radius, accent);
    d.drawRoundRect(box_x + 1, box_y + 1, box_w - 2, box_h - 2, frame_radius - 1, accent);
    d.setTextSize(2);
    for (uint8_t i = 0; i < count; ++i) {
      const int row_y = box_y + 4 + i * row_h;
      const bool disabled = dumpsMenuItemDisabled(i);
      if (i == dumps_menu_index && !disabled) {
        d.fillRoundRect(box_x + 3, row_y, box_w - 6, row_h - 1, 5, accent);
        d.setTextColor(TFT_BLACK, accent);
      } else if (disabled) {
        d.setTextColor(0x6B6D, TFT_BLACK);
      } else {
        d.setTextColor(TFT_WHITE, TFT_BLACK);
      }
      d.setCursor(box_x + 8, row_y + 4);
      d.print(dumpsMenuLabel(i));
    }
    d.setTextSize(1);
  }

  if (dumps_emu_popup_active) {
    if (millis() > dumps_emu_popup_until_ms) {
      dumps_emu_popup_active = false;
    } else {
      const uint16_t header_bg = scaleColor565(accent, 210);
      const int frame_radius = 10;

      // Measure UID at size 2; fall back to size 1 for 7-byte UIDs.
      d.setFont(&fonts::Font0);
      String uid_line = dumps_emu_popup_line2;
      uid_line.replace("\r", " ");
      uid_line.replace("\n", " ");
      int uid_size = 2;
      d.setTextSize(uid_size);
      if (d.textWidth(uid_line) > w - 24) {
        uid_size = 1;
        d.setTextSize(uid_size);
      }
      const int uid_px_w = d.textWidth(uid_line);

      // Measure type label.
      d.setFont(&fonts::Font2);
      d.setTextSize(1);
      const int type_px_w = dumps_emu_popup_type.isEmpty() ? 0 : d.textWidth(dumps_emu_popup_type);

      const int box_w = min(w - 8, max(w * 2 / 3, max(uid_px_w, type_px_w) + 16));
      const int box_h = min(h - 8, 66);
      const int box_x = x + (w - box_w) / 2;
      const int box_y = y + (h - box_h) / 2;

      // 3px black shadow ring to separate from background content.
      d.fillRoundRect(box_x - 3, box_y - 3, box_w + 6, box_h + 6, frame_radius + 3, TFT_BLACK);
      d.fillRoundRect(box_x, box_y, box_w, box_h, frame_radius, TFT_BLACK);
      d.drawRoundRect(box_x, box_y, box_w, box_h, frame_radius, accent);
      d.drawRoundRect(box_x + 1, box_y + 1, box_w - 2, box_h - 2, frame_radius - 1, accent);
      d.fillRect(box_x + 2, box_y + 2, box_w - 4, 16, header_bg);
      d.fillRect(box_x + 2, box_y + 17, box_w - 4, 2, accent);

      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      d.setTextColor(TFT_BLACK, header_bg);
      d.setCursor(box_x + 6, box_y + 3);
      d.print(dumps_emu_popup_line1);

      d.setTextSize(uid_size);
      d.setTextColor(TFT_WHITE, TFT_BLACK);
      d.setCursor(box_x + 6, box_y + 23);
      d.print(uid_line);

      if (!dumps_emu_popup_type.isEmpty()) {
        d.setFont(&fonts::Font2);
        d.setTextSize(1);
        d.setTextColor(accent, TFT_BLACK);
        d.setCursor(box_x + 6, box_y + box_h - d.fontHeight() - 5);
        d.print(dumps_emu_popup_type);
      }

      d.setFont(&fonts::Font2);
      d.setTextSize(1);
    }
  }
}

void drawHomeIconReader(lgfx::v1::LGFXBase& d, int cx, int cy, uint16_t accent) {
  const int x = cx - 24;
  const int y = cy - 16;
  d.fillRect(x, y, 28, 24, accent);
  d.fillRect(x + 4, y + 4, 20, 12, TFT_BLACK);
  d.fillRect(x + 4, y + 18, 20, 3, accent);

  d.fillRect(x + 32, y + 9, 3, 6, accent);
  d.fillRect(x + 36, y + 6, 3, 12, accent);
  d.fillRect(x + 40, y + 3, 3, 18, accent);
}

void drawHomeIconEmulator(lgfx::v1::LGFXBase& d, int cx, int cy, uint16_t accent) {
  const int x = cx - 18;
  const int y = cy - 18;
  const int s = 36;

  d.drawRect(x, y, s, s, accent);
  d.drawRect(x + 1, y + 1, s - 2, s - 2, accent);

  d.fillRect(x + 4, y + 4, s - 8, 3, accent);
  d.fillRect(x + s - 7, y + 4, 3, s - 11, accent);
  d.fillRect(x + 10, y + s - 10, s - 17, 3, accent);
  d.fillRect(x + 10, y + 10, 3, s - 20, accent);
  d.fillRect(x + 10, y + 10, s - 20, 3, accent);
  d.fillRect(x + s - 13, y + 10, 3, s - 26, accent);
  d.fillRect(x + 16, y + s - 16, s - 29, 3, accent);
}

void drawHomeIconDumps(lgfx::v1::LGFXBase& d, int cx, int cy, uint16_t accent) {
  const int x = cx - 19;
  const int y = cy - 18;

  d.drawRect(x, y, 38, 36, accent);
  d.drawRect(x + 1, y + 1, 36, 34, accent);

  d.fillRect(x + 6, y + 6, 16, 3, accent);
  d.fillRect(x + 6, y + 6, 3, 12, accent);
  d.fillRect(x + 6, y + 16, 10, 3, accent);

  d.fillRect(x + 20, y + 17, 12, 3, accent);
  d.fillRect(x + 29, y + 17, 3, 12, accent);
  d.fillRect(x + 16, y + 27, 16, 3, accent);
}

void drawAboutPage(lgfx::v1::LGFXBase& d, int x, int y, int w, int h, uint16_t accent, int8_t page_idx) {
  d.fillRect(x, y, w, h, TFT_BLACK);
  d.setFont(&fonts::Font0);
  d.setTextSize(2);
  if (page_idx == 0) {
    const int line_h = 20;
    const int indent = 12;
    int ty = y + 4;
    d.setTextColor(accent, TFT_BLACK);
    d.setCursor(x + 2, ty); d.print("IIC Hardware"); ty += line_h;
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setCursor(x + 2 + indent, ty); d.print("M5 Unit NFC"); ty += line_h;
    d.setCursor(x + 2 + indent, ty); d.print("MT GroveNFC"); ty += line_h;
    ty += line_h / 2;
    d.setTextColor(accent, TFT_BLACK);
    d.setCursor(x + 2, ty); d.print("Github"); ty += line_h;
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setCursor(x + 2 + indent, ty); d.print("whywilson/GroveNFC"); ty += line_h;
  } else {
    const char* url = "github.com/whywilson/GroveNFC";
    const int qr_size = max(24, min(w, h));
    const int qr_x = x + (w - qr_size) / 2;
    const int qr_y = y + (h - qr_size) / 2;
    d.fillRect(x, y, w, h, TFT_BLACK);
    d.fillRect(qr_x, qr_y, qr_size, qr_size, TFT_WHITE);
    d.qrcode(url, qr_x, qr_y, qr_size, 0);
  }
}

String readerTypeLabel(const grove_nfc::CardInfo& card, bool only_14b) {
  if (card.valid) {
    if (card.protocol == "ISO15693" && !card.detail.isEmpty()) {
      String type = card.detail;
      const int separator = type.indexOf(" | ");
      if (separator >= 0) type = type.substring(0, separator);
      type.trim();
      if (!type.isEmpty()) return type;
    }
    return String(protocolFull(card.protocol));
  }
  return only_14b ? String("SCAN 14B") : String("SCANNING");
}

String readerIdLabel(const grove_nfc::CardInfo& card) {
  if (!card.valid) return "--";
  String id = formatIdText(card.uid);
  if (!id.isEmpty()) return id;
  return "--";
}

void drawReaderUnitName(lgfx::v1::LGFXBase& d,
                        int x,
                        int y,
                        int width,
                        int height,
                        uint16_t accent) {
  String unit_text = upperText(nfc_module_name);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextWrap(false);

  const int unit_w = d.textWidth(unit_text);
  const int unit_x = x + (width - unit_w) / 2;
  const int top_y = y + 3;
  const int bottom_y = y + height - d.fontHeight() - 3;

  d.setTextColor(scaleColor565(accent, 190), TFT_BLACK);
  d.setCursor(unit_x, top_y);
  d.print(unit_text);

  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  d.setCursor(unit_x, bottom_y);
  d.print(unit_text);
}

void drawReaderPixelCard(lgfx::v1::LGFXBase& d,
                         int x,
                         int y,
                         int width,
                         int height,
                         uint16_t accent,
                         const grove_nfc::CardInfo& card,
                         bool only_14b,
                         bool anim_only = false) {
  if (width < 40 || height < 24) return;

  d.startWrite();

  if (!anim_only) {
    d.fillRect(x, y, width, height, TFT_BLACK);
  }

  if (!card.valid) {
    // ---------- Curtain scan box ----------
    const int border = 3;
#ifdef APP_TARGET_ATOMS3
    const int card_w = width;                   // square screen
#else
    const int card_w = (width * 3) / 5;             // 3/5 screen width
#endif
    const int card_h = min(48, height - 6);
    const int card_x = x + (width - card_w) / 2;
    const int card_y = y + (height - card_h) / 2;

    const int inner_left = card_x + border;
    const int inner_top  = card_y + border;
    const int inner_w    = max(1, card_w - border * 2);
    const int inner_h    = max(1, card_h - border * 2);
    const int line_gap   = 3;                       // gap between line and border
    const int line_top   = inner_top + line_gap;
    const int line_h     = max(1, inner_h - line_gap * 2);
    const int travel     = max(1, inner_w - 2);

    static const char* kProtoLabels[] = {"ISO14443A", "ISO14443B", "ISO15693", "Felica"};
    static constexpr int kProtoCount = 4;
    static int proto_idx = 0;

    static float progress = 0.0f;
    static int phase = 0;           // 0=L→R, 1=R→L
    static bool initialized = false;
    static uint32_t last_step_us = 0;

    // Pre-rendered text sprite: rebuilt only when proto or dimensions change.
    // pushSprite with clip is a single DMA burst vs d.print() which fires per-char SPI transactions.
    static LGFX_Sprite txt_spr;
    static int txt_spr_w = 0;
    static int txt_spr_h = 0;
    static int txt_spr_proto = -1;
    static uint16_t txt_spr_color = 0;

    // Off-screen inner-area buffer: compose full frame here, then push once.
    // This eliminates the black-flash caused by fillRect + pushSprite being
    // two separate SPI transactions — display scans between them and shows black.
    static LGFX_Sprite inner_spr;
    static int inner_spr_w = 0;
    static int inner_spr_h = 0;

    if (!initialized) {
      progress = 0.0f;
      phase = 0;
      last_step_us = micros();
      initialized = true;
    } else if (anim_only) {
      // Fixed-step animator driven by micros() inside the draw function.
      // This avoids external gating edge-cases that can freeze progress at 0.
      const uint32_t now_us = micros();
      uint32_t elapsed = now_us - last_step_us;
      if (elapsed >= kReaderAnimStepUs) {
        uint32_t steps = elapsed / kReaderAnimStepUs;
        if (steps > 8) steps = 8;  // clamp catch-up to keep motion smooth
        last_step_us += steps * kReaderAnimStepUs;
        const float dt = static_cast<float>(kReaderAnimStepUs) / static_cast<float>(kReaderSweepUs);
        progress += dt * static_cast<float>(steps);
        while (progress >= 1.0f) {
          progress -= 1.0f;
          if (phase == 1) {
            proto_idx = (proto_idx + 1) % kProtoCount;
            phase = 0;
          } else {
            phase = 1;
          }
        }
      }
    } else {
      // On full refresh, sync step clock so next anim tick doesn't burst.
      last_step_us = micros();
    }

    // Stronger edge deceleration: mostly sinusoidal easing with enough linear term
    // to keep endpoint velocity non-zero after integer rounding.
    const float t = progress;
    const float ease_in_out = 0.5f - 0.5f * cosf(t * 3.1415926f);  // 0..1
    const float eased = 0.56f * t + 0.44f * ease_in_out;
    int scan_pos;
    if (phase == 0) {
      scan_pos = static_cast<int>(eased * travel + 0.5f);
    } else {
      scan_pos = travel - static_cast<int>(eased * travel + 0.5f);
    }
    if (scan_pos < 0) scan_pos = 0;
    if (scan_pos > travel) scan_pos = travel;
    const int scan_x = inner_left + scan_pos;

    const char* label = kProtoLabels[proto_idx];
    const uint16_t text_color = scaleColor565(accent, 180);

    // Rebuild text sprite if proto / dimensions / color changed
    if (txt_spr_proto != proto_idx || txt_spr_w != inner_w || txt_spr_h != inner_h || txt_spr_color != text_color) {
      if (txt_spr_w != inner_w || txt_spr_h != inner_h) {
        txt_spr.deleteSprite();
        txt_spr.setColorDepth(16);
        txt_spr.createSprite(inner_w, inner_h);
        txt_spr_w = inner_w;
        txt_spr_h = inner_h;
      }
      txt_spr.fillSprite(TFT_BLACK);
      txt_spr.setFont(&fonts::Font0);
      txt_spr.setTextSize(2);
      txt_spr.setTextWrap(false);
      txt_spr.setTextColor(text_color);
      int16_t tw2 = txt_spr.textWidth(label);
      int16_t th2 = txt_spr.fontHeight();
      txt_spr.setCursor((inner_w - tw2) / 2, (inner_h - th2) / 2);
      txt_spr.print(label);
      txt_spr_proto = proto_idx;
      txt_spr_color = text_color;
    }

    // Rebuild inner sprite if dimensions changed
    if (inner_spr_w != inner_w || inner_spr_h != inner_h) {
      inner_spr.deleteSprite();
      inner_spr.setColorDepth(16);
      inner_spr.createSprite(inner_w, inner_h);
      inner_spr_w = inner_w;
      inner_spr_h = inner_h;
    }

    // --- Draw border (only on full refresh) ---
    if (!anim_only) {
      const uint16_t c1 = scaleColor565(accent, 220);
      const uint16_t c2 = scaleColor565(accent, 170);
      const uint16_t c3 = scaleColor565(accent, 120);
      d.drawRect(card_x, card_y, card_w, card_h, c1);
      d.drawRect(card_x + 1, card_y + 1, card_w - 2, card_h - 2, c2);
      d.drawRect(card_x + 2, card_y + 2, card_w - 4, card_h - 4, c3);
      const int rc = 5;
      d.fillRect(card_x, card_y, rc, 1, TFT_BLACK);
      d.fillRect(card_x, card_y, 1, rc, TFT_BLACK);
      d.fillRect(card_x + 1, card_y + 1, 2, 1, TFT_BLACK);
      d.fillRect(card_x + 1, card_y + 1, 1, 2, TFT_BLACK);
      d.fillRect(card_x + card_w - rc, card_y, rc, 1, TFT_BLACK);
      d.fillRect(card_x + card_w - 1, card_y, 1, rc, TFT_BLACK);
      d.fillRect(card_x + card_w - 3, card_y + 1, 2, 1, TFT_BLACK);
      d.fillRect(card_x + card_w - 2, card_y + 1, 1, 2, TFT_BLACK);
      d.fillRect(card_x, card_y + card_h - 1, rc, 1, TFT_BLACK);
      d.fillRect(card_x, card_y + card_h - rc, 1, rc, TFT_BLACK);
      d.fillRect(card_x + 1, card_y + card_h - 2, 2, 1, TFT_BLACK);
      d.fillRect(card_x + 1, card_y + card_h - 3, 1, 2, TFT_BLACK);
      d.fillRect(card_x + card_w - rc, card_y + card_h - 1, rc, 1, TFT_BLACK);
      d.fillRect(card_x + card_w - 1, card_y + card_h - rc, 1, rc, TFT_BLACK);
      d.fillRect(card_x + card_w - 3, card_y + card_h - 2, 2, 1, TFT_BLACK);
      d.fillRect(card_x + card_w - 2, card_y + card_h - 3, 1, 2, TFT_BLACK);
    }

    // --- Compose full frame into inner_spr, then push once (single DMA transaction).
    // This eliminates the black-flash that occurs when fillRect and pushSprite are
    // two separate SPI writes with the display scanning between them.
    inner_spr.fillSprite(TFT_BLACK);
    // Curtain reveal: text visible to the LEFT of the scan line in both phases.
    //   phase 0 (L→R): text grows from left as line sweeps right
    //   phase 1 (R→L): line sweeps left, progressively covering text from right
    if (scan_pos > 0) {
      inner_spr.setClipRect(0, 0, scan_pos, inner_h);
      txt_spr.pushSprite(&inner_spr, 0, 0);
      inner_spr.clearClipRect();
    }
    // Scan line (coordinates relative to inner_spr origin)
    inner_spr.fillRect(scan_pos, line_gap, 2, line_h, TFT_WHITE);
    // Push the completed frame to display in one transaction — no intermediate state shown
    inner_spr.pushSprite(static_cast<lgfx::v1::LovyanGFX*>(&d), inner_left, inner_top);
    d.endWrite();
    return;
  }

  if (anim_only) { d.endWrite(); return; }


  String type_text = upperText(readerTypeLabel(card, only_14b));
  String id_text = marqueeText(readerIdLabel(card), 16, 180);

  d.setFont(&fonts::Font0);
  d.setTextSize(2);

  const int type_w = d.textWidth(type_text);
  const int type_x = x + (width - type_w) / 2;
  const int center_y = y + height / 2;
  const int type_y = center_y - 18;

  d.setTextColor(accent, TFT_BLACK);
  d.setCursor(type_x, type_y);
  d.print(type_text);

#ifndef APP_TARGET_ATOMS3
  // Left/right chunky arrows pointing inward (skip on small square screen)
  const int deco_gap = 6;
  const int deco_y = type_y + 1;
  const int left_deco_x = type_x - deco_gap - 12;
  const int right_deco_x = type_x + type_w + deco_gap;
  if (left_deco_x >= x + 1 && right_deco_x + 12 <= x + width - 1) {
    d.fillRect(left_deco_x + 1, deco_y + 2, 3, 10, accent);
    d.fillRect(left_deco_x + 4, deco_y + 4, 3, 6, accent);
    d.fillRect(left_deco_x + 7, deco_y + 6, 3, 2, accent);

    d.fillRect(right_deco_x + 8, deco_y + 2, 3, 10, accent);
    d.fillRect(right_deco_x + 5, deco_y + 4, 3, 6, accent);
    d.fillRect(right_deco_x + 2, deco_y + 6, 3, 2, accent);
  }
#endif

#ifdef APP_TARGET_ATOMS3
  // On AtomS3: wrap long ID text across multiple lines
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  {
    const int id_y = center_y + 4;
    const int max_w = width - 4;  // usable width with small margin
    const int font_h = d.fontHeight();
    // Split id_text into lines that fit
    String remaining = id_text;
    int cur_y = id_y;
    while (remaining.length() > 0 && cur_y + font_h <= y + height) {
      // Find how many chars fit in max_w
      int fit = remaining.length();
      for (int i = 1; i <= (int)remaining.length(); i++) {
        if (d.textWidth(remaining.substring(0, i)) > max_w) {
          fit = i - 1;
          break;
        }
      }
      if (fit <= 0) fit = 1;
      String line = remaining.substring(0, fit);
      remaining = remaining.substring(fit);
      int lw = d.textWidth(line);
      d.setCursor(x + (width - lw) / 2, cur_y);
      d.print(line);
      cur_y += font_h + 2;
    }
  }
#else
  const int id_w = d.textWidth(id_text);
  const int id_x = x + (width - id_w) / 2;
  const int id_y = center_y + 4;
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setCursor(id_x, id_y);
  d.print(id_text);
#endif

  d.endWrite();
}

void drawPianoKeyboard(lgfx::v1::LGFXBase& d,
                       int x,
                       int y,
                       int width,
                       int height,
                       int8_t active_note,
                       uint16_t accent) {
  if (width < 80 || height < 30) return;

  d.fillRect(x, y, width, height, TFT_BLACK);
  d.drawRect(x, y, width, height, accent);

  const int white_count = kPianoNoteCount;
  const int white_w = width / white_count;
  const int white_h = height - 2;
  const uint16_t active_color = scaleColor565(accent, 220);

  for (int i = 0; i < white_count; ++i) {
    const int key_x = x + i * white_w;
    const bool active = (active_note == i);
    d.fillRect(key_x + 1, y + 1, white_w - 1, white_h, active ? active_color : TFT_WHITE);
    d.drawRect(key_x, y, white_w, height, TFT_BLACK);
  }

  static const uint8_t black_after[] = {0, 1, 3, 4, 5};
  const int black_w = max(4, white_w / 2);
  const int black_h = (height * 58) / 100;

  for (uint8_t i = 0; i < sizeof(black_after); ++i) {
    const int left_idx = black_after[i];
    const int bx = x + (left_idx + 1) * white_w - black_w / 2;
    d.fillRect(bx, y + 1, black_w, black_h, TFT_BLACK);
    d.drawRect(bx, y + 1, black_w, black_h, TFT_DARKGREY);
  }
}

bool getPianoPlayLayout(int& note_x,
                        int& note_y,
                        int& note_w,
                        int& note_h,
                        int& key_x,
                        int& key_y,
                        int& key_w,
                        int& key_h) {
#ifdef APP_TARGET_M5PAPER
  auto& d = g_canvas;
#else
  auto& d = M5.Display;
#endif
  const int w = d.width();
  const int h = d.height();

#if defined(APP_TARGET_STICKS3) || defined(APP_TARGET_STICKCPLUS) || defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
  if (w > h) {
    const int header_h = 18;
    const int content_top = header_h + 2;
    const int content_h = h - content_top - 2;
    note_x = 4;
    note_y = content_top + 20;
    note_w = w - 8;
    note_h = 16;
    key_x = 4;
    key_y = content_top + 38;
    key_w = w - 8;
    key_h = max(18, content_h - 44);
    return true;
  }
#endif

  const int body_y = 54;
  note_x = 4;
  note_y = body_y;
  note_w = w - 8;
  note_h = 18;
  key_x = 4;
  key_y = body_y + 20;
  key_w = w - 8;
  key_h = h - (body_y + 24);
  return true;
}

void drawPianoBlackKeys(lgfx::v1::LGFXBase& d, int x, int y, int width, int height) {
  const int white_w = width / kPianoNoteCount;
  static const uint8_t black_after[] = {0, 1, 3, 4, 5};
  const int black_w = max(4, white_w / 2);
  const int black_h = (height * 58) / 100;

  for (uint8_t i = 0; i < sizeof(black_after); ++i) {
    const int left_idx = black_after[i];
    const int bx = x + (left_idx + 1) * white_w - black_w / 2;
    d.fillRect(bx, y + 1, black_w, black_h, TFT_BLACK);
    d.drawRect(bx, y + 1, black_w, black_h, TFT_DARKGREY);
  }
}

void drawPianoWhiteKey(lgfx::v1::LGFXBase& d,
                       int key_x,
                       int key_y,
                       int key_w,
                       int key_h,
                       int key_idx,
                       bool active,
                       uint16_t accent) {
  if (key_idx < 0 || key_idx >= static_cast<int>(kPianoNoteCount)) return;
  const int white_w = key_w / kPianoNoteCount;
  const int x = key_x + key_idx * white_w;
  const uint16_t active_color = scaleColor565(accent, 220);
  d.fillRect(x + 1, key_y + 1, white_w - 1, key_h - 2, active ? active_color : TFT_WHITE);
  d.drawRect(x, key_y, white_w, key_h, TFT_BLACK);
}

void drawPianoPlayDiff(int8_t old_note, int8_t new_note, bool update_note_text) {
  if (in_home || menu_page != MenuPage::Piano || piano_stage != PianoStage::Play) return;
#ifdef APP_TARGET_M5PAPER
  constexpr int partial_x = 24;
  constexpr int partial_y = 380;
  const int partial_w = g_canvas.width() - 48;
  constexpr int partial_h = 300;
  drawPianoKeyboard(g_canvas, partial_x, partial_y, partial_w, partial_h, new_note, TFT_BLACK);
  drawPianoKeyboard(M5.Display, partial_x, partial_y, partial_w, partial_h, new_note, TFT_BLACK);
  M5.Display.display(partial_x, partial_y, partial_w, partial_h);
  return;
#endif

  int note_x = 0, note_y = 0, note_w = 0, note_h = 0;
  int key_x = 0, key_y = 0, key_w = 0, key_h = 0;
  if (!getPianoPlayLayout(note_x, note_y, note_w, note_h, key_x, key_y, key_w, key_h)) return;

  if (key_w < 80 || key_h < 30) {
    drawPianoPlayPartial();
    return;
  }

  auto& d =
#ifdef APP_TARGET_M5PAPER
    g_canvas;
#else
    M5.Display;
#endif
  const uint16_t accent = TFT_MAGENTA;
  d.setFont(&fonts::Font0);
  d.setTextSize(2);

  if (update_note_text) {
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.fillRect(note_x, note_y, note_w, note_h, TFT_BLACK);
    d.setCursor(note_x, note_y);
    d.print("Note: " + piano_last_note);
  }

  if (old_note != new_note) {
    drawPianoWhiteKey(d, key_x, key_y, key_w, key_h, old_note, false, accent);
    drawPianoWhiteKey(d, key_x, key_y, key_w, key_h, new_note, true, accent);
    drawPianoBlackKeys(d, key_x, key_y, key_w, key_h);
  }
}

void drawPianoPlayPartial() {
  if (in_home || menu_page != MenuPage::Piano || piano_stage != PianoStage::Play) return;

  auto& d =
#ifdef APP_TARGET_M5PAPER
    g_canvas;
#else
    M5.Display;
#endif
  int note_x = 0, note_y = 0, note_w = 0, note_h = 0;
  int key_x = 0, key_y = 0, key_w = 0, key_h = 0;
  getPianoPlayLayout(note_x, note_y, note_w, note_h, key_x, key_y, key_w, key_h);
  const uint16_t accent = TFT_MAGENTA;

  d.setFont(&fonts::Font0);
  d.setTextSize(2);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.fillRect(note_x, note_y, note_w, note_h, TFT_BLACK);
  d.setCursor(note_x, note_y);
  d.print("Note: " + piano_last_note);
  drawPianoKeyboard(d, key_x, key_y, key_w, key_h, piano_active_note_idx, accent);
}

EmuType emuTypeWithOffset(EmuType base, int8_t offset) {
  const int count = static_cast<int>(EmuType::Count);
  const int step = (offset >= 0) ? 1 : -1;
  int steps = abs(static_cast<int>(offset));
  int idx = static_cast<int>(base);

  if (steps == 0) {
    if (isEmuTypeSupportedCurrentMode(static_cast<EmuType>(idx))) return static_cast<EmuType>(idx);
    steps = 1;
  }

  while (steps-- > 0) {
    int guard = 0;
    do {
      idx += step;
      while (idx < 0) idx += count;
      idx %= count;
      ++guard;
    } while (guard <= count && !isEmuTypeSupportedCurrentMode(static_cast<EmuType>(idx)));
  }

  const EmuType out = static_cast<EmuType>(idx);
  return isEmuTypeSupportedCurrentMode(out) ? out : base;
}

float emuTypeAnimProgress() {
  if (!emu_type_anim_active || emu_type_anim_duration_ms == 0) return 1.0f;
  const uint32_t now = millis();
  const uint32_t elapsed = now - emu_type_anim_start_ms;
  if (elapsed >= emu_type_anim_duration_ms) {
    emu_type_anim_active = false;
    return 1.0f;
  }
  return static_cast<float>(elapsed) / static_cast<float>(emu_type_anim_duration_ms);
}

float easeOutCubic(float t) {
  const float u = 1.0f - t;
  return 1.0f - (u * u * u);
}

int fitEmuCenterTextSize(lgfx::v1::LGFXBase& d, const String& text, int desired, int max_w) {
  (void)text;
  (void)max_w;
  return min(desired, 2);
}

int drawEmuCenterLabel(lgfx::v1::LGFXBase& d,
                       int center_x,
                       int center_y,
                       int avail_w,
                       const String& text,
                       int desired_size,
                       int x_offset,
                       uint16_t color) {
  const int size = fitEmuCenterTextSize(d, text, desired_size, avail_w);
  d.setFont(&fonts::Font0);
  d.setTextSize(size);
  const int tw = d.textWidth(text);
  const int th = d.fontHeight();
  const int tx = center_x - tw / 2 + x_offset;
  const int ty = center_y - th / 2 - 1;
  d.setTextColor(color, TFT_BLACK);
  d.setCursor(tx, ty);
  d.print(text);
  return tw;
}

void drawEmuBottomLine(lgfx::v1::LGFXBase& d,
                       int x,
                       int y,
                       int w,
                       const String& text,
                       uint16_t color) {
  d.setFont(&fonts::Font0);
  d.setTextSize(2);
  d.setTextColor(color, TFT_BLACK);

  const int pad = 2;
  const int clip_x = x + pad;
  const int clip_w = max(8, w - pad * 2);
  const int line_h = d.fontHeight();
  const int text_w = d.textWidth(text);
 
  if (text_w <= clip_w) {
    d.setCursor(x + (w - text_w) / 2, y);
    d.print(text);
    return;
  }
 
  // Split into 2 fixed lines
  const size_t half = text.length() / 2;
  const String line1 = text.substring(0, half);
  const String line2 = text.substring(half);
  const int w1 = d.textWidth(line1);
  const int w2 = d.textWidth(line2);
  d.setCursor(x + (w - w1) / 2, y);
  d.print(line1);
  d.setCursor(x + (w - w2) / 2, y + line_h + 1);
  d.print(line2);
}

void drawEmuTypeCarousel(lgfx::v1::LGFXBase& d, int x, int y, int w, int h, uint16_t accent) {
  if (w <= 0 || h <= 0) return;

  const int center_x = x + w / 2;
  const int center_y = y + h / 2;
  const int label_center_y = center_y - 4;
  const int avail_w = max(20, w - 16);

  const String left_hint = String(emuName(emuTypeWithOffset(emu_type, -1)));
  const String right_hint = String(emuName(emuTypeWithOffset(emu_type, +1)));
  d.setFont(&fonts::Font0);
  d.setTextSize(1);
  d.setTextColor(scaleColor565(accent, 130), TFT_BLACK);
  const int hint_y = max(y + 1, label_center_y - 33);
  const int left_x = x + 4;
  const int right_w = d.textWidth(right_hint);
  const int right_x = x + w - right_w - 4;
  d.setCursor(left_x, hint_y);
  d.print(left_hint);
  d.setCursor(right_x, hint_y);
  d.print(right_hint);

  int anchor_w = 0;
  if (emu_type_anim_active) {
    const float eased = easeOutCubic(emuTypeAnimProgress());
    const int travel = min(max(18, w / 4), 44);
    const uint8_t to_scale = static_cast<uint8_t>(min(255, 70 + static_cast<int>(eased * 185.0f)));

    const String to_text = String(emuName(emu_type_anim_to));

    const int to_offset = emu_type_anim_dir * static_cast<int>((1.0f - eased) * travel);

    anchor_w = drawEmuCenterLabel(d,
                                  center_x,
                                  label_center_y,
                                  avail_w,
                                  to_text,
                                  3,
                                  to_offset,
                                  scaleColor565(accent, to_scale));
  } else {
    const String cur_text = String(emuName(emu_type));
    const int desired = 3;
    anchor_w = drawEmuCenterLabel(d, center_x, label_center_y, avail_w, cur_text, desired, 0, accent);
  }

  (void)anchor_w;

  const int deco_y = label_center_y - 6;
  const int edge_pad = 2;
  const int left_deco_x = x + edge_pad;
  const int right_deco_x = x + w - edge_pad - 12;
  if (left_deco_x >= x + 1 && right_deco_x + 12 <= x + w - 1) {
    d.fillRect(left_deco_x + 1, deco_y + 6, 3, 2, accent);
    d.fillRect(left_deco_x + 4, deco_y + 4, 3, 6, accent);
    d.fillRect(left_deco_x + 7, deco_y + 2, 3, 10, accent);
    d.fillRect(right_deco_x + 8, deco_y + 6, 3, 2, accent);
    d.fillRect(right_deco_x + 5, deco_y + 4, 3, 6, accent);
    d.fillRect(right_deco_x + 2, deco_y + 2, 3, 10, accent);
  }
}

// ---------- Battery icon helper ----------
static void drawBatteryIcon(lgfx::v1::LGFXBase& d, int x, int y, int icon_w, int icon_h, uint16_t border_color) {
  const int32_t level = M5.Power.getBatteryLevel();
  if (level < 0) return;
  const int body_w = icon_w - 3;
  const int body_h = icon_h;
  const int nub_w = 2;
  const int nub_h = icon_h / 2;
  uint16_t color;
  if (level <= 10) {
    color = TFT_RED;
  } else if (level <= 20) {
    color = TFT_YELLOW;
  } else {
    color = TFT_GREEN;
  }
  d.fillRect(x + body_w, y + (icon_h - nub_h) / 2, nub_w, nub_h, border_color);
  d.drawRect(x, y, body_w, body_h, border_color);
  const int fill = (level * (body_w - 2)) / 100;
  if (fill > 0) {
    d.fillRect(x + 1, y + 1, fill, body_h - 2, color);
  }
}

#ifdef APP_TARGET_M5PAPER
static void drawM5PaperButton(int x, int y, int w, int h, const String& label, bool filled = false) {
  g_canvas.fillRoundRect(x, y, w, h, 12, filled ? TFT_BLACK : TFT_WHITE);
  g_canvas.drawRoundRect(x, y, w, h, 12, TFT_BLACK);
  g_canvas.setFont(&fonts::Font0);
  g_canvas.setTextSize(2);
  g_canvas.setTextColor(filled ? TFT_WHITE : TFT_BLACK, filled ? TFT_BLACK : TFT_WHITE);
  const int tw = g_canvas.textWidth(label);
  g_canvas.setCursor(x + (w - tw) / 2, y + (h - g_canvas.fontHeight()) / 2);
  g_canvas.print(label);
}

static void drawM5PaperPageTitle(const String& title, const String& subtitle) {
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setFont(&fonts::Font0);
  g_canvas.setTextSize(3);
  g_canvas.setCursor(24, kStatusBarHeight + 18);
  g_canvas.print(title);
  g_canvas.setTextSize(2);
  g_canvas.setCursor(26, kStatusBarHeight + 58);
  g_canvas.print(subtitle);
}

static void drawM5PaperPixelWrappedText(int x,
                                        int y,
                                        int width,
                                        int line_height,
                                        int max_lines,
                                        const String& text) {
  String remaining = text;
  remaining.trim();
  for (int line = 0; line < max_lines && !remaining.isEmpty(); ++line) {
    size_t fit = 0;
    size_t last_break = 0;
    for (size_t i = 1; i <= remaining.length(); ++i) {
      const String candidate = remaining.substring(0, i);
      if (g_canvas.textWidth(candidate) > width) break;
      fit = i;
      const char c = remaining[i - 1];
      if (c == ' ' || c == '/' || c == ',' || c == ';' || c == '\n') last_break = i;
    }
    if (fit == 0) fit = 1;
    size_t take = fit;
    if (fit < remaining.length() && last_break > 0) take = last_break;
    String part = remaining.substring(0, take);
    part.trim();
    if (line == max_lines - 1 && take < remaining.length()) {
      while (!part.isEmpty() && g_canvas.textWidth(part + "...") > width) {
        part.remove(part.length() - 1);
      }
      part += "...";
    }
    g_canvas.setCursor(x, y + line * line_height);
    g_canvas.print(part);
    remaining = remaining.substring(take);
    remaining.trim();
  }
}

static void drawM5PaperHomeIcon(int cx, int cy, MenuPage page, uint16_t color) {
  if (page == MenuPage::Reader) {
    g_canvas.drawRoundRect(cx - 28, cy - 22, 56, 42, 6, color);
    g_canvas.drawRoundRect(cx - 22, cy - 16, 44, 30, 4, color);
    g_canvas.drawArc(cx - 2, cy - 1, 7, 5, 300, 60, color);
    g_canvas.drawArc(cx - 2, cy - 1, 12, 10, 300, 60, color);
  } else if (page == MenuPage::Piano) {
    g_canvas.drawRoundRect(cx - 32, cy - 24, 64, 48, 5, color);
    for (int i = 1; i < 8; ++i) {
      const int x = cx - 32 + i * 8;
      g_canvas.drawLine(x, cy - 23, x, cy + 23, color);
    }
    static const uint8_t black_after[] = {1, 2, 4, 5, 6};
    for (uint8_t i : black_after) {
      const int x = cx - 32 + i * 8 - 2;
      g_canvas.fillRect(x, cy - 23, 5, 24, color);
    }
  } else {
    drawHomeIconForPage(g_canvas, cx, cy, page, color);
  }
}

static void drawM5PaperReaderPage() {
  const int w = g_canvas.width();
  g_canvas.fillScreen(TFT_WHITE);
  drawM5PaperPageTitle("CARD READER", "AUTO / ALL PROTOCOLS");

  const int card_x = 24, card_y = 170, card_w = w - 48, card_h = 570;
  g_canvas.drawRoundRect(card_x, card_y, card_w, card_h, 18, TFT_BLACK);
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setFont(&fonts::Font0);
  if (last_card.valid) {
    g_canvas.setTextSize(2);
    g_canvas.setCursor(card_x + 28, card_y + 38);
    g_canvas.print("CARD DETECTED");
    g_canvas.drawLine(card_x + 28, card_y + 78, card_x + card_w - 28, card_y + 78, TFT_BLACK);
    g_canvas.setTextSize(2);
    g_canvas.setCursor(card_x + 28, card_y + 118);
    g_canvas.print("PROTOCOL");
    g_canvas.setTextSize(3);
    g_canvas.setCursor(card_x + 28, card_y + 148);
    g_canvas.print(readerTypeLabel(last_card, false));
    g_canvas.setTextSize(2);
    g_canvas.setCursor(card_x + 28, card_y + 230);
    g_canvas.print("UID / IDENTIFIER");
    String uid = formatUidDisplay(last_card.uid);
    g_canvas.setTextSize(3);
    if (g_canvas.textWidth(uid) > card_w - 56) g_canvas.setTextSize(2);
    g_canvas.setCursor(card_x + 28, card_y + 270);
    g_canvas.print(uid);
    g_canvas.setTextSize(2);
    g_canvas.setCursor(card_x + 28, card_y + 430);
    g_canvas.print("DETAIL");
    drawM5PaperPixelWrappedText(card_x + 28, card_y + 470, card_w - 56, 34, 3, last_card.detail);
  } else {
    g_canvas.setTextSize(3);
    const String waiting = "READY TO SCAN";
    g_canvas.setCursor(card_x + (card_w - g_canvas.textWidth(waiting)) / 2, card_y + 180);
    g_canvas.print(waiting);
    g_canvas.setTextSize(2);
    const String hint = "Place a card near GroveNFC";
    g_canvas.setCursor(card_x + (card_w - g_canvas.textWidth(hint)) / 2, card_y + 250);
    g_canvas.print(hint);
    g_canvas.setTextSize(2);
    const String no_card = last_card.detail.isEmpty() ? String("No Card") : last_card.detail;
    const int no_card_w = g_canvas.textWidth(no_card);
    g_canvas.setCursor(card_x + (card_w - no_card_w) / 2, card_y + 330);
    g_canvas.print(no_card);
  }
  drawM5PaperButton(24, 820, w - 48, 88, "SCAN NOW", true);
}

void drawM5PaperEmulatorHexTable(lgfx::v1::LGFXBase& d) {
  constexpr int x = 24, y = 558, w = 492, h = 382;
  constexpr int header_h = 42, row_h = 33;
  const bool two_columns = emu_type == EmuType::N213 || emu_type == EmuType::N215 ||
                           emu_type == EmuType::N216 || emu_type == EmuType::ISO15;
  constexpr int rows_per_column = 10;
  const size_t page_items = two_columns ? rows_per_column * 2 : rows_per_column;
  const uint8_t type_idx = static_cast<uint8_t>(emu_type);
  const String path = type_idx < static_cast<uint8_t>(EmuType::Count)
                          ? emu_dump_loaded_path[type_idx]
                          : String();

  d.fillRoundRect(x, y, w, h, 18, TFT_WHITE);
  d.drawRoundRect(x, y, w, h, 18, TFT_BLACK);
  d.setFont(&fonts::Font0);
  d.setTextSize(2);
  d.setTextColor(TFT_BLACK, TFT_WHITE);
  String header_text = "DUMP HEX";
  if (!path.isEmpty()) {
    header_text = path.substring(path.lastIndexOf('/') + 1);
    const int title_max_w = w - 105;
    bool shortened = false;
    while (!header_text.isEmpty() && d.textWidth(header_text + "...") > title_max_w) {
      header_text.remove(header_text.length() - 1);
      shortened = true;
    }
    if (shortened) header_text += "...";
  }
  d.setCursor(x + 14, y + 11);
  d.print(header_text);

  if (path.isEmpty() || !littlefs_ready || !LittleFS.exists(path)) {
    d.setTextColor(TFT_BLACK, TFT_WHITE);
    d.setTextSize(3);
    const String built_in = "BUILT-IN DATA";
    d.setCursor(x + (w - d.textWidth(built_in)) / 2, y + 142);
    d.print(built_in);
    d.setTextSize(2);
    const String hint = "Load a dump to inspect its HEX data";
    d.setCursor(x + (w - d.textWidth(hint)) / 2, y + 194);
    d.print(hint);
    return;
  }

  const String preview = buildDumpPreview(path);
  size_t data_start = 0;
  for (int header = 0; header < 3 && data_start < preview.length(); ++header) {
    const int nl = preview.indexOf('\n', data_start);
    data_start = nl < 0 ? preview.length() : static_cast<size_t>(nl + 1);
  }
  size_t total = 0;
  for (size_t p = data_start; p < preview.length(); ++total) {
    const int nl = preview.indexOf('\n', p);
    p = nl < 0 ? preview.length() : static_cast<size_t>(nl + 1);
  }
  if (total == 0) m5paper_emu_hex_offset = 0;
  else if (m5paper_emu_hex_offset >= total) m5paper_emu_hex_offset = ((total - 1) / page_items) * page_items;

  size_t start = data_start;
  for (size_t skipped = 0; skipped < m5paper_emu_hex_offset && start < preview.length(); ++skipped) {
    const int nl = preview.indexOf('\n', start);
    start = nl < 0 ? preview.length() : static_cast<size_t>(nl + 1);
  }

  const size_t pages = total == 0 ? 1 : (total + page_items - 1) / page_items;
  const size_t page = total == 0 ? 1 : m5paper_emu_hex_offset / page_items + 1;
  const String page_text = String(page) + "/" + String(pages);
  d.setTextSize(2);
  d.setTextColor(TFT_BLACK, TFT_WHITE);
  d.setCursor(x + w - 14 - d.textWidth(page_text), y + 11);
  d.print(page_text);

  const int data_y = y + header_h + 5;
  const int col_w = (w - 20) / 2;
  for (size_t item = 0; item < page_items && start < preview.length(); ++item) {
    int nl = preview.indexOf('\n', start);
    if (nl < 0) nl = preview.length();
    String line = preview.substring(start, static_cast<size_t>(nl));
    start = static_cast<size_t>(nl + 1);
    const int colon = line.indexOf(':');
    if (colon < 0) continue;
    const unsigned index = static_cast<unsigned>(line.substring(0, colon).toInt());
    String hex = line.substring(colon + 1);
    const int ascii = hex.indexOf('|');
    String ascii_text;
    if (ascii >= 0) {
      ascii_text = hex.substring(ascii + 1);
      ascii_text.trim();
      hex = hex.substring(0, ascii);
    }
    hex.trim();
    char index_text[8]{};
    snprintf(index_text, sizeof(index_text), "%03u", index);
    const String display = two_columns
                               ? String(index_text) + ":" + hex + " " + ascii_text
                               : String(index_text) + ": " + hex;
    const int row = two_columns ? static_cast<int>(item % rows_per_column) : static_cast<int>(item);
    const int col = two_columns ? static_cast<int>(item / rows_per_column) : 0;
    const int rx = x + 10 + col * col_w;
    const int ry = data_y + row * row_h;
    const int rw = two_columns ? col_w : w - 20;
    if ((row & 1) == 0) d.fillRect(rx, ry - 3, rw, row_h, 0xDEFB);
    d.setTextColor(TFT_BLACK, (row & 1) == 0 ? 0xDEFB : TFT_WHITE);
    // A MIFARE block is 16 bytes: index + 32 HEX digits still fits the full
    // 492px single-column table at TextSize 2, so do not compress it to the
    // half-width-looking small font used previously.
    d.setTextSize(2);
    d.setCursor(rx + 7, ry + 3);
    d.print(display);
  }
  if (two_columns) d.drawLine(x + w / 2, y + header_h, x + w / 2, y + h - 5, TFT_LIGHTGREY);
}

void refreshM5PaperEmulatorHexTable() {
  constexpr int x = 24, y = 558, w = 492, h = 382;
  drawM5PaperEmulatorHexTable(g_canvas);
  drawM5PaperEmulatorHexTable(M5.Display);
  M5.Display.display(x, y, w, h);
}

static void drawM5PaperEmulatorPage() {
  const int w = g_canvas.width();
  g_canvas.fillScreen(TFT_WHITE);
  drawM5PaperPageTitle("CARD EMULATOR", "TOUCH A TYPE TO EMULATE");

  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setTextSize(2);
  g_canvas.setCursor(24, 146);
  g_canvas.print("TAP A CARD TYPE");
  constexpr int grid_x = 24, grid_y = 174, cell_w = 117, cell_h = 76, gap = 8;
  for (uint8_t i = 0; i < static_cast<uint8_t>(EmuType::Count); ++i) {
    const EmuType type = static_cast<EmuType>(i);
    const int col = i % 4, row = i / 4;
    const int x = grid_x + col * (cell_w + gap);
    const int y = grid_y + row * (cell_h + gap);
    const bool selected = type == emu_type;
    const bool supported = isEmuTypeSupportedCurrentMode(type);
    g_canvas.fillRoundRect(x, y, cell_w, cell_h, 10, selected ? TFT_BLACK : TFT_WHITE);
    g_canvas.drawRoundRect(x, y, cell_w, cell_h, 10, supported ? TFT_BLACK : 0xC618);
    g_canvas.setTextSize(2);
    g_canvas.setTextColor(selected ? TFT_WHITE : (supported ? TFT_BLACK : 0xC618),
                          selected ? TFT_BLACK : TFT_WHITE);
    const String label = emuName(type);
    g_canvas.setCursor(x + (cell_w - g_canvas.textWidth(label)) / 2, y + 27);
    g_canvas.print(label);
  }

  const int panel_x = 24, panel_y = 366, panel_w = w - 48, panel_h = 170;
  // The final grid item (ISO15) may be selected, leaving the canvas in
  // reverse text mode. Restore the normal card palette before drawing status
  // and UID so those labels never inherit the selected tile background.
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setFont(&fonts::Font0);
  g_canvas.fillRoundRect(panel_x, panel_y, panel_w, panel_h, 18, TFT_WHITE);
  g_canvas.drawRoundRect(panel_x, panel_y, panel_w, panel_h, 18, TFT_BLACK);
  g_canvas.setTextSize(2);
  g_canvas.setCursor(panel_x + 20, panel_y + 20);
  g_canvas.print("STATUS");
  g_canvas.setTextSize(3);
  g_canvas.setCursor(panel_x + 20, panel_y + 50);
  g_canvas.print(emu_started ? "EMULATING" : "STARTING...");
  drawM5PaperButton(354, 384, 144, 64, "LOAD DUMP");
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.drawLine(panel_x + 20, panel_y + 98, panel_x + panel_w - 20, panel_y + 98, TFT_BLACK);
  g_canvas.setTextSize(2);
  g_canvas.setCursor(panel_x + 20, panel_y + 112);
  g_canvas.print("ACTIVE UID");
  g_canvas.setTextSize(2);
  g_canvas.setCursor(panel_x + 160, panel_y + 112);
  g_canvas.print(ellipsizeCanvasText(formatUidDisplay(activeEmulatorDisplayId(emu_type)), panel_w - 200));

  drawM5PaperEmulatorHexTable(g_canvas);
  g_canvas.setTextSize(2);
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
}

static void drawM5PaperDumpsPage() {
  const int w = g_canvas.width();
  g_canvas.fillScreen(TFT_WHITE);
  drawM5PaperPageTitle(dumps_pick_for_emu ? "LOAD DUMP" : "DUMP LIBRARY",
                       dumps_pick_for_emu ? String("FILTER: ") + emuName(emu_type) : emu_dump_status);
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setTextSize(2);
  const String ap_line = String("AP: ") + kEmuApSsid + (emu_ap_active ? "  ON" : "  OFF");
  g_canvas.setCursor(w - g_canvas.textWidth(ap_line) - 22, 142);
  g_canvas.print(ap_line);
  const int list_y = 170, row_h = 76, visible = 7;
  int first = 0;
  if (dump_file_index >= visible) first = dump_file_index - visible + 1;
  if (emu_dump_count == 0) {
    g_canvas.setTextSize(2);
    g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    g_canvas.setCursor(36, 250);
    g_canvas.print("No compatible dump files");
    g_canvas.setTextSize(2);
    g_canvas.setCursor(36, 300);
    drawWrappedText(g_canvas, 36, 300, w - 72, 28, 3, 20,
                    "Use the Dump Library Wi-Fi portal to add files.", TFT_BLACK, TFT_WHITE);
  }
  for (int row = 0; row < visible; ++row) {
    const int idx = first + row;
    if (idx >= emu_dump_count) break;
    const int y = list_y + row * row_h;
    const bool selected = idx == dump_file_index;
    g_canvas.fillRoundRect(20, y, w - 40, row_h - 8, 10, selected ? TFT_BLACK : TFT_WHITE);
    g_canvas.drawRoundRect(20, y, w - 40, row_h - 8, 10, TFT_BLACK);
    g_canvas.setTextColor(selected ? TFT_WHITE : TFT_BLACK, selected ? TFT_BLACK : TFT_WHITE);
    g_canvas.setTextSize(2);
    g_canvas.setCursor(32, y + 12);
    g_canvas.print(emu_dump_uid_view[idx]);
    g_canvas.setTextSize(2);
    g_canvas.setCursor(32, y + 43);
    String file_line = dumpTypeLabel(emu_dump_files[idx]) + String("  ") + emu_dump_files[idx].substring(emu_dump_files[idx].lastIndexOf('/') + 1);
    g_canvas.print(ellipsizeCanvasText(file_line, w - 64));
  }
  g_canvas.setTextSize(2);
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setCursor(20, 870);
  g_canvas.print("Tap a dump for actions");

  if (m5paper_dump_actions && dump_file_index < emu_dump_count) {
    g_canvas.fillRoundRect(36, 480, w - 72, 440, 18, TFT_WHITE);
    g_canvas.drawRoundRect(36, 480, w - 72, 440, 18, TFT_BLACK);
    g_canvas.setTextSize(2);
    g_canvas.setCursor(60, 510);
    String selected_name = emu_dump_files[dump_file_index];
    selected_name = selected_name.substring(selected_name.lastIndexOf('/') + 1);
    g_canvas.print(selected_name);
    drawM5PaperButton(60, 560, 420, 76, "PREVIEW");
    drawM5PaperButton(60, 650, 420, 76, "EMULATE", true);
    drawM5PaperButton(60, 740, 420, 76, "DELETE");
    drawM5PaperButton(60, 830, 420, 76, "CANCEL");
  }
  if (m5paper_dump_delete_confirm) {
    g_canvas.fillRoundRect(44, 520, w - 88, 250, 18, TFT_WHITE);
    g_canvas.drawRoundRect(44, 520, w - 88, 250, 18, TFT_BLACK);
    g_canvas.setTextSize(3);
    g_canvas.setCursor(86, 555);
    g_canvas.print("DELETE DUMP?");
    g_canvas.setTextSize(2);
    g_canvas.setCursor(82, 595);
    g_canvas.print("This cannot be undone.");
    drawM5PaperButton(70, 620, 180, 86, "DELETE", true);
    drawM5PaperButton(290, 620, 180, 86, "CANCEL");
  }
}

static void drawM5PaperDumpPreviewPage() {
  const int w = g_canvas.width();
  g_canvas.fillScreen(TFT_WHITE);
  if (dump_file_index >= emu_dump_count) return;
  const String path = emu_dump_files[dump_file_index];
  String filename = path.substring(path.lastIndexOf('/') + 1);
  drawM5PaperPageTitle("DUMP PREVIEW", filename);

  String type_label, uid, sak, atqa, source_label;
  readDumpMetaForWeb(path, type_label, uid, sak, atqa, source_label);
  File f = LittleFS.open(path, "r");
  const size_t file_size = f ? static_cast<size_t>(f.size()) : 0;
  if (f) f.close();

  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setTextSize(2);
  g_canvas.drawRoundRect(20, 165, w - 40, 200, 14, TFT_BLACK);
  g_canvas.setCursor(38, 185);
  g_canvas.print("TYPE");
  g_canvas.setCursor(170, 185);
  g_canvas.print(type_label);
  g_canvas.setCursor(38, 220);
  g_canvas.print("UID");
  g_canvas.setCursor(170, 220);
  g_canvas.print(uid);
  g_canvas.setCursor(38, 255);
  g_canvas.print("ATQA / SAK");
  g_canvas.setCursor(210, 255);
  g_canvas.print(atqa + " / " + sak);
  g_canvas.setCursor(38, 290);
  g_canvas.print("SOURCE");
  g_canvas.setCursor(170, 290);
  g_canvas.print(source_label);
  g_canvas.setCursor(38, 325);
  g_canvas.print("FILE SIZE");
  g_canvas.setCursor(210, 325);
  g_canvas.print(String(file_size) + " bytes");

  constexpr int table_x = 20;
  constexpr int table_y = 390;
  constexpr int table_w = 500;
  constexpr int header_h = 42;
  constexpr int row_h = 27;
  constexpr int rows_per_page = 17;
  g_canvas.fillRect(table_x, table_y, table_w, header_h, TFT_BLACK);
  g_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  g_canvas.setTextSize(2);
  g_canvas.setCursor(table_x + 14, table_y + 11);
  g_canvas.print("INDEX");
  g_canvas.setCursor(table_x + 118, table_y + 11);
  g_canvas.print("HEX DATA");
  const bool show_ascii = type_label == "ISO15693" || type_label.startsWith("NTAG");
  if (show_ascii) {
    g_canvas.setCursor(table_x + 360, table_y + 11);
    g_canvas.print("ASCII");
  }

  size_t start = 0;
  int header_lines = 0;
  while (start < dumps_preview_text.length() && header_lines < 3) {
    const int nl = dumps_preview_text.indexOf('\n', start);
    if (nl < 0) break;
    start = static_cast<size_t>(nl + 1);
    ++header_lines;
  }
  for (size_t skipped = 0; skipped < dumps_preview_offset && start < dumps_preview_text.length(); ++skipped) {
    const int nl = dumps_preview_text.indexOf('\n', start);
    start = nl < 0 ? dumps_preview_text.length() : static_cast<size_t>(nl + 1);
  }

  g_canvas.setFont(&fonts::Font0);
  for (int row = 0; row < rows_per_page && start < dumps_preview_text.length(); ++row) {
    int nl = dumps_preview_text.indexOf('\n', start);
    if (nl < 0) nl = dumps_preview_text.length();
    String line = dumps_preview_text.substring(start, static_cast<size_t>(nl));
    start = static_cast<size_t>(nl + 1);
    const int colon = line.indexOf(':');
    if (colon < 0) continue;
    String index_text = line.substring(0, colon);
    String hex_text = line.substring(colon + 1);
    hex_text.trim();
    String ascii_text;
    const int ascii_sep = hex_text.indexOf("|");
    if (ascii_sep >= 0) {
      ascii_text = hex_text.substring(ascii_sep + 1);
      ascii_text.trim();
      hex_text = hex_text.substring(0, ascii_sep);
      hex_text.trim();
    }
    const int y = table_y + header_h + row * row_h;
    if ((row & 1) == 0) g_canvas.fillRect(table_x, y, table_w, row_h, 0xDEFB);
    g_canvas.setTextColor(TFT_BLACK, (row & 1) == 0 ? 0xDEFB : TFT_WHITE);
    g_canvas.setTextSize(2);
    char index_buf[8];
    snprintf(index_buf, sizeof(index_buf), "%04u", static_cast<unsigned>(index_text.toInt()));
    g_canvas.setCursor(table_x + 14, y + 5);
    g_canvas.print(index_buf);
    g_canvas.setCursor(table_x + 112, y + 5);
    g_canvas.print(hex_text);
    if (show_ascii) {
      g_canvas.setCursor(table_x + 360, y + 5);
      g_canvas.print(ascii_text);
    }
    g_canvas.drawLine(table_x, y + row_h - 1, table_x + table_w, y + row_h - 1, TFT_LIGHTGREY);
  }
  g_canvas.drawRect(table_x, table_y, table_w, header_h + rows_per_page * row_h, TFT_BLACK);
  g_canvas.drawLine(table_x + 96, table_y, table_x + 96, table_y + header_h + rows_per_page * row_h, TFT_BLACK);
  if (show_ascii) {
    g_canvas.drawLine(table_x + 348, table_y, table_x + 348, table_y + header_h + rows_per_page * row_h, TFT_BLACK);
  }
  g_canvas.setTextSize(1);
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setCursor(20, 915);
  g_canvas.print("Tap table for next page");
}

static void drawM5PaperPianoPage() {
  const int w = g_canvas.width();
  g_canvas.fillScreen(TFT_WHITE);
  drawM5PaperPageTitle("NFC PIANO", String("MAPPED CARDS: ") + pianoMappedCount() + "/8");
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  if (piano_stage == PianoStage::Menu) {
    drawM5PaperButton(24, 260, w - 48, 110, "PLAY", true);
    drawM5PaperButton(24, 400, w - 48, 110, "CONFIGURE 8 CARDS");
    g_canvas.setTextSize(2);
    drawWrappedText(g_canvas, 36, 570, w - 72, 32, 5, 20,
                    "Map eight NFC cards to musical notes, then tap PLAY and present a card to sound its note.",
                    TFT_BLACK, TFT_WHITE);
  } else {
    g_canvas.setTextSize(3);
    const String note = piano_stage == PianoStage::Config ? String(kPianoNoteName[piano_config_step]) : piano_last_note;
    const int note_w = g_canvas.textWidth(note);
    g_canvas.setCursor((w - note_w) / 2, 210);
    g_canvas.print(note);
    g_canvas.setTextSize(2);
    const int status_w = g_canvas.textWidth(piano_status);
    g_canvas.setCursor(max(20, (w - status_w) / 2), 270);
    g_canvas.print(piano_status);
    drawPianoKeyboard(g_canvas, 24, 380, w - 48, 300, piano_active_note_idx, TFT_BLACK);
    g_canvas.setTextSize(2);
    drawWrappedText(g_canvas, 32, 730, w - 64, 32, 4, 20,
                    piano_stage == PianoStage::Config
                        ? "Present one card for each requested note. Configuration saves after all eight cards."
                        : "Present a configured card to play. Tap Back to leave piano mode.",
                    TFT_BLACK, TFT_WHITE);
  }
}

static void drawM5PaperAboutPage() {
  const int w = g_canvas.width();
  g_canvas.fillScreen(TFT_WHITE);
  drawM5PaperPageTitle("ABOUT", "GROVENFC TOUCH REFERENCE DEVICE");
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setTextSize(2);
  g_canvas.setCursor(28, 190);
  g_canvas.print("DEVICE");
  drawM5PaperPixelWrappedText(28, 228, w - 56, 32, 5,
                              "M5Paper 540x960 e-ink touch controller with GroveNFC on I2C GPIO25/GPIO32. Reader, emulator and dump library run locally.");
  g_canvas.setCursor(28, 405);
  g_canvas.print("SUPPORTED READING");
  drawM5PaperPixelWrappedText(28, 443, w - 56, 32, 5,
                              "ISO14443A, ISO14443B, ISO15693 and FeliCa. Detects MIFARE Classic, Ultralight, NTAG and DESFire families when available.");
  g_canvas.setCursor(28, 620);
  g_canvas.print("SUPPORTED EMULATION");
  drawM5PaperPixelWrappedText(28, 658, w - 56, 32, 5,
                              "MIFARE Classic 1K/4K, NTAG213, NTAG215, NTAG216, ISO14443B, FeliCa and ISO15693. JSON/BIN dumps can be loaded from LittleFS.");
  g_canvas.setTextSize(2);
  g_canvas.setCursor(28, 870);
  g_canvas.printf("Module: %s  %s", nfc_module_name.c_str(), nfc_port_name.c_str());
  g_canvas.setCursor(28, 896);
  g_canvas.printf("HW:%04X  FW:%04X", hw_ver, fw_ver);
}
#endif
 
void drawScreen(bool popup_only) {
  auto& d = g_canvas;  // draw to off-screen sprite; push to display at every exit
  if (!popup_only) {
    d.fillScreen(TFT_BLACK);
  }
  d.setFont(&fonts::Font0);
  d.setTextSize(1);

  const int w = d.width();
  const int h = d.height();

  uint16_t accent = TFT_GREEN;
  if (menu_page == MenuPage::ReadNDEF) accent = TFT_CYAN;
  if (menu_page == MenuPage::Emulator) accent = TFT_ORANGE;
  if (menu_page == MenuPage::WebFiles) accent = TFT_CYAN;
  if (menu_page == MenuPage::Diagnose) accent = TFT_YELLOW;
  if (menu_page == MenuPage::Piano) accent = TFT_MAGENTA;
  if (menu_page == MenuPage::About) accent = TFT_CYAN;
  // Unit NFC and GroveNFC share the same color mapping.

#ifdef APP_TARGET_M5PAPER
  if (!in_home && menu_page == MenuPage::Reader) {
    drawM5PaperReaderPage();
    pushCanvasToEink();
    return;
  }
  if (!in_home && menu_page == MenuPage::Emulator) {
    drawM5PaperEmulatorPage();
    pushCanvasToEink();
    return;
  }
  if (!in_home && menu_page == MenuPage::WebFiles) {
    if (dumps_stage == DumpsStage::Preview) drawM5PaperDumpPreviewPage();
    else drawM5PaperDumpsPage();
    pushCanvasToEink();
    return;
  }
  if (!in_home && menu_page == MenuPage::Piano) {
    drawM5PaperPianoPage();
    pushCanvasToEink();
    return;
  }
  if (!in_home && menu_page == MenuPage::About) {
    drawM5PaperAboutPage();
    pushCanvasToEink();
    return;
  }
#endif

#if defined(APP_TARGET_STICKS3) || defined(APP_TARGET_STICKCPLUS)
  if (w > h) {
    const int header_h = 18;
    const int content_top = header_h + 2;
    const int content_h = h - content_top - 2;

    auto drawHeaderBar = [&](const String& title, bool show_back) {
      d.fillRect(0, 0, w, header_h, TFT_BLACK);
      d.drawLine(0, header_h, w, header_h, accent);
      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      d.setTextColor(accent, TFT_BLACK);
      if (show_back) {
        d.setCursor(2, 1);
        d.print("<");
      }
     const int tw = d.textWidth(title);
     d.setCursor((w - tw) / 2, 2);
     d.print(title);
      // Battery icon on Reader/Emulator pages
      if (menu_page == MenuPage::Reader || menu_page == MenuPage::Emulator) {
        drawBatteryIcon(d, w - 22, 4, 18, 10, accent);
      }
    };

    auto drawMenuBox = [&](int box_x, int box_y, int box_w, int box_h) {
      d.fillRect(box_x, box_y, box_w, box_h, TFT_BLACK);
      d.drawRect(box_x - 2, box_y - 2, box_w + 4, box_h + 4, TFT_BLACK);
      d.drawRect(box_x - 1, box_y - 1, box_w + 2, box_h + 2, TFT_BLACK);
      d.drawRect(box_x, box_y, box_w, box_h, accent);
      d.drawRect(box_x + 1, box_y + 1, box_w - 2, box_h - 2, accent);
      d.fillRect(box_x, box_y, 3, 3, TFT_BLACK);
      d.fillRect(box_x + box_w - 3, box_y, 3, 3, TFT_BLACK);
      d.fillRect(box_x, box_y + box_h - 3, 3, 3, TFT_BLACK);
      d.fillRect(box_x + box_w - 3, box_y + box_h - 3, 3, 3, TFT_BLACK);
      d.fillRect(box_x + 2, box_y + 1, 2, 2, accent);
      d.fillRect(box_x + box_w - 4, box_y + 1, 2, 2, accent);
      d.fillRect(box_x + 2, box_y + box_h - 3, 2, 2, accent);
      d.fillRect(box_x + box_w - 4, box_y + box_h - 3, 2, 2, accent);
    };

    if (popup_only) {
      if (!(menu_page == MenuPage::Emulator && emu_config_stage == EmuConfigStage::TypeMenu)) {
        return;
      }

      const int list_count = static_cast<int>(dumpMenuCount());
      const int visible_count = min(static_cast<int>(kEmuMenuVisibleCount), max(1, list_count));
      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      bool compact_font = false;
      int max_text_w = 0;
      for (int i = 0; i < list_count; ++i) {
        const int tw = d.textWidth(typeMenuLabel(static_cast<uint8_t>(i)));
        if (tw > max_text_w) max_text_w = tw;
      }
      const int box_w = min(w - 10, max(112, max_text_w + 22));
      const int row_w = box_w - 8;
      if (max_text_w + 8 > row_w) {
        compact_font = true;
        d.setFont(&fonts::Font2);
        d.setTextSize(1);
      }
      const int text_h = d.fontHeight();
      const int row_h = max(compact_font ? 14 : 18, text_h + 6);
      const int box_h = visible_count * row_h + 8;
      const int box_x = (w - box_w) / 2;
      const int box_y = content_top + (content_h - box_h) / 2;
      drawMenuBox(box_x, box_y, box_w, box_h);

      const int max_scroll = max(0, list_count - visible_count);
      const int scroll_start = min(static_cast<int>(emu_type_scroll), max_scroll);
      const int selected_row = min(static_cast<int>(emu_type_cursor), visible_count - 1);
      const int row_x = box_x + 4;

      for (int i = 0; i < visible_count; ++i) {
        const int idx = scroll_start + i;
        if (idx >= list_count) break;
        const bool selected = (i == selected_row);
        const int row_y = box_y + 4 + i * row_h;
        if (selected) {
          d.fillRect(row_x, row_y, row_w, row_h - 1, accent);
          d.setTextColor(TFT_BLACK, accent);
        } else {
          d.setTextColor(TFT_WHITE, TFT_BLACK);
        }
        const int text_y = row_y + max(0, ((row_h - 1) - text_h) / 2);
        d.setCursor(row_x + 6, text_y);
        d.print(typeMenuLabel(static_cast<uint8_t>(idx)));
      }
#ifdef APP_TARGET_M5PAPER
      pushCanvasToEink();
#else
      g_canvas.pushSprite(&M5.Display, 0, 0);
#endif
      return;
    }

    if (in_home) {
      d.fillScreen(TFT_BLACK);
      d.setTextColor(breathingColor(accent), TFT_BLACK);
      d.setTextSize(2);
      d.setFont(&fonts::Font0);
      String brand_s3 = nfc_module_name;
      const int brand_w = d.textWidth(brand_s3);
      d.setCursor((w - brand_w) / 2, 4);
      d.print(brand_s3);

      const int icon_cy = content_top + content_h / 2;

      const int icon_box_cx = w / 2;
      const int icon_box_w = 60;
      const int icon_box_h = 44;
      const int icon_box_x = icon_box_cx - icon_box_w / 2;
      const int icon_box_y = icon_cy - icon_box_h / 2;

      // Arrows with flash feedback
      auto drawArrow = [&](bool left, uint16_t col) {
        if (left) {
          d.fillRect(8, icon_cy - 2, 4, 4, col);
          d.fillRect(12, icon_cy - 6, 4, 12, col);
          d.fillRect(16, icon_cy - 10, 4, 20, col);
        } else {
          d.fillRect(w - 12, icon_cy - 2, 4, 4, col);
          d.fillRect(w - 16, icon_cy - 6, 4, 12, col);
          d.fillRect(w - 20, icon_cy - 10, 4, 20, col);
        }
      };
      const uint32_t now_ms = millis();
      drawArrow(true,  accent);
      drawArrow(false, accent);
      const bool flash_right_ok = home_arrow_flash_right && now_ms < home_arrow_flash_until_ms;
      const bool flash_left_ok  = home_arrow_flash_left  && now_ms < home_arrow_flash_until_ms;
      if (flash_right_ok) {
        drawArrow(false, TFT_BLACK);  // hide right arrow
      } else if (flash_left_ok) {
        drawArrow(true,  TFT_BLACK);  // hide left arrow
      } else {
        home_arrow_flash_right = false;
        home_arrow_flash_left = false;
      }

      if (home_anim_active) {
        const uint32_t now = millis();
        const uint32_t elapsed = now - home_anim_start_ms;
        float t = 1.0f;
        if (elapsed < home_anim_duration_ms) {
          t = static_cast<float>(elapsed) / static_cast<float>(home_anim_duration_ms);
        } else {
          home_anim_active = false;
        }
        const float eased = 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t);
        const float travel = 1.0f - eased; // 1→0, how far from center

        // ---- Icon: slide + alpha fade ----
        // Next (dir=1): from (current) slides LEFT out; to (new) slides in from RIGHT
        // Prev (dir=-1): from slides RIGHT out; to slides in from LEFT
        // Alpha curves differ: from fades out faster, to fades in with delay
        // so they don't appear simultaneously at mid-animation.
        const int travel_px = static_cast<int>(travel * icon_box_cx);
        const int from_cx = icon_box_cx - static_cast<int>(eased * icon_box_cx) * home_anim_dir;
        const int to_cx   = icon_box_cx + travel_px * home_anim_dir;

        d.fillRect(0, icon_box_y, w, icon_box_h, TFT_BLACK);

        // "From" icon: slides away, fades out (fast — gone by eased=0.5)
        {
          const uint8_t a = static_cast<uint8_t>(max(0, 255 - static_cast<int>(min(1.0f, eased * 2.0f) * 255.0f)));
          if (a > 4) {
            drawHomeIconForPage(d, from_cx, icon_cy, home_anim_from, scaleColor565(accent, a));
          }
        }

        // "To" icon: slides in, fades in (early — starts at eased=0.10)
        {
          const float reveal = max(0.0f, min(1.0f, (eased - 0.10f) / 0.90f));
          const uint8_t a = static_cast<uint8_t>(max(0, static_cast<int>(reveal * 255.0f)));
          if (a > 4) {
            drawHomeIconForPage(d, to_cx, icon_cy, home_anim_to, scaleColor565(accent, a));
          }
        }

        // ---- Text: staggered slide + alpha fade ----
        // Text slides same speed as icon but fades in much later,
        // creating a "follow behind" feel without a position jump.
        const float text_eased = max(0.0f, min(1.0f, (eased - 0.10f) / 0.90f));
        const float text_travel = 1.0f - text_eased;

        const int from_tx = icon_box_cx - static_cast<int>(eased * icon_box_cx) * home_anim_dir;
        const int to_tx   = icon_box_cx + static_cast<int>(text_travel * icon_box_cx) * home_anim_dir;

        const String name_from = String(homePageName(home_anim_from));
        const String name_to = String(homePageName(home_anim_to));
        d.setFont(&fonts::Font0);
        d.setTextSize(2);
        const int name_y = h - 20;

        // Show only one label at a time. The old label owns the first half of
        // the slide and the new label owns the second half, preventing two
        // identical names from being visible during a wrap-around transition.
        {
          const uint8_t a = static_cast<uint8_t>(max(0, 255 - static_cast<int>(min(1.0f, eased * 2.0f) * 255.0f)));
          if (eased < 0.5f && a > 4) {
            const int nw = d.textWidth(name_from);
            d.setTextColor(scaleColor565(accent, a), TFT_BLACK);
            d.setCursor(from_tx - nw / 2, name_y);
            d.print(name_from);
          }
        }

        // "To" text: slide + fade in during the second half only.
        {
          const float reveal_t = max(0.0f, min(1.0f, (eased - 0.5f) * 2.0f));
          const uint8_t a = static_cast<uint8_t>(max(0, static_cast<int>(reveal_t * 255.0f)));
          if (eased >= 0.5f && a > 4) {
            const int nw = d.textWidth(name_to);
            d.setTextColor(scaleColor565(accent, a), TFT_BLACK);
            d.setCursor(to_tx - nw / 2, name_y);
            d.print(name_to);
          }
        }
      } else {
        // Static home page — no animation
        d.fillRect(icon_box_x, icon_box_y, icon_box_w, icon_box_h, TFT_BLACK);
        drawHomeIconForPage(d, icon_box_cx, icon_cy, menu_page, accent);

        d.setTextColor(accent, TFT_BLACK);
        const String& home_name = homePageName(menu_page);
        d.setTextSize(2);
        const int hw = d.textWidth(home_name);
        d.setCursor((w - hw) / 2, h - 20);
        d.print(home_name);
      }

      if (!boot_notice_line.isEmpty()) {
        d.setTextSize(1);
        d.setTextColor(TFT_RED, TFT_BLACK);
        d.setCursor(4, h - 24);
        d.print(boot_notice_line);
      }
#ifdef APP_TARGET_M5PAPER
      pushCanvasToEink();
#else
      g_canvas.pushSprite(&M5.Display, 0, 0);
#endif
      return;
    }

    String page_title;
    if (menu_page == MenuPage::Reader) page_title = "Reader";
    else if (menu_page == MenuPage::ReadNDEF) page_title = "Read NDEF";
    else if (menu_page == MenuPage::Emulator) page_title = "Emulator";
    else if (menu_page == MenuPage::WebFiles) page_title = "Dumps";
    else if (menu_page == MenuPage::Piano) page_title = "Piano";
    else if (menu_page == MenuPage::About) page_title = "About";
    else page_title = "Diagnose";
    drawHeaderBar(page_title, true);
    if (menu_page == MenuPage::WebFiles && emu_dump_count > 0 && !dumps_pick_for_emu) {
      const String counter = String(dump_file_index + 1) + "/" + String(emu_dump_count);
      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      d.setTextColor(accent, TFT_BLACK);
      const int cw = d.textWidth(counter);
      d.setCursor(w - cw - 4, 2);
      d.print(counter);
    }

    d.fillRect(0, content_top, w, content_h, TFT_BLACK);
    d.setTextSize(2);
    d.setFont(&fonts::Font0);

    if (menu_page == MenuPage::Emulator && emu_config_stage == EmuConfigStage::TypeMenu) {
      const int list_count = static_cast<int>(dumpMenuCount());
      const int visible_count = min(static_cast<int>(kEmuMenuVisibleCount), max(1, list_count));
      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      bool compact_font = false;
      int max_text_w = 0;
      for (int i = 0; i < list_count; ++i) {
        const int tw = d.textWidth(typeMenuLabel(static_cast<uint8_t>(i)));
        if (tw > max_text_w) max_text_w = tw;
      }
      int box_w = min(w - 10, max(112, max_text_w + 22));
      int row_w = box_w - 8;
      if (max_text_w + 8 > row_w) {
        compact_font = true;
        d.setFont(&fonts::Font2);
        d.setTextSize(1);
      }
      const int text_h = d.fontHeight();
      const int row_h = max(compact_font ? 14 : 18, text_h + 6);
      box_w = min(w - 10, max(112, max_text_w + 22));
      row_w = box_w - 8;
      const int box_h = visible_count * row_h + 8;
      const int box_x = (w - box_w) / 2;
      const int box_y = content_top + (content_h - box_h) / 2;
      drawMenuBox(box_x, box_y, box_w, box_h);

      const int max_scroll = max(0, list_count - visible_count);
      const int scroll_start = min(static_cast<int>(emu_type_scroll), max_scroll);
      const int selected_row = min(static_cast<int>(emu_type_cursor), visible_count - 1);
      const int row_x = box_x + 4;

      for (int i = 0; i < visible_count; ++i) {
        const int idx = scroll_start + i;
        if (idx >= list_count) break;
        const bool selected = (i == selected_row);
        const int row_y = box_y + 4 + i * row_h;
        if (selected) {
          d.fillRect(row_x, row_y, row_w, row_h - 1, accent);
          d.setTextColor(TFT_BLACK, accent);
        } else {
          d.setTextColor(TFT_WHITE, TFT_BLACK);
        }
        const int text_y = row_y + max(0, ((row_h - 1) - text_h) / 2);
        d.setCursor(row_x + 6, text_y);
        d.print(typeMenuLabel(static_cast<uint8_t>(idx)));
      }
    } else if (menu_page == MenuPage::Emulator && emu_show_menu) {
      const int visible_count = kEmuMenuVisibleCount;
      const int list_count = kEmuActionCount;
      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      bool compact_font = false;
      int max_text_w = 0;
      for (int i = 0; i < list_count; ++i) {
        const int tw = d.textWidth(emuActionName(i));
        if (tw > max_text_w) max_text_w = tw;
      }
      int box_w = min(w - 10, max(112, max_text_w + 22));
      int row_w = box_w - 8;
      if (max_text_w + 8 > row_w) {
        compact_font = true;
        d.setFont(&fonts::Font2);
        d.setTextSize(1);
      }
      const int row_h = compact_font ? 14 : 18;
      box_w = min(w - 10, max(112, d.textWidth("ISO14443B") + 22));
      const int box_h = visible_count * row_h + 8;
      const int box_x = (w - box_w) / 2;
      const int box_y = content_top + (content_h - box_h) / 2;
      drawMenuBox(box_x, box_y, box_w, box_h);

      const int max_scroll = max(0, list_count - visible_count);
      const int scroll_start = min(static_cast<int>(emu_type_scroll), max_scroll);
      const int selected_row = min(static_cast<int>(emu_type_cursor), visible_count - 1);
      const int text_h = d.fontHeight();

      for (int i = 0; i < visible_count; ++i) {
        const int idx = scroll_start + i;
        if (idx >= list_count) break;
        const bool selected = (i == selected_row);
        const int row_y = box_y + 4 + i * row_h;
        if (selected) {
          d.fillRect(box_x + 4, row_y, box_w - 8, row_h - 1, accent);
          d.setTextColor(TFT_BLACK, accent);
        } else {
          d.setTextColor(TFT_WHITE, TFT_BLACK);
        }
        const int text_y = row_y + max(0, ((row_h - 1) - text_h) / 2);
        d.setCursor(box_x + 8, text_y);
        d.print(emuActionName(static_cast<uint8_t>(idx)));
      }
    } else if (menu_page == MenuPage::Reader) {
      drawReaderPixelCard(d, 4, content_top + 2, w - 8, content_h - 4, accent, last_card, reader_14b_only);
    } else if (menu_page == MenuPage::ReadNDEF) {
      String title = "Scanning";
      String body = "";
      if (!ndef_text.isEmpty()) {
        title = ndef_is_wifi ? "WiFi" : "Payload";
        body = ndef_text;
      } else if (ndef_detail.indexOf("No ISO14443A tag") < 0) {
        title = "Read Error";
        body = ndef_detail;
      }
      if (!wifi_status.isEmpty()) {
        if (!body.isEmpty()) body += " | ";
        body += wifi_status;
      }
      d.setTextColor(accent, TFT_BLACK);
      d.setCursor(4, content_top + 2);
      d.print(title);
      d.setTextColor(TFT_WHITE, TFT_BLACK);
      drawWrappedText(d, 4, content_top + 20, w - 8, 18, 2, 12, body, TFT_WHITE, TFT_BLACK);
    } else if (menu_page == MenuPage::WebFiles) {
      if (dumps_stage == DumpsStage::PortalQr || dumps_stage == DumpsStage::Preview) {
        drawDumpsPage(d, 0, 0, w, h, accent);
      } else {
        drawDumpsPage(d, 4, content_top + 2, w - 8, content_h - 4, accent);
      }
    } else if (menu_page == MenuPage::Emulator) {
      if (isNfcUnitMode()) {
        // Unit NFC: card-style layout — type centred + arrows + ID below. No Slot.
        d.fillRect(4, content_top + 2, w - 8, content_h - 4, TFT_BLACK);
        const int ex = 4, ey = content_top + 2, ew = w - 8, eh = content_h - 4;
        d.setFont(&fonts::Font0);
        d.setTextSize(2);
        const int uid_y = ey + eh - d.fontHeight() * 2 - 2;
        drawEmuTypeCarousel(d, ex, ey, ew, eh, accent);
        // ID line centred below — dynamic based on type support and emulation state
        String id_text;
        uint16_t id_color;
        if (emu_switch_apply_pending) {
          id_text = "";
          id_color = TFT_WHITE;
        } else if (emu_started) {
          id_text = activeEmulatorDisplayId(emu_type);
          id_color = TFT_GREEN;
        } else {
          id_text = "";
          id_color = TFT_WHITE;
        }
        drawEmuBottomLine(d, ex, uid_y, ew, id_text, id_color);
      } else {
        // Grove mode: card-style with centred type + arrows + coloured ID.
        d.fillRect(4, content_top + 2, w - 8, content_h - 4, TFT_BLACK);
        const int gex = 4, gey = content_top + 2, gew = w - 8, geh = content_h - 4;
        d.setFont(&fonts::Font0);
        d.setTextSize(2);
        const int uid_y = gey + geh - d.fontHeight() * 2 - 2;
        drawEmuTypeCarousel(d, gex, gey, gew, geh, accent);
        // ID line centred below
        {
          String grove_id_text;
          uint16_t grove_id_col;
          grove_id_text = emu_switch_apply_pending ? String("") : activeEmulatorDisplayId(emu_type);
          grove_id_col = emu_started ? (uint16_t)TFT_GREEN : (uint16_t)TFT_WHITE;
          drawEmuBottomLine(d, gex, uid_y, gew, grove_id_text, grove_id_col);
        }
      }
    } else if (menu_page == MenuPage::Piano) {
      d.setTextColor(accent, TFT_BLACK);
      d.setCursor(4, content_top + 2);
      d.print(piano_stage == PianoStage::Play ? "Play" : (piano_stage == PianoStage::Config ? "Scan" : "Menu"));
      d.setTextColor(TFT_WHITE, TFT_BLACK);
      if (piano_stage == PianoStage::Menu) {
        d.setCursor(4, content_top + 20);
        d.print(piano_menu_index == 0 ? "> Play" : "  Play");
        d.setCursor(4, content_top + 38);
        d.print(piano_menu_index == 1 ? "> Config" : "  Config");
        d.setCursor(4, content_top + 56);
        d.print(piano_menu_index == 2 ? "> Exit" : "  Exit");
      } else if (piano_stage == PianoStage::Config) {
        d.setCursor(4, content_top + 20);
        d.printf("Scan %s", kPianoNoteName[piano_config_step]);
        d.setCursor(4, content_top + 38);
        d.print(piano_status);
      } else {
        d.setCursor(4, content_top + 20);
        d.print("Note: " + piano_last_note);
        drawPianoKeyboard(d,
                          4,
                          content_top + 38,
                          w - 8,
                          max(18, content_h - 44),
                          piano_active_note_idx,
                          accent);
      }
    } else if (menu_page == MenuPage::About) {
      if (about_page_idx == 1) {
        drawAboutPage(d, 0, 0, w, h, accent, about_page_idx);
      } else {
        drawAboutPage(d, 4, content_top + 2, w - 8, content_h - 4, accent, about_page_idx);
      }
    } else {
      String head = diagnose_ok ? "DIAG PASS" : "DIAG CHECK";
      d.setTextColor(accent, TFT_BLACK);
      d.setCursor(4, content_top + 2);
      d.print(head);

      String hw_line = "HW 0x" + String(hw_ver, HEX);
      String fw_line = "FW 0x" + String(fw_ver, HEX);
      hw_line.toUpperCase();
      fw_line.toUpperCase();
      d.setTextColor(TFT_WHITE, TFT_BLACK);
      d.setCursor(4, content_top + 20);
      d.print(hw_line);
      d.setCursor(76, content_top + 20);
      d.print(fw_line);

      drawWrappedText(d, 4, content_top + 38, w - 8, 18, 2, 12, diagnose_report, TFT_WHITE, TFT_BLACK);
    }

    if (wifi_popup) {
      const int box_w = w - 10;
      const int box_h = h - 20;
      const int box_x = (w - box_w) / 2;
      const int box_y = (h - box_h) / 2;
      d.fillRect(box_x, box_y, box_w, box_h, TFT_BLACK);
      d.drawRect(box_x, box_y, box_w, box_h, TFT_CYAN);
      d.setTextColor(TFT_WHITE, TFT_BLACK);
      d.setCursor(box_x + 6, box_y + 8);
      d.print("WIFI?");
      drawWrappedText(d, box_x + 6, box_y + 22, box_w - 12, 12, 2, 6, wifi_ssid, TFT_WHITE, TFT_BLACK);
      d.setCursor(box_x + 6, box_y + box_h - 14);
      d.print("CLICK NO   HOLD YES");
    }
#ifdef APP_TARGET_M5PAPER
    pushCanvasToEink();
#else
    g_canvas.pushSprite(&M5.Display, 0, 0);
#endif
    return;
  }
#endif

  if (popup_only) {
    if (!(menu_page == MenuPage::Emulator && emu_config_stage == EmuConfigStage::TypeMenu)) {
      return;
    }

    d.setTextSize(2);
    d.setFont(&fonts::Font0);
    bool compact_font = false;

    const int list_count = static_cast<int>(dumpMenuCount());
    const int item_count = min(static_cast<int>(kEmuMenuVisibleCount), max(1, list_count));
    int max_text_w = 0;
    for (int i = 0; i < list_count; ++i) {
      const int tw = d.textWidth(typeMenuLabel(static_cast<uint8_t>(i)));
      if (tw > max_text_w) max_text_w = tw;
    }

    const int max_sel_w_limit = (w - 20) - 16;
    if (max_text_w + 10 > max_sel_w_limit) {
      compact_font = true;
      d.setTextSize(1);
      d.setFont(&fonts::Font2);
      max_text_w = 0;
      for (int i = 0; i < list_count; ++i) {
        const int tw = d.textWidth(typeMenuLabel(static_cast<uint8_t>(i)));
        if (tw > max_text_w) max_text_w = tw;
      }
    }

    const int row_h = compact_font ? 18 : 20;
    const int sel_w = min(max_sel_w_limit, max_text_w + 10);
    const int box_w = min(w - 20, max(80, sel_w + 16));
    const int box_h = item_count * row_h + 12;
    const int box_x = (w - box_w) / 2;
    const int box_y = 24 + (104 - box_h) / 2;

    d.fillRect(box_x - 3, box_y - 3, box_w + 6, box_h + 6, TFT_BLACK);
    d.fillRect(box_x, box_y, box_w, box_h, TFT_BLACK);
    d.drawRect(box_x - 2, box_y - 2, box_w + 4, box_h + 4, TFT_BLACK);
    d.drawRect(box_x - 1, box_y - 1, box_w + 2, box_h + 2, TFT_BLACK);
    d.drawRect(box_x, box_y, box_w, box_h, accent);
    d.drawRect(box_x + 1, box_y + 1, box_w - 2, box_h - 2, accent);
    d.fillRect(box_x, box_y, 3, 3, TFT_BLACK);
    d.fillRect(box_x + box_w - 3, box_y, 3, 3, TFT_BLACK);
    d.fillRect(box_x, box_y + box_h - 3, 3, 3, TFT_BLACK);
    d.fillRect(box_x + box_w - 3, box_y + box_h - 3, 3, 3, TFT_BLACK);
    d.fillRect(box_x + 2, box_y + 1, 2, 2, accent);
    d.fillRect(box_x + box_w - 4, box_y + 1, 2, 2, accent);
    d.fillRect(box_x + 2, box_y + box_h - 3, 2, 2, accent);
    d.fillRect(box_x + box_w - 4, box_y + box_h - 3, 2, 2, accent);

    const int visible_count = item_count;
    const int max_scroll = max(0, list_count - visible_count);
    const int scroll_start = min(static_cast<int>(emu_type_scroll), max_scroll);
    const int selected_row = min(static_cast<int>(emu_type_cursor), visible_count - 1);

    const int scroll_w = 4;
    const int scroll_gap = 3;
    const int row_x = box_x + 4;
    const int row_w = box_w - 8 - scroll_w - scroll_gap;
    const int text_h = d.fontHeight();

    for (int i = 0; i < visible_count; ++i) {
      const int idx = scroll_start + i;
      if (idx >= list_count) break;
      const bool selected = (i == selected_row);
      const int row_y = box_y + 6 + i * row_h;
      const int text_y = row_y + max(0, ((row_h - 2) - text_h) / 2);
      if (selected) {
        d.fillRect(row_x, row_y, row_w, row_h - 2, accent);
        d.setTextColor(TFT_BLACK, accent);
      } else {
        d.setTextColor(TFT_WHITE, TFT_BLACK);
      }
      d.setCursor(row_x + 6, text_y);
      d.print(typeMenuLabel(static_cast<uint8_t>(idx)));
    }

    const int track_x = box_x + box_w - scroll_w - 2;
    const int track_y = box_y + 7;
    const int track_h = row_h * visible_count - 2;
    d.drawRect(track_x, track_y, scroll_w, track_h, TFT_DARKGREY);
    const int thumb_h = max(6, ((track_h - 2) * visible_count) / list_count);
    int thumb_y = track_y + 1;
    if (max_scroll > 0) {
      const int safe_scroll = max(1, max_scroll);
      thumb_y += ((track_h - 2 - thumb_h) * scroll_start) / safe_scroll;
    }
    d.fillRect(track_x + 1, thumb_y, scroll_w - 2, thumb_h, accent);
    return;
  }

#ifdef APP_TARGET_M5PAPER
  // ---- M5Paper: 2-column grid home layout with status bar ----
  if (in_home) {
    auto& hd = g_canvas;
    const int screen_w = M5.Display.width();
    const int screen_h = M5.Display.height();
    hd.fillScreen(TFT_WHITE);

    const int sb_h = kStatusBarHeight;
    hd.fillRect(0, 0, w, sb_h, TFT_WHITE);
    hd.drawLine(0, sb_h, w, sb_h, TFT_BLACK);

    hd.setFont(&fonts::Font0);
    hd.setTextSize(2);
    hd.setTextColor(TFT_BLACK, TFT_WHITE);
    hd.setCursor(4, 5);
    hd.print("GroveNFC");
    hd.setTextSize(1);
    const String module_port = nfc_module_name + " " + nfc_port_name;
    hd.setCursor(screen_w - hd.textWidth(module_port) - 4, 8);
    hd.print(module_port);

    const int items = homePageCount();
    for (int i = 0; i < items; ++i) {
      const auto tile = getM5PaperHomeTile(i, screen_w, screen_h);
      const bool selected = (i == home_index);
      const MenuPage pg = homePageAt(i);

      if (selected) {
        hd.fillRoundRect(tile.x, tile.y, tile.w, tile.h, 8, TFT_BLACK);
        hd.drawRoundRect(tile.x, tile.y, tile.w, tile.h, 8, TFT_BLACK);
      } else {
        hd.drawRoundRect(tile.x, tile.y, tile.w, tile.h, 8, TFT_BLACK);
      }

      drawM5PaperHomeIcon(tile.icon_cx, tile.icon_cy, pg, selected ? TFT_WHITE : TFT_BLACK);

      hd.setFont(&fonts::Font0);
      hd.setTextSize(2);
      hd.setTextColor(selected ? TFT_WHITE : TFT_BLACK, selected ? TFT_BLACK : TFT_WHITE);
      const String name = String(homePageName(pg));
      const int label_w = hd.textWidth(name);
      hd.setCursor(tile.icon_cx - label_w / 2, tile.y + tile.h - 28);
      hd.print(name);
    }

    hd.setFont(&fonts::Font0);
    hd.setTextSize(1);
    hd.setTextColor(TFT_BLACK, TFT_WHITE);
    if (!boot_notice_line.isEmpty()) {
      const int notice_w = hd.textWidth(boot_notice_line);
      hd.setCursor(screen_w - notice_w - 4, screen_h - 12);
      hd.print(boot_notice_line);
    }
    pushCanvasToEink(true);
    return;
  }
#else
  if (in_home) {

    // Arrows with flash feedback
    auto drawArrow = [&](bool left, uint16_t col) {
      if (left) {
        d.fillRect(8, 66, 4, 4, col);
        d.fillRect(12, 62, 4, 12, col);
        d.fillRect(16, 58, 4, 20, col);
      } else {
        d.fillRect(w - 12, 66, 4, 4, col);
        d.fillRect(w - 16, 62, 4, 12, col);
        d.fillRect(w - 20, 58, 4, 20, col);
      }
    };
    drawArrow(true,  accent);
    drawArrow(false, accent);
    const uint32_t now_ms = millis();
    const bool flash_right_ok = home_arrow_flash_right && now_ms < home_arrow_flash_until_ms;
    const bool flash_left_ok  = home_arrow_flash_left  && now_ms < home_arrow_flash_until_ms;
    if (flash_right_ok) {
      drawArrow(false, TFT_BLACK);  // hide right arrow
    } else if (flash_left_ok) {
      drawArrow(true,  TFT_BLACK);  // hide left arrow
    } else {
      home_arrow_flash_right = false;
      home_arrow_flash_left = false;
    }

    const int icon_cx = w / 2;
    const int icon_cy = 57;
    const int icon_box_w = 60;
    const int icon_box_h = 56;
    const int icon_box_x = icon_cx - icon_box_w / 2;
    const int icon_box_y = 29;

    if (home_anim_active) {
      const uint32_t now = millis();
      const uint32_t elapsed = now - home_anim_start_ms;
      float t = 1.0f;
      if (elapsed < home_anim_duration_ms) {
        t = static_cast<float>(elapsed) / static_cast<float>(home_anim_duration_ms);
      } else {
        home_anim_active = false;
      }
      const float eased = 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t);
      const float travel = 1.0f - eased;

      d.fillRect(0, icon_box_y, w, icon_box_h, TFT_BLACK);

      // ---- Icon: slide + alpha fade ----
      const int travel_px = static_cast<int>(travel * icon_cx);
      const int from_cx = icon_cx - static_cast<int>(eased * icon_cx) * home_anim_dir;
      const int to_cx   = icon_cx + travel_px * home_anim_dir;

      // "From" icon: slides away, fades out (fast)
      {
        const uint8_t a = static_cast<uint8_t>(max(0, 255 - static_cast<int>(min(1.0f, eased * 2.0f) * 255.0f)));
        if (a > 4) {
          drawHomeIconForPage(d, from_cx, icon_cy, home_anim_from, scaleColor565(accent, a));
        }
      }

      // "To" icon: slides in, fades in (early — starts at eased=0.10)
      {
        const float reveal = max(0.0f, min(1.0f, (eased - 0.10f) / 0.90f));
        const uint8_t a = static_cast<uint8_t>(max(0, static_cast<int>(reveal * 255.0f)));
        if (a > 4) {
          drawHomeIconForPage(d, to_cx, icon_cy, home_anim_to, scaleColor565(accent, a));
        }
      }

      // ---- Text: staggered slide + alpha fade ----
      // Text slides same speed as icon but fades in much later (alpha delay),
      // so it appears "behind" without a position jump.
      const float text_eased = max(0.0f, min(1.0f, (eased - 0.10f) / 0.90f));
      const float text_travel = 1.0f - text_eased;

      const int from_tx = icon_cx - static_cast<int>(eased * icon_cx) * home_anim_dir;
      const int to_tx   = icon_cx + static_cast<int>(text_travel * icon_cx) * home_anim_dir;

      const String name_from = String(homePageName(home_anim_from));
      const String name_to = String(homePageName(home_anim_to));
      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      d.fillRect(0, 94, w, 22, TFT_BLACK);

      // "From" text: fast fade out — sync with icon (use eased)
      {
        const uint8_t a = static_cast<uint8_t>(max(0, 255 - static_cast<int>(min(1.0f, eased * 2.0f) * 255.0f)));
        if (a > 4) {
          const int nw = d.textWidth(name_from);
          d.setTextColor(scaleColor565(accent, a), TFT_BLACK);
          d.setCursor(from_tx - nw / 2, 98);
          d.print(name_from);
        }
      }
      // "To" text: slow fade in — extra delay
      {
        const float reveal_t = max(0.0f, min(1.0f, (text_eased - 0.55f) / 0.45f));
        const uint8_t a = static_cast<uint8_t>(max(0, static_cast<int>(reveal_t * 255.0f)));
        if (a > 4) {
          const int nw = d.textWidth(name_to);
          d.setTextColor(scaleColor565(accent, a), TFT_BLACK);
          d.setCursor(to_tx - nw / 2, 98);
          d.print(name_to);
        }
      }
    } else {
      d.fillRect(icon_box_x, icon_box_y, icon_box_w, icon_box_h, TFT_BLACK);
      drawHomeIconForPage(d, icon_cx, icon_cy, menu_page, accent);

      d.setTextSize(2);
      d.setFont(&fonts::Font0);
      d.setTextColor(accent, TFT_BLACK);
      const String& home_name = homePageName(menu_page);
      d.fillRect(0, 94, w, 22, TFT_BLACK);
      d.setTextSize(2);
      const int home_name_w = d.textWidth(home_name);
      const int home_name_x = (w - home_name_w) / 2;
      d.setCursor(home_name_x, 98);
      d.print(home_name);
    }

    if (!boot_notice_line.isEmpty()) {
      d.setTextSize(1);
      d.setTextColor(TFT_RED, TFT_BLACK);
      d.setCursor(4, 118);
      d.print(boot_notice_line);
    }
#ifdef APP_TARGET_M5PAPER
    pushCanvasToEink();
#else
    g_canvas.pushSprite(&M5.Display, 0, 0);
#endif
    return;
  }
#endif

  String title_line;
  String sub_title_line;
  String body_line;
  String emu_id_line;
  if (menu_page == MenuPage::Reader) {
    sub_title_line = "Reader";
    if (last_card.valid) {
      title_line = readerTypeLabel(last_card, reader_14b_only);
    } else {
      title_line = reader_14b_only ? String("Scan 14B") : String("Scanning");
    }
    body_line = readerBodyText(last_card);
    if (!reader_14b_only) {
      if (body_line.isEmpty()) body_line = "Mode: AUTO";
      else body_line += "  AUTO";
    } else {
      if (body_line.isEmpty()) body_line = "Mode: 14B ONLY";
      else body_line += "  14B";
    }
    body_line += "  " + nfc_module_name;
  } else if (menu_page == MenuPage::ReadNDEF) {
    sub_title_line = "NDEF";
    title_line = "Scanning";
    if (!ndef_text.isEmpty()) {
      title_line = ndef_is_wifi ? "WiFi" : "Payload";
      body_line = ndef_text;
    } else {
      if (ndef_detail.indexOf("No ISO14443A tag") >= 0) {
        title_line = "Scanning";
        body_line = "";
      } else {
        title_line = "Read Error";
        body_line = ndef_detail;
      }
    }
    if (!wifi_status.isEmpty()) {
      body_line += " | ";
      body_line += wifi_status;
    }
  } else if (menu_page == MenuPage::Emulator) {
    emu_user_stopped = false;
    sub_title_line = "Emulator";
    title_line = String(emuName(emu_type));
    const String emu_id = activeEmulatorDisplayId(emu_type);
    if (isNfcUnitMode()) {
      emu_id_line = activeEmulatorDisplayId(emu_type);
      body_line = emu_id_line;
    } else {
      emu_id_line = emu_id;
      body_line = emu_id_line;
      if (!emu_dump_status.isEmpty()) body_line += " | " + emu_dump_status;
    }
  } else if (menu_page == MenuPage::WebFiles) {
    sub_title_line = "Dumps";
    title_line = emu_ap_active ? "AP ON" : "AP OFF";
    body_line = emu_dump_status;
  } else if (menu_page == MenuPage::About) {
    sub_title_line = "About";
    title_line = "GroveNFC";
  } else if (menu_page == MenuPage::Piano) {
    sub_title_line = "Piano";
    if (piano_stage == PianoStage::Menu) {
      title_line = "Select";
      body_line = String(piano_menu_index == 0 ? ">" : " ") + "Play  " +
                  String(piano_menu_index == 1 ? ">" : " ") + "Config  " +
                  String(piano_menu_index == 2 ? ">" : " ") + "Exit";
    } else if (piano_stage == PianoStage::Config) {
      title_line = String("Scan ") + String(piano_config_step + 1) + "/8";
      body_line = String("Scan ") + kPianoNoteName[piano_config_step] + " | " + piano_status;
    } else {
      title_line = "Play";
      body_line = "Note " + piano_last_note + " | " + piano_status;
    }
  } else {
    sub_title_line = "Diagnose";
    title_line = diagnose_ok ? "DIAG PASS" : "DIAG CHECK";
    body_line = "HW 0x" + String(hw_ver, HEX) + " FW 0x" + String(fw_ver, HEX) + " " + diagnose_report;
  }

  ui_marquee_active = false;
  if (!in_home && (menu_page == MenuPage::Reader || menu_page == MenuPage::ReadNDEF)) {
    body_line.replace('\n', ' ');
    ui_marquee_active = body_line.length() > 18;
    if (ui_marquee_active) {
      body_line = marqueeText(body_line, 18, 220);
    }
  }

  d.fillRect(0, 0, w, 22, TFT_BLACK);
  d.drawLine(0, 22, w, 22, accent);

  d.setTextSize(2);
  d.setFont(&fonts::Font0);
  d.setCursor(2, 2);
  d.setTextColor(accent, TFT_BLACK);
  d.print("<");
  const int sub_w = d.textWidth(sub_title_line);
  const int header_left = 14;
  const int header_width = w - header_left;
  d.setCursor(header_left + (header_width - sub_w) / 2, 2);
  d.print(sub_title_line);
  if (menu_page == MenuPage::WebFiles && emu_dump_count > 0 && !dumps_pick_for_emu) {
    const String counter = String(dump_file_index + 1) + "/" + String(emu_dump_count);
    d.setTextColor(accent, TFT_BLACK);
   const int cw = d.textWidth(counter);
   d.setCursor(w - cw - 2, 2);
   d.print(counter);
  }
  // Battery icon on Reader/Emulator pages
  if (menu_page == MenuPage::Reader || menu_page == MenuPage::Emulator) {
    drawBatteryIcon(d, w - 22, 6, 18, 10, accent);
  }

  d.setTextSize(2);
  d.setFont(&fonts::Font0);
  d.setCursor(4, 26);
  d.setTextColor(accent, TFT_BLACK);
  d.print(title_line);

  int body_y = 54;

  if (menu_page == MenuPage::Reader) {
    drawReaderPixelCard(d, 4, 26, w - 8, h - 30, accent, last_card, reader_14b_only);
  } else if (menu_page == MenuPage::ReadNDEF) {
    d.setFont(&fonts::Font0);
    d.setTextSize(2);
    drawWrappedText(d, 4, body_y, w - 8, 18, 4, 12, body_line, TFT_WHITE, TFT_BLACK);
  } else if (menu_page == MenuPage::Emulator) {
    d.setFont(&fonts::Font0);
    d.setTextSize(2);
    if (isNfcUnitMode()) {
      // Unit NFC: replicate Reader card style (type centred + arrows + ID below).
      // Clear the whole card area (overwriting the header title_line printed above).
      const int ex = 4, ey = 26, ew = w - 8, eh = h - 30;
      d.fillRect(ex, ey, ew, eh, TFT_BLACK);
      const int uid_y = ey + eh - d.fontHeight() * 2 - 2;
      drawEmuTypeCarousel(d, ex, ey, ew, eh, accent);
      // ID line centred below – runtime colour based on emulation state
      {
        String nu_id;
        uint16_t nu_col;
        if (emu_switch_apply_pending) {
          nu_id  = "";
          nu_col = TFT_WHITE;
        } else if (emu_started) {
          nu_id  = activeEmulatorDisplayId(emu_type);
          nu_col = TFT_GREEN;
        } else {
          nu_id  = "";
          nu_col = TFT_WHITE;
        }
        drawEmuBottomLine(d, ex, uid_y, ew, nu_id, nu_col);
      }
    } else {
      // Grove mode: card-style with centred type + arrows + coloured ID.
      const int gex = 4, gey = 26, gew = w - 8, geh = h - 30;
      const int uid_y = gey + geh - d.fontHeight() * 2 - 2;
      d.fillRect(gex, gey, gew, geh, TFT_BLACK);
      drawEmuTypeCarousel(d, gex, gey, gew, geh, accent);
      // ID line centred below
      {
        String ls_show;
        uint16_t ls_col;
        ls_show = emu_switch_apply_pending ? String("") : activeEmulatorDisplayId(emu_type);
        ls_col = emu_started ? (uint16_t)TFT_GREEN : (uint16_t)TFT_WHITE;
        drawEmuBottomLine(d, gex, uid_y, gew, ls_show, ls_col);
      }
    }
  } else if (menu_page == MenuPage::WebFiles) {
    if (dumps_stage == DumpsStage::PortalQr || dumps_stage == DumpsStage::Preview) {
      drawDumpsPage(d, 0, 0, w, h, accent);
    } else {
      drawDumpsPage(d, 4, 26, w - 8, h - 30, accent);
    }
  } else if (menu_page == MenuPage::About) {
    if (about_page_idx == 1) {
      drawAboutPage(d, 0, 0, w, h, accent, about_page_idx);
    } else {
      drawAboutPage(d, 4, 26, w - 8, h - 30, accent, about_page_idx);
    }
  } else if (menu_page == MenuPage::Diagnose) {
    d.setFont(&fonts::Font2);
    d.setTextSize(1);

    String hw_line = "HW 0x" + String(hw_ver, HEX);
    hw_line.toUpperCase();
    String fw_line = "FW 0x" + String(fw_ver, HEX);
    fw_line.toUpperCase();

    drawWrappedText(d, 4, 46, w - 8, 14, 1, 8, hw_line, TFT_WHITE, TFT_BLACK);
    drawWrappedText(d, 4, 60, w - 8, 14, 1, 8, fw_line, TFT_WHITE, TFT_BLACK);

    String report_lines[10];
    int report_count = 0;
    size_t pos = 0;
    while (pos < diagnose_report.length() && report_count < 10) {
      int end = diagnose_report.indexOf('\n', pos);
      if (end < 0) end = diagnose_report.length();
      String item = diagnose_report.substring(pos, end);
      item.trim();
      if (!item.isEmpty() && !item.startsWith("HW:") && !item.startsWith("FW:")) {
        report_lines[report_count++] = item;
      }
      pos = static_cast<size_t>(end) + 1;
    }

    d.fillRect(2, 74, w - 4, 52, TFT_BLACK);
    const int visible = 3;
    int start = 0;
    if (report_count > visible && diagnose_ok) {
      start = (millis() / kDiagScrollMs) % (report_count - visible + 1);
    }
    for (int i = 0; i < visible && start + i < report_count; ++i) {
      drawWrappedText(d, 4, 76 + i * 16, w - 8, 14, 1, 8, report_lines[start + i], TFT_WHITE, TFT_BLACK);
    }
  } else if (menu_page == MenuPage::Piano && piano_stage == PianoStage::Play) {
    d.setFont(&fonts::Font0);
    d.setTextSize(2);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setCursor(4, body_y);
    d.print("Note: " + piano_last_note);
    drawPianoKeyboard(d, 4, body_y + 20, w - 8, h - (body_y + 24), piano_active_note_idx, accent);
  } else {
    d.setFont(&fonts::Font0);
    d.setTextSize(2);
    drawWrappedText(d, 4, body_y, w - 8, 18, 4, 12, body_line, TFT_WHITE, TFT_BLACK);
  }
  d.setFont(&fonts::Font0);
  d.setTextColor(TFT_WHITE, TFT_BLACK);

  if (menu_page == MenuPage::Emulator) {
    if (emu_config_stage == EmuConfigStage::None && emu_show_menu) {
      const int list_count = kEmuActionCount;
      const int visible_count = kEmuMenuVisibleCount;
      d.setFont(&fonts::Font0);
      d.setTextSize(2);
      bool compact_font = false;

      int max_text_w = 0;
      {
        for (int i = 0; i < list_count; ++i) {
          const int tw_item = d.textWidth(emuActionName(i));
          if (tw_item > max_text_w) max_text_w = tw_item;
        }
      }

      const int max_sel_w_limit = (w - 20) - 16;
      if (max_text_w + 10 > max_sel_w_limit) {
        compact_font = true;
        d.setFont(&fonts::Font2);
        d.setTextSize(1);
        max_text_w = 0;
        for (int i = 0; i < list_count; ++i) {
          const int tw_item = d.textWidth(emuActionName(i));
          if (tw_item > max_text_w) max_text_w = tw_item;
        }
      }

      const int row_h = compact_font ? 18 : 20;
      const int item_count = visible_count;
      const int sel_w = min(max_sel_w_limit, max_text_w + 10);
      const int box_w = min(w - 20, max(80, sel_w + 16));
      const int box_h = item_count * row_h + 12;
      const int box_x = (w - box_w) / 2;
      const int box_y = 24 + (104 - box_h) / 2;

      d.fillRect(box_x, box_y, box_w, box_h, TFT_BLACK);
      d.drawRect(box_x - 2, box_y - 2, box_w + 4, box_h + 4, TFT_BLACK);
      d.drawRect(box_x - 1, box_y - 1, box_w + 2, box_h + 2, TFT_BLACK);
      d.drawRect(box_x, box_y, box_w, box_h, accent);
      d.drawRect(box_x + 1, box_y + 1, box_w - 2, box_h - 2, accent);
      // pixel rounded corners + bigger corner dots
      d.fillRect(box_x, box_y, 3, 3, TFT_BLACK);
      d.fillRect(box_x + box_w - 3, box_y, 3, 3, TFT_BLACK);
      d.fillRect(box_x, box_y + box_h - 3, 3, 3, TFT_BLACK);
      d.fillRect(box_x + box_w - 3, box_y + box_h - 3, 3, 3, TFT_BLACK);
      d.fillRect(box_x + 2, box_y + 1, 2, 2, accent);
      d.fillRect(box_x + box_w - 4, box_y + 1, 2, 2, accent);
      d.fillRect(box_x + 2, box_y + box_h - 3, 2, 2, accent);
      d.fillRect(box_x + box_w - 4, box_y + box_h - 3, 2, 2, accent);

      const int max_scroll = max(0, list_count - visible_count);
      const int scroll_start = min(static_cast<int>(emu_type_scroll), max_scroll);
      const int selected_row = min(static_cast<int>(emu_type_cursor), visible_count - 1);
      const int scroll_w = 4;
      const int scroll_gap = 3;

      for (int i = 0; i < visible_count; ++i) {
        const int idx = scroll_start + i;
        String label = emuActionName(static_cast<uint8_t>(idx));
        const bool selected = (i == selected_row);
        const int row_y = box_y + 6 + i * row_h;
        const int row_x = box_x + 4;
        const int row_w = box_w - 8 - scroll_w - scroll_gap;
        const int text_h = d.fontHeight();
        const int text_y = row_y + max(0, ((row_h - 2) - text_h) / 2);
        if (selected) {
          d.fillRect(row_x, row_y, row_w, row_h - 2, accent);
          d.setTextColor(TFT_BLACK, accent);
        } else {
          d.setTextColor(TFT_WHITE, TFT_BLACK);
        }
        d.setCursor(row_x + 6, text_y);
        d.print(label);
      }

      const int track_x = box_x + box_w - scroll_w - 2;
      const int track_y = box_y + 7;
      const int track_h = row_h * visible_count - 2;
      d.drawRect(track_x, track_y, scroll_w, track_h, TFT_DARKGREY);
      const int thumb_h = max(6, ((track_h - 2) * visible_count) / list_count);
      int thumb_y = track_y + 1;
      if (max_scroll > 0) {
        const int safe_scroll = max(1, max_scroll);
        thumb_y += ((track_h - 2 - thumb_h) * scroll_start) / safe_scroll;
      }
      d.fillRect(track_x + 1, thumb_y, scroll_w - 2, thumb_h, accent);
#ifdef APP_TARGET_M5PAPER
      pushCanvasToEink();
#else
      g_canvas.pushSprite(&M5.Display, 0, 0);
#endif
      return;
    }
  }

  if (menu_page == MenuPage::Emulator && emu_config_stage != EmuConfigStage::None) {
    d.setTextSize(2);
    d.setFont(&fonts::Font0);
    bool compact_font = false;

    int item_count = 4;
    int visible_count_menu = 4;
    int max_text_w = 0;
    if (emu_config_stage == EmuConfigStage::TypeMenu) {
      const int list_count = static_cast<int>(dumpMenuCount());
      visible_count_menu = min(static_cast<int>(kEmuMenuVisibleCount), max(1, list_count));
      item_count = visible_count_menu;
      for (int i = 0; i < list_count; ++i) {
        const int tw = d.textWidth(typeMenuLabel(static_cast<uint8_t>(i)));
        if (tw > max_text_w) max_text_w = tw;
      }
    }

    const int max_sel_w_limit = (w - 20) - 16;
    if (max_text_w + 10 > max_sel_w_limit) {
      compact_font = true;
      d.setTextSize(1);
      d.setFont(&fonts::Font2);
      max_text_w = 0;
      if (emu_config_stage == EmuConfigStage::TypeMenu) {
        const int list_count = static_cast<int>(dumpMenuCount());
        for (int i = 0; i < list_count; ++i) {
          const int tw = d.textWidth(typeMenuLabel(static_cast<uint8_t>(i)));
          if (tw > max_text_w) max_text_w = tw;
        }
      }
    }

    const int text_h_menu = d.fontHeight();
    const int row_h = max(compact_font ? 18 : 20, text_h_menu + 6);

    const int sel_w = min(max_sel_w_limit, max_text_w + 10);
    const int box_w = min(w - 20, max(80, sel_w + 16));
    const int box_h = item_count * row_h + 12;
    const int box_x = (w - box_w) / 2;
    const int box_y = 24 + (104 - box_h) / 2;

    d.fillRect(box_x, box_y, box_w, box_h, TFT_BLACK);
    d.drawRect(box_x - 2, box_y - 2, box_w + 4, box_h + 4, TFT_BLACK);
    d.drawRect(box_x - 1, box_y - 1, box_w + 2, box_h + 2, TFT_BLACK);
    d.drawRect(box_x, box_y, box_w, box_h, accent);
    d.drawRect(box_x + 1, box_y + 1, box_w - 2, box_h - 2, accent);
    // pixel rounded corners + bigger corner dots
    d.fillRect(box_x, box_y, 3, 3, TFT_BLACK);
    d.fillRect(box_x + box_w - 3, box_y, 3, 3, TFT_BLACK);
    d.fillRect(box_x, box_y + box_h - 3, 3, 3, TFT_BLACK);
    d.fillRect(box_x + box_w - 3, box_y + box_h - 3, 3, 3, TFT_BLACK);
    d.fillRect(box_x + 2, box_y + 1, 2, 2, accent);
    d.fillRect(box_x + box_w - 4, box_y + 1, 2, 2, accent);
    d.fillRect(box_x + 2, box_y + box_h - 3, 2, 2, accent);
    d.fillRect(box_x + box_w - 4, box_y + box_h - 3, 2, 2, accent);

    if (emu_config_stage == EmuConfigStage::TypeMenu) {
      const int list_count = static_cast<int>(dumpMenuCount());
      const int visible_count = visible_count_menu;
      const int max_scroll = max(0, list_count - visible_count);
      const int scroll_start = min(static_cast<int>(emu_type_scroll), max_scroll);
      const int selected_row = min(static_cast<int>(emu_type_cursor), visible_count - 1);

      const int scroll_w = 4;
      const int scroll_gap = 3;
      const int row_x = box_x + 4;
      const int row_w = box_w - 8 - scroll_w - scroll_gap;
      const int text_h = d.fontHeight();

      for (int i = 0; i < visible_count; ++i) {
        const int idx = scroll_start + i;
        if (idx >= list_count) break;
        const bool selected = (i == selected_row);
        const int row_y = box_y + 6 + i * row_h;
        const int text_y = row_y + max(0, ((row_h - 2) - text_h) / 2);
        if (selected) {
          d.fillRect(row_x, row_y, row_w, row_h - 2, accent);
          d.setTextColor(TFT_BLACK, accent);
        } else {
          d.setTextColor(TFT_WHITE, TFT_BLACK);
        }
        d.setCursor(row_x + 6, text_y);
        d.print(typeMenuLabel(static_cast<uint8_t>(idx)));
      }

      const int track_x = box_x + box_w - scroll_w - 2;
      const int track_y = box_y + 7;
      const int track_h = row_h * visible_count - 2;
      d.drawRect(track_x, track_y, scroll_w, track_h, TFT_DARKGREY);
      const int thumb_h = max(6, ((track_h - 2) * visible_count) / list_count);
      int thumb_y = track_y + 1;
      if (max_scroll > 0) {
        const int safe_scroll = max(1, max_scroll);
        thumb_y += ((track_h - 2 - thumb_h) * scroll_start) / safe_scroll;
      }
      d.fillRect(track_x + 1, thumb_y, scroll_w - 2, thumb_h, accent);
    }

    d.setTextSize(1);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
  }

  if (wifi_popup) {
    d.fillRoundRect(2, 16, w - 4, 110, 5, TFT_DARKGREY);
    d.fillRoundRect(4, 18, w - 8, 106, 5, TFT_BLACK);
    d.drawRoundRect(4, 18, w - 8, 106, 5, TFT_CYAN);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setTextSize(2);
    d.setFont(&fonts::Font2);
    d.setCursor(10, 26);
    d.print("WIFI?");
    drawWrappedText(d, 10, 52, w - 20, 16, 3, 12, wifi_ssid, TFT_WHITE, TFT_BLACK);
    d.setTextSize(1);
    d.setCursor(10, 112);
    d.print("CLICK NO  HOLD YES");
    d.setTextColor(TFT_WHITE, TFT_BLACK);
  }
#ifdef APP_TARGET_M5PAPER
  pushCanvasToEink();
#else
  g_canvas.pushSprite(&M5.Display, 0, 0);
#endif
}

bool recoverNfc(const char* reason, bool rebegin) {
  if (!nfc_ready) return false;
  const uint32_t now = millis();
  if (now - last_recover_ms < kRecoverCooldownMs) return false;
  last_recover_ms = now;

  if (reason && reason[0] != '\0') {
    Serial.printf("[NFC] Recover: %s\n", reason);
  }

  // Delegate to NFC worker on Core 0
  bool ok = false;
  if (sendNfcCmdAndWait(NfcCmd::Recover, 3000)) {
    ok = nfc_cmd_result.ok;
  }

  if (!ok) {
    nfc_ready = false;
    stopAllModes();
    Serial.println("[NFC] Recover failed, switch to reconnect flow");
    return false;
  }

  last_reader_success_ms = now;
  return true;
}

bool initNfcAtBoot() {
  auto beginWireWithActivePins = [&]() {
#if defined(APP_TARGET_M5PAPER)
    Wire.end();
    delay(2);
    Wire.begin(active_sda_pin, active_scl_pin, kI2CFreq);
    delay(8);
#elif defined(APP_TARGET_STICKS3) || defined(APP_TARGET_STICKCPLUS) || defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
  Wire.begin(active_sda_pin, active_scl_pin, kI2CFreq);
    delay(5);
#endif
  };

  auto tryInit = [&]() -> bool {
    for (uint8_t attempt = 1; attempt <= kNfcBootRetryCount; ++attempt) {
      const bool ok = nfc.begin();
      if (ok) {
        if (attempt > 1) {
          Serial.printf("[BOOT] NFC init recovered on attempt %u/%u\n", attempt, kNfcBootRetryCount);
        }
        return true;
      }

      Serial.printf("[BOOT] NFC init attempt %u/%u failed\n", attempt, kNfcBootRetryCount);
      delay(kNfcBootRetryDelayMs);
      beginWireWithActivePins();
    }
    return false;
  };

  delay(kNfcBootPowerSettleMs);

#if defined(APP_TARGET_M5PAPER)
  struct M5PaperPortPins {
    const char* name;
    int sda;
    int scl;
  };
  static constexpr M5PaperPortPins ports[] = {
      {"Port A", 25, 32},
      {"Port B", 26, 33},
      {"Port C", 18, 19},
  };
  for (const auto& port : ports) {
    active_sda_pin = port.sda;
    active_scl_pin = port.scl;
    nfc_port_name = port.name;
    Serial.printf("[BOOT] NFC probe %s SDA=GPIO%d SCL=GPIO%d\n",
                  port.name, port.sda, port.scl);
    beginWireWithActivePins();
    if (tryInit()) {
      Serial.printf("[BOOT] NFC found on %s\n", port.name);
      return true;
    }
  }
  nfc_port_name = "No Port";
  return false;
#else
  if (tryInit()) return true;

#if defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
  // Some CardPuter ADV wiring harnesses label Grove as G1/G2 but route I2C in reverse order.
  // If default (GPIO2/GPIO1) fails, retry with swapped pins (GPIO1/GPIO2).
  // Safe to run on both CardPuter variants — harmless on standard boards.
  if (active_sda_pin == 2 && active_scl_pin == 1) {
    active_sda_pin = 1;
    active_scl_pin = 2;
    Serial.println("[BOOT] I2C fallback: retry on SDA=GPIO1 SCL=GPIO2");
    beginWireWithActivePins();
    if (tryInit()) return true;
  }
#endif

  return false;
#endif
}

void stopAllModes() {
  sendNfcCmdAndWait(NfcCmd::StopRF, 1000);
  emu_started = false;
}

void goHome() {
  in_home = true;
  menu_page = MenuPage::Reader;
  wifi_popup = false;
  emu_config_stage = EmuConfigStage::None;
  emu_show_menu = false;
  emu_switch_apply_pending = false;
  dumps_stage = DumpsStage::Browse;
  dumps_pick_for_emu = false;
#ifdef APP_TARGET_M5PAPER
  home_index = -1;
#else
  home_index = 0;
#endif
  setSpeaker5V(false);
  drawScreen();
}

void enterCurrentFeature() {
  in_home = false;
  boot_notice_line = "";
  wifi_popup = false;
  emu_config_stage = EmuConfigStage::None;

  if (menu_page == MenuPage::Reader) {
    if (nfc_ready && isNfcUnitMode() && nfc.isNfcUnitEmulating()) {
      sendNfcCmdAndWait(NfcCmd::StopRF, 2000);
      sendNfcCmdAndWait(NfcCmd::Recover, 3000);
    }
    last_card.valid = false;
    last_card.protocol = "None";
    last_card.uid = "";
    last_card.detail = "Scanning";
    reader_14b_only = false;
    reader_last_hold_log_ms = 0;
    reader_fail_streak = 0;
    reader_need_first_tone = true;
    playTone(880, 200);
    delay(250);
    setSpeaker5V(true);
    reader_need_first_tone = false;
    last_poll_ms = 0;
  } else if (menu_page == MenuPage::ReadNDEF) {
    ndef_text = "";
    ndef_detail = "Scanning";
    ndef_is_wifi = false;
    wifi_status = "";
    last_ndef_auto_ms = 0;
  } else if (menu_page == MenuPage::Emulator) {
    emu_switch_apply_pending = false;
    // GroveNFC has native MIFARE Classic tag modes. NFC Unit keeps NTAG213
    // because its Classic emulation backend is intentionally unavailable.
    emu_type = isNfcUnitMode() ? EmuType::N213 : EmuType::MF1K;
    emu_menu_index = 1;
    emu_show_menu = false;
    const String saved_path = loadLastDumpForType(emu_type);
    if (saved_path.isEmpty()) {
      updateEmulatorSourceStatus();
    } else {
      emu_dump_status = "Saved " + shortDumpName(saved_path, 14);
    }
    if (nfc_ready) {
      startCurrentEmulation();
      emu_menu_index = 1;
    }
  } else if (menu_page == MenuPage::WebFiles) {
    dumps_pick_for_emu = false;
    dumps_stage = DumpsStage::Browse;
    dumps_menu_index = 0;
    startEmuApPortal();
    refreshDumpFiles(false);
  } else if (menu_page == MenuPage::About) {
    about_page_idx = 0;
  } else if (menu_page == MenuPage::Diagnose) {
    runDiagnose();
    return;
  } else if (menu_page == MenuPage::Piano) {
    piano_stage = PianoStage::Menu;
    piano_menu_index = 0;
    piano_active_card_key = "";
    piano_active_note_idx = -1;
    piano_last_sustain_ms = 0;
    piano_last_note = "-";
    const uint8_t mapped = pianoMappedCount();
    piano_status = "Configured " + String(mapped) + "/8";
  }

  drawScreen();
}

void enterMenu(MenuPage mode) {
  stopAllModes();
  menu_page = mode;
  wifi_popup = false;
  in_home = false;

  if (menu_page == MenuPage::Reader) {
    last_card.valid = false;
    last_card.protocol = "None";
    last_card.uid = "";
    last_card.detail = "Waiting card...";
    reader_14b_only = false;
    reader_last_hold_log_ms = 0;
    reader_fail_streak = 0;
    reader_need_first_tone = true;
  }

  if (menu_page == MenuPage::ReadNDEF) {
    ndef_text = "";
    ndef_detail = "Hold to scan";
    ndef_is_wifi = false;
    wifi_status = "";
    last_ndef_auto_ms = 0;
  }

  if (menu_page == MenuPage::Emulator && nfc_ready) {
    emu_config_stage = EmuConfigStage::None;
    emu_menu_type = emu_type;
    emu_menu_index = 1;
    emu_show_menu = false;
  } else if (menu_page == MenuPage::Piano) {
    piano_stage = PianoStage::Menu;
    piano_menu_index = 0;
    piano_active_card_key = "";
    piano_active_note_idx = -1;
    piano_last_sustain_ms = 0;
    piano_last_note = "-";
    const uint8_t mapped = pianoMappedCount();
    piano_status = "Configured " + String(mapped) + "/8";
  } else {
    emu_config_stage = EmuConfigStage::None;
    emu_show_menu = false;
  }

  drawScreen();
}

void startCurrentEmulation() {
  if (!nfc_ready) return;

  const uint8_t type_idx = static_cast<uint8_t>(emu_type);
  if (!emu_dump_restore_checked[type_idx]) {
    emu_dump_restore_checked[type_idx] = true;
    const String saved_path = loadLastDumpForType(emu_type);
    if (!saved_path.isEmpty() && saved_path != emu_dump_loaded_path[type_idx] && littlefs_ready && LittleFS.exists(saved_path)) {
      applyDumpToCurrentType(saved_path, false, true, true);
    }
  }

  // Delegate to NFC worker on Core 0
  nfc_w_emu_type = emu_type;
  Serial.printf("[EMU] Start request type=%s\n", emuName(emu_type));
  if (sendNfcCmdAndWait(NfcCmd::StartEmulation, 3000)) {
    emu_started = nfc_cmd_result.ok;
    Serial.printf("[EMU] Start result type=%s ok=%d\n", emuName(emu_type), (int)emu_started);
  } else {
    emu_started = false;
    Serial.printf("[EMU] Start timeout type=%s\n", emuName(emu_type));
  }
  emu_last_start_retry_ms = millis();
  drawScreen();
}

void switchEmuType(int8_t dir = 1) {
  if (dir == 0) dir = 1;
  const EmuType from = emu_type;
  const EmuType to = emuTypeWithOffset(emu_type, dir > 0 ? 1 : -1);
  emu_type = to;
  updateEmulatorSourceStatus();
  emu_type_anim_from = from;
  emu_type_anim_to = to;
  emu_type_anim_dir = (dir > 0) ? 1 : -1;
  emu_type_anim_start_ms = millis();
  emu_type_anim_active = (from != to);
  last_emu_anim_ms = 0;
  emu_user_stopped = false;
  emu_switch_apply_pending = nfc_ready;
  if (emu_switch_apply_pending) {
    emu_started = false;
  } else {
    emu_started = false;
  }
  drawScreen();
}

void selectEmuType(EmuType type) {
  if (!isEmuTypeSupportedCurrentMode(type) || type == emu_type) return;
  const EmuType from = emu_type;
  emu_type = type;
  updateEmulatorSourceStatus();
  emu_type_anim_from = from;
  emu_type_anim_to = type;
  emu_type_anim_active = false;
  emu_user_stopped = false;
  emu_started = false;
  emu_switch_apply_pending = nfc_ready;
  drawScreen();
}

void autoScanNdef() {
  if (in_home || menu_page != MenuPage::ReadNDEF || wifi_popup || !nfc_ready) return;

  const uint32_t now = millis();
  if (now - last_ndef_auto_ms < kNdefAutoPollMs) return;
  last_ndef_auto_ms = now;

  String detail;
  String text;
  const bool ok = nfc.readNdef(text, detail);

  if (!ok) {
    ndef_fail_count++;
    if (detail.indexOf("short") >= 0 || detail.indexOf("invalid") >= 0 || detail.indexOf("error") >= 0 ||
        detail.indexOf("fail") >= 0 || detail.indexOf("timeout") >= 0 || ndef_fail_count >= 3) {
      recoverNfc("NDEF read abnormal", true);
      ndef_fail_count = 0;
    }
    if (!ndef_text.isEmpty() || ndef_detail != detail) {
      ndef_text = "";
      ndef_detail = detail;
      ndef_is_wifi = false;
      wifi_status = "";
      drawScreen();
    }
    return;
  }

  const bool changed = (text != ndef_text);
  ndef_fail_count = 0;
  ndef_text = text;
  ndef_detail = detail;
  String auth;
  ndef_is_wifi = parseWifiNdef(ndef_text, wifi_ssid, wifi_pass, auth);
  wifi_type = auth;

  if (changed) {
    if (ndef_is_wifi) wifi_popup = true;
    playNdefTone(ndef_is_wifi);
    drawScreen();
    Serial.printf("[NDEF] %s\n", ndef_text.c_str());
  }
}

void runDiagnose() {
  if (!nfc_ready) {
    diagnose_ok = false;
    diagnose_report = "I2C/NFC not ready";
    drawScreen();
    return;
  }

  if (sendNfcCmdAndWait(NfcCmd::RunDiagnose, 5000)) {
    diagnose_ok = nfc_cmd_result.ok;
    diagnose_report = normalizeReportNewlines(nfc_cmd_result.report);
    hw_ver = nfc_cmd_result.hw;
    fw_ver = nfc_cmd_result.fw;
  } else {
    diagnose_ok = false;
    diagnose_report = "Diagnose timeout";
  }
  Serial.printf("[DIAG] %s\n%s\n", diagnose_ok ? "PASS" : "FAIL", diagnose_report.c_str());
  drawScreen();
}

void runBootDebugFlow() {
  if (!kAutoBootDebug) return;

  boot_debug_running = true;
  in_home = false;
  menu_page = MenuPage::Diagnose;
  diagnose_ok = false;
  diagnose_report = "Boot debug running...";

#ifndef APP_TARGET_M5PAPER
  auto& d = M5.Display;
  d.fillScreen(TFT_BLACK);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  const int line_h = d.fontHeight() + 2;
  int row = 0;
  int y;

  auto drawLine = [&](const String& text, uint16_t color, uint16_t wait_ms) {
    y = 2 + row * line_h;
    d.setTextColor(color, TFT_BLACK);
    d.setCursor(2, y);
    d.print(text);
    delay(wait_ms);
    row++;
  };

  drawLine(String("[BOOT] ") + nfc_module_name, TFT_GREEN, 150);
  if (!nfc_ready) {
    drawLine("[FAIL] NFC not ready", TFT_RED, 1500);
    boot_notice_line = "NFC FAIL";
    boot_debug_running = false;
    goHome();
    return;
  }

  drawLine("[OK] NFC online", TFT_GREEN, 120);
  {
    String v = "[VER] HW=0x" + String(hw_ver, HEX) + " FW=0x" + String(fw_ver, HEX);
    v.toUpperCase();
    drawLine(v, TFT_WHITE, 120);
  }

  diagnose_ok = nfc.selfCheck(diagnose_report);
  diagnose_report = normalizeReportNewlines(diagnose_report);
  hw_ver = nfc.hardwareVersion();
  fw_ver = nfc.firmwareVersion();
  Serial.printf("[BOOT] DIAG: %s\n%s\n", diagnose_ok ? "PASS" : "FAIL", diagnose_report.c_str());
  drawLine(diagnose_ok ? "[OK] Self-check pass" : "[FAIL] Self-check fail",
           diagnose_ok ? TFT_GREEN : TFT_RED, 180);

  drawLine("[BOOT] Ready", TFT_YELLOW, 1000);

  diagnose_report = diagnose_ok ? "Boot check done" : "Boot check fail";
  boot_notice_line = diagnose_ok ? "" : "DIAG FAIL";
  boot_debug_running = false;
  goHome();
  Serial.println("[BOOT] Auto debug done");
#endif
}

String getFieldValue(const String& source, const String& key) {
  const int key_pos = source.indexOf(key);
  if (key_pos < 0) return "";
  const int start = key_pos + key.length();
  int end = source.indexOf(';', start);
  if (end < 0) end = source.length();
  String value = source.substring(start, end);
  value.replace("\\;", ";");
  value.replace("\\,", ",");
  value.replace("\\:", ":");
  return value;
}

bool parseWifiNdef(const String& input, String& ssid, String& pass, String& auth) {
  int pos = input.indexOf("WIFI:");
  if (pos < 0) pos = input.indexOf("wifi:");
  if (pos < 0) return false;

  String wifi = input.substring(pos);
  ssid = getFieldValue(wifi, "S:");
  pass = getFieldValue(wifi, "P:");
  auth = getFieldValue(wifi, "T:");
  return !ssid.isEmpty();
}

void scanNdefNow() {
  if (!nfc_ready) {
    ndef_text = "";
    ndef_detail = "NFC not ready";
    ndef_is_wifi = false;
    drawScreen();
    return;
  }

  if (!sendNfcCmdAndWait(NfcCmd::ScanNdefNow, 3000)) {
    ndef_text = "";
    ndef_detail = "Read timeout";
    ndef_is_wifi = false;
    wifi_status = "";
    drawScreen();
    return;
  }

  if (!nfc_cmd_result.ok) {
    String detail = nfc_cmd_result.report;
    ndef_fail_count++;
    if (detail.indexOf("short") >= 0 || detail.indexOf("invalid") >= 0 || detail.indexOf("error") >= 0 ||
        detail.indexOf("fail") >= 0 || detail.indexOf("timeout") >= 0 || ndef_fail_count >= 2) {
      sendNfcCmdAndWait(NfcCmd::Recover, 2000);
      ndef_fail_count = 0;
    }
    ndef_text = "";
    ndef_detail = detail;
    ndef_is_wifi = false;
    wifi_status = "";
    drawScreen();
    return;
  }

  // Parse packed result: text + '\x01' + detail
  String packed = nfc_cmd_result.report;
  int sep = packed.indexOf('\x01');
  String text = (sep >= 0) ? packed.substring(0, sep) : packed;
  String detail = (sep >= 0) ? packed.substring(sep + 1) : "";
  ndef_text = text;
  ndef_fail_count = 0;
  ndef_detail = detail;
  wifi_status = "";
  String auth;
  ndef_is_wifi = parseWifiNdef(ndef_text, wifi_ssid, wifi_pass, auth);
  wifi_type = auth;
  if (ndef_is_wifi) {
    wifi_popup = true;
  }
  playNdefTone(ndef_is_wifi);
  drawScreen();
  Serial.printf("[NDEF] %s\n", ndef_text.c_str());
}

void connectWifiNow() {
  if (emu_ap_active) {
    stopEmuApPortal();
  }
  wifi_popup = false;
  wifi_status = "Connecting...";
  drawScreen();

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_status = "OK: " + WiFi.localIP().toString();
  } else {
    wifi_status = "Connect failed";
    WiFi.disconnect(true, true);
  }
  drawScreen();
}

void emitHeartbeat() {
  const uint32_t now = millis();
  if (now - last_heartbeat_ms < kHeartbeatMs) return;
  last_heartbeat_ms = now;
}

void maintainNfcConnection() {
  const uint32_t now = millis();

  if (nfc_ready) {
    if (now - last_nfc_health_ms < kNfcHealthCheckMs) return;
    last_nfc_health_ms = now;

    if (!nfc.ping()) {
      nfc_ready = false;
      stopAllModes();
      last_card.valid = false;
      last_card.protocol = "None";
      last_card.uid = "";
      last_card.detail = "NFC disconnected";
      Serial.println("[NFC] Lost connection, waiting reconnect");
      drawScreen();
    } else if (menu_page == MenuPage::Reader && now - last_reader_success_ms > kReaderRecoverMs) {
      recoverNfc(nullptr, true);
    }
    return;
  }

  if (now - last_nfc_reconnect_ms < kNfcReconnectMs) return;
  last_nfc_reconnect_ms = now;

  if (nfc.begin()) {
    nfc_ready = true;
    hw_ver = nfc.hardwareVersion();
    fw_ver = nfc.firmwareVersion();
    last_nfc_health_ms = now;
    if (menu_page == MenuPage::Reader) {
      last_card.valid = false;
      last_card.protocol = "None";
      last_card.uid = "";
      last_card.detail = "Waiting card...";
        reader_need_first_tone = true;
    }
    Serial.printf("[NFC] Reconnected HW=0x%04X FW=0x%04X\n", hw_ver, fw_ver);
    drawScreen();
  }
}

void handleReader() {
  if (!nfc_ready) return;
  const uint32_t now = millis();
  uint32_t poll_interval = last_card.valid ? kReaderHoldCheckMs : kPollIntervalMs;
  if (!last_card.valid && !in_home && menu_page == MenuPage::Reader && poll_interval < 220) {
    // 无卡扫描动画期间降低读卡调用频率，减少I2C阻塞导致的动画顿挫
    poll_interval = 220;
  }
  if (now - last_poll_ms < poll_interval) return;
  last_poll_ms = now;

  CardInfo card;
  const bool got_card = reader_14b_only ? nfc.readOnlyISO14B(card) : nfc.readAny(card);
  if (got_card) {
    reader_fail_streak = 0;
    last_reader_success_ms = now;
    if (card.uid != last_card.uid || card.protocol != last_card.protocol || !last_card.valid) {
      last_card = card;
      playCardTone(card.protocol);
      drawScreen();
      Serial.println(formatCardLogLine(card));
      reader_last_hold_log_ms = now;
    } else if (reader_14b_only && card.protocol == "ISO14443B" && now - reader_last_hold_log_ms >= 2000) {
      Serial.printf("[CARD][14B][HOLD] UID=%s\n", card.uid.c_str());
      reader_last_hold_log_ms = now;
    }
  } else {
    if (reader_fail_streak < 0xFF) ++reader_fail_streak;
    if (reader_14b_only && reader_fail_streak >= 8) {
      Serial.println("[READER] 14B fail streak -> auto recover");
      recoverNfc("Reader 14B fail streak", true);
      reader_fail_streak = 0;
      last_poll_ms = 0;
      return;
    }
    if (last_card.valid && now - last_reader_success_ms > 1200) {
      recoverNfc("Reader stuck after tag", true);
    }
    card.valid = false;
    card.protocol = "None";
    card.uid = "";
    card.detail = "No card";
    if (last_card.valid || last_card.detail != card.detail || last_card.protocol != card.protocol) {
      last_card = card;
      drawScreen();
    }
  }
}

void handlePiano() {
  if (!nfc_ready) return;
  if (piano_stage == PianoStage::Menu) return;

  const uint32_t now = millis();
  if (now - last_poll_ms < kPianoPollMs) return;
  last_poll_ms = now;

  CardInfo card;
  if (!nfc.readAny(card)) {
    const int8_t old_note = piano_active_note_idx;
    piano_active_card_key = "";
    piano_active_note_idx = -1;
    piano_last_sustain_ms = 0;
    if (piano_stage == PianoStage::Play) {
      piano_last_note = "-";
      piano_status = "Tap card to play";
      drawPianoPlayDiff(old_note, piano_active_note_idx, true);
    }
    return;
  }

  const String card_key = buildPianoCardKey(card);
  if (card_key.isEmpty()) return;

  if (piano_stage == PianoStage::Config) {
    if (card_key == piano_active_card_key) return;
    piano_active_card_key = card_key;

    for (uint8_t i = 0; i < piano_config_step; ++i) {
      if (piano_card_map[i] == card_key) {
        piano_status = "Card already used";
        drawScreen();
        return;
      }
    }

    piano_card_map[piano_config_step] = card_key;
    playTone(kPianoFreq[piano_config_step], 120);
    piano_last_note = kPianoNoteName[piano_config_step];
    ++piano_config_step;

    if (piano_config_step >= kPianoNoteCount) {
      savePianoConfig();
      piano_stage = PianoStage::Menu;
      piano_menu_index = 0;
      piano_status = "Config saved 8/8";
    } else {
      piano_status = String("Saved ") + String(piano_config_step) + "/8";
    }
    drawScreen();
    return;
  }

  const int8_t note_idx = findPianoNoteByCard(card_key);
  const bool same_card = (card_key == piano_active_card_key);
  piano_active_card_key = card_key;

  if (note_idx < 0) {
    const int8_t old_note = piano_active_note_idx;
    piano_last_note = "-";
    piano_active_note_idx = -1;
    piano_status = "Card not mapped";
    drawPianoPlayDiff(old_note, piano_active_note_idx, true);
    return;
  }

  const int8_t old_note = piano_active_note_idx;
  const bool new_note = (note_idx != piano_active_note_idx);
  const bool need_retrigger = !same_card || new_note || (now - piano_last_sustain_ms >= kPianoSustainRetriggerMs);
  if (need_retrigger) {
    playTone(kPianoFreq[note_idx], kPianoSustainToneMs);
    piano_last_sustain_ms = now;
  }

  const bool note_changed = (piano_active_note_idx != note_idx) || (piano_last_note != kPianoNoteName[note_idx]);
  piano_active_note_idx = note_idx;
  piano_last_note = kPianoNoteName[note_idx];
  piano_status = "Play " + String(kPianoNoteName[note_idx]);
  if (note_changed) {
    drawPianoPlayDiff(old_note, piano_active_note_idx, true);
  }
}

// ---- NFC Worker Task (runs on Core 0) ----
// All I2C/NFC blocking calls happen here. Results are written to
// shared result structs and consumed by the UI loop on Core 1.
void nfcWorkerTask(void* /*param*/) {
  uint32_t w_last_poll_ms = 0;
  uint32_t w_last_health_ms = 0;
  uint32_t w_last_reconnect_ms = 0;
  uint32_t w_last_ndef_ms = 0;
  uint32_t w_last_piano_ms = 0;
  uint32_t w_last_reader_success_ms = millis();
  uint32_t w_last_recover_ms = 0;
  uint8_t w_reader_fail_streak = 0;
  uint8_t w_ndef_fail_count = 0;

  for (;;) {
    // --- Process one-shot commands from UI (non-blocking) ---
    NfcCmd cmd = NfcCmd::None;
    if (xQueueReceive(nfc_cmd_queue, &cmd, 0) == pdTRUE) {
      if (xSemaphoreTake(nfc_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        NfcCmdResult res;
        switch (cmd) {
          case NfcCmd::StartEmulation: {
            auto startForType = [&](EmuType et) -> bool {
              if (isNfcUnitMode()) {
                // Unit NFC supports NTAG213/215/216 via EmulationLayerA, FeliCa via EmulationLayerF
                switch (et) {
                  case EmuType::N213:  return nfc.startNfcUnitEmulationNtag(213);
                  case EmuType::N215:  return nfc.startNfcUnitEmulationNtag(215);
                  case EmuType::N216:  return nfc.startNfcUnitEmulationNtag(216);
                  case EmuType::Felica:return nfc.startNfcUnitEmulationFelica();
                  default:             return false;  // MFC, ISO14B, ISO15 not supported on Unit
                }
              }
              switch (et) {
                case EmuType::MF1K:   return nfc.startEmulationMifare1K();
                case EmuType::MF4K:   return nfc.startEmulationMifare4K();
                case EmuType::N213:   return nfc.startEmulationNtag213();
                case EmuType::N215:   return nfc.startEmulationNtag215();
                case EmuType::N216:   return nfc.startEmulationNtag216();
                case EmuType::ISO14B: return nfc.startEmulationChinaII();
                case EmuType::Felica: return nfc.startEmulationFelica();
                case EmuType::ISO15:  return nfc.startEmulationISO15();
                default:              return false;
              }
            };

            if (nfc_ready) {
              // Restore the 2026-07-04 Unit NFC lifecycle that is confirmed to
              // emulate NTAG213 on StickS3: recover reader state, explicitly
              // end emulation/RF, wait for the field to collapse, then start A.
              nfc.recover();
              if (isNfcUnitMode()) nfc.stopNfcUnitEmulation();
              else nfc.stopRF();
              delay(50);
              EmuType et = static_cast<EmuType>(nfc_w_emu_type);
              bool ok = startForType(et);
              if (!ok) {
                // First-start races are observed on some Unit NFC fw/host combos.
                bool recovered = nfc.recover();
                if (recovered) recovered = nfc.begin();
                if (recovered) {
                  ok = startForType(et);
                  if (!ok) {
                    // Leave module clean after retry failure so the next
                    // StartEmulation (e.g. after user switches type) begins
                    // from a known state rather than a stale/corrupted mode.
                    nfc.recover();
                  }
                } else {
                  nfc_ready = false;
                }
              }
              res.ok = ok;
            }
            res.done = true;
            nfc_cmd_result = res;
            break;
          }
          case NfcCmd::StopRF: {
            if (isNfcUnitMode()) {
              nfc.stopNfcUnitEmulation();
            } else {
              nfc.stopRF();
            }
            res.ok = true;
            res.done = true;
            nfc_cmd_result = res;
            break;
          }
          case NfcCmd::RunDiagnose: {
            if (nfc_ready) {
              String report;
              res.ok = nfc.selfCheck(report);
              res.report = report;
              res.hw = nfc.hardwareVersion();
              res.fw = nfc.firmwareVersion();
            }
            res.done = true;
            nfc_cmd_result = res;
            break;
          }
          case NfcCmd::ScanNdefNow: {
            if (nfc_ready) {
              String text, detail;
              res.ok = nfc.readNdef(text, detail);
              res.report = res.ok ? text : detail;
              // Pack both into report with separator
              if (res.ok) {
                res.report = text + "\x01" + detail;
              } else {
                res.report = detail;
              }
            }
            res.done = true;
            nfc_cmd_result = res;
            break;
          }
          case NfcCmd::Recover: {
            if (nfc_ready) {
              bool ok = nfc.recover();
              if (ok) ok = nfc.begin();
              if (!ok) {
                nfc_ready = false;
                res.ok = false;
              } else {
                w_last_reader_success_ms = millis();
                res.ok = true;
              }
            }
            res.done = true;
            nfc_cmd_result = res;
            break;
          }
          default:
            break;
        }
        xSemaphoreGive(nfc_mutex);
      }
    }

    const uint32_t now = millis();
    const bool w_ready = nfc_ready;
    const bool w_in_home = nfc_w_in_home;
    const bool nfc_unit_emulating = (isNfcUnitMode() && nfc.isNfcUnitEmulating());

    // --- maintainNfcConnection (health check / reconnect) ---
    // IMPORTANT: while Unit NFC emulation is active, skip ping/recover checks.
    // ping() may reconfigure mode and can disrupt the emulation RF state.
    if (!nfc_unit_emulating && xSemaphoreTake(nfc_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (w_ready) {
        if (now - w_last_health_ms >= kNfcHealthCheckMs) {
          w_last_health_ms = now;
          if (!nfc.ping()) {
            nfc_ready = false;
            nfc.stopRF();
            NfcHealthResult hr;
            hr.lost_connection = true;
            hr.new_result = true;
            nfc_health_result = hr;
            Serial.println("[NFC-W] Lost connection");
          } else if (nfc_w_is_reader_page && now - w_last_reader_success_ms > kReaderRecoverMs) {
            if (now - w_last_recover_ms >= kRecoverCooldownMs) {
              w_last_recover_ms = now;
              bool ok = nfc.recover();
              if (ok) ok = nfc.begin();
              if (!ok) {
                nfc_ready = false;
                NfcHealthResult hr;
                hr.lost_connection = true;
                hr.new_result = true;
                nfc_health_result = hr;
              } else {
                w_last_reader_success_ms = now;
              }
            }
          }
        }
      } else {
        if (now - w_last_reconnect_ms >= kNfcReconnectMs) {
          w_last_reconnect_ms = now;
          if (nfc.begin()) {
            nfc_ready = true;
            NfcHealthResult hr;
            hr.reconnected = true;
            hr.new_hw_ver = nfc.hardwareVersion();
            hr.new_fw_ver = nfc.firmwareVersion();
            hr.new_result = true;
            nfc_health_result = hr;
            w_last_health_ms = now;
            Serial.printf("[NFC-W] Reconnected HW=0x%04X FW=0x%04X\n", hr.new_hw_ver, hr.new_fw_ver);
          }
        }
      }
      xSemaphoreGive(nfc_mutex);
    }

    // --- handleReader (periodic card poll) ---
    if (nfc_ready && !w_in_home && nfc_w_is_reader_page) {
      const uint32_t poll_iv = nfc_w_card_valid ? kReaderHoldCheckMs : kPollIntervalMs;
      if (now - w_last_poll_ms >= poll_iv) {
        w_last_poll_ms = now;
        if (xSemaphoreTake(nfc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          CardInfo card;
          const bool got = nfc_w_reader_14b_only ? nfc.readOnlyISO14B(card) : nfc.readAny(card);
          NfcReaderResult rr;
          rr.card = card;
          rr.got_card = got;
          rr.new_result = true;
          nfc_reader_result = rr;
          if (got) {
            w_reader_fail_streak = 0;
            w_last_reader_success_ms = now;
          } else {
            if (w_reader_fail_streak < 0xFF) ++w_reader_fail_streak;
            // Unit NFC can occasionally get into a scan-stall state on CardPuter-class boards.
            // If we keep polling with no hit for a while, proactively recover + re-begin
            // so users don't need to leave/re-enter Reader page.
            if (isNfcUnitMode() && !nfc_w_reader_14b_only && w_reader_fail_streak >= 12) {
              if (now - w_last_recover_ms >= kRecoverCooldownMs) {
                w_last_recover_ms = now;
                bool ok = nfc.recover();
                if (ok) ok = nfc.begin();
                if (!ok) {
                  nfc_ready = false;
                }
                w_reader_fail_streak = 0;
                w_last_poll_ms = 0;
              }
            }
            if (nfc_w_reader_14b_only && w_reader_fail_streak >= 8) {
              if (now - w_last_recover_ms >= kRecoverCooldownMs) {
                w_last_recover_ms = now;
                bool ok = nfc.recover();
                if (ok) ok = nfc.begin();
                if (!ok) nfc_ready = false;
                w_reader_fail_streak = 0;
                w_last_poll_ms = 0;
              }
            }
            if (nfc_w_card_valid && now - w_last_reader_success_ms > 1200) {
              if (now - w_last_recover_ms >= kRecoverCooldownMs) {
                w_last_recover_ms = now;
                bool ok = nfc.recover();
                if (ok) ok = nfc.begin();
                if (!ok) nfc_ready = false;
              }
            }
          }
          xSemaphoreGive(nfc_mutex);
        }
      }
    }

    // --- autoScanNdef (periodic NDEF poll) ---
    if (nfc_ready && !w_in_home && nfc_w_is_ndef_page && !nfc_w_wifi_popup) {
      if (now - w_last_ndef_ms >= kNdefAutoPollMs) {
        w_last_ndef_ms = now;
        if (xSemaphoreTake(nfc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          String text, detail;
          bool ok = nfc.readNdef(text, detail);
          NfcNdefResult nr;
          nr.text = text;
          nr.detail = detail;
          nr.ok = ok;
          nr.new_result = true;
          nfc_ndef_result = nr;
          if (!ok) {
            w_ndef_fail_count++;
            if (detail.indexOf("short") >= 0 || detail.indexOf("invalid") >= 0 || detail.indexOf("error") >= 0 ||
                detail.indexOf("fail") >= 0 || detail.indexOf("timeout") >= 0 || w_ndef_fail_count >= 3) {
              if (now - w_last_recover_ms >= kRecoverCooldownMs) {
                w_last_recover_ms = now;
                bool rok = nfc.recover();
                if (rok) rok = nfc.begin();
                if (!rok) nfc_ready = false;
                w_ndef_fail_count = 0;
              }
            }
          } else {
            w_ndef_fail_count = 0;
          }
          xSemaphoreGive(nfc_mutex);
        }
      }
    }

    // --- handlePiano (periodic card poll for piano) ---
    if (nfc_ready && !w_in_home && nfc_w_is_piano_page && nfc_w_piano_stage != PianoStage::Menu) {
      if (now - w_last_piano_ms >= kPianoPollMs) {
        w_last_piano_ms = now;
        if (xSemaphoreTake(nfc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          CardInfo card;
          bool got = nfc.readAny(card);
          NfcPianoResult pr;
          pr.card = card;
          pr.got_card = got;
          pr.new_result = true;
          nfc_piano_result = pr;
          xSemaphoreGive(nfc_mutex);
        }
      }
    }

    // --- Unit NFC emulation state machine update ---
    // emu_a.update() must be called continuously while emulating.
    // This section runs every task tick (5ms) regardless of current page.
    if (nfc_ready && isNfcUnitMode() && nfc.isNfcUnitEmulating()) {
      // Keep emulation update cadence as high as possible on Unit NFC.
      // Avoid mutex waits here: this code already runs inside the single NFC worker task.
      nfc.tickNfcUnitEmulation();
      // Reader commands arrive immediately after anticollision/selection.
      // A timed delay here makes Android/Flipper GET_VERSION and READ frames
      // intermittently miss their response window. Yield without sleeping so
      // the NFC state machine can be serviced again as soon as Core 0 runs.
      taskYIELD();
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));  // yield to other tasks
    }
  }
}

// Helper: send command to NFC worker and wait for result
bool sendNfcCmdAndWait(NfcCmd cmd, uint32_t timeout_ms) {
  nfc_cmd_result.done = false;
  xQueueSend(nfc_cmd_queue, &cmd, pdMS_TO_TICKS(100));
  const uint32_t start = millis();
  while (!nfc_cmd_result.done) {
    if (millis() - start > timeout_ms) return false;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return true;
}

// UI-side: process reader result from NFC worker
void processReaderResult() {
  if (!nfc_reader_result.new_result) return;
  // Copy result
  CardInfo card = nfc_reader_result.card;
  bool got_card = nfc_reader_result.got_card;
  nfc_reader_result.new_result = false;

  if (got_card) {
    reader_fail_streak = 0;
    last_reader_success_ms = millis();
    if (card.uid != last_card.uid || card.protocol != last_card.protocol || !last_card.valid) {
      last_card = card;
      playCardTone(card.protocol);
      drawScreen();
      Serial.println(formatCardLogLine(card));
      reader_last_hold_log_ms = millis();
    } else if (reader_14b_only && card.protocol == "ISO14443B" && millis() - reader_last_hold_log_ms >= 2000) {
      Serial.printf("[CARD][14B][HOLD] UID=%s\n", card.uid.c_str());
      reader_last_hold_log_ms = millis();
    }
  } else {
    if (reader_fail_streak < 0xFF) ++reader_fail_streak;
    CardInfo nc;
    nc.valid = false;
    nc.protocol = "None";
    nc.uid = "";
    nc.detail = "No card";
    if (last_card.valid || last_card.detail != nc.detail || last_card.protocol != nc.protocol) {
      last_card = nc;
      drawScreen();
    }
  }
  // Update shared flag for worker
  nfc_w_card_valid = last_card.valid;
}

// UI-side: process NDEF auto-scan result from NFC worker
void processNdefResult() {
  if (!nfc_ndef_result.new_result) return;
  String text = nfc_ndef_result.text;
  String detail = nfc_ndef_result.detail;
  bool ok = nfc_ndef_result.ok;
  nfc_ndef_result.new_result = false;

  if (!ok) {
    if (!ndef_text.isEmpty() || ndef_detail != detail) {
      ndef_text = "";
      ndef_detail = detail;
      ndef_is_wifi = false;
      wifi_status = "";
      drawScreen();
    }
    return;
  }

  const bool changed = (text != ndef_text);
  ndef_text = text;
  ndef_detail = detail;
  String auth;
  ndef_is_wifi = parseWifiNdef(ndef_text, wifi_ssid, wifi_pass, auth);
  wifi_type = auth;

  if (changed) {
    if (ndef_is_wifi) wifi_popup = true;
    playNdefTone(ndef_is_wifi);
    drawScreen();
    Serial.printf("[NDEF] %s\n", ndef_text.c_str());
  }
}

// UI-side: process health check result from NFC worker
void processHealthResult() {
  if (!nfc_health_result.new_result) return;
  bool lost = nfc_health_result.lost_connection;
  bool reconnected = nfc_health_result.reconnected;
  uint16_t nhw = nfc_health_result.new_hw_ver;
  uint16_t nfw = nfc_health_result.new_fw_ver;
  nfc_health_result.new_result = false;

  if (lost) {
    home_anim_active = false;
    emu_started = false;
    last_card.valid = false;
    last_card.protocol = "None";
    last_card.uid = "";
    last_card.detail = "NFC disconnected";
    Serial.println("[NFC] Lost connection");
    drawScreen();
  }
  if (reconnected) {
    home_anim_active = false;
    hw_ver = nhw;
    fw_ver = nfw;
    nfc_module_name = nfc.deviceName();
    // Clamp home_index in case it exceeds the new page count (e.g. switching from Grove to Unit NFC)
    if (home_index >= homePageCount()) home_index = 0;
    if (menu_page == MenuPage::Reader) {
      last_card.valid = false;
      last_card.protocol = "None";
      last_card.uid = "";
      last_card.detail = "Waiting card...";
      reader_need_first_tone = true;
    }
    drawScreen();
  }
}

// UI-side: process piano result from NFC worker
void processPianoResult() {
  if (!nfc_piano_result.new_result) return;
  CardInfo card = nfc_piano_result.card;
  bool got_card = nfc_piano_result.got_card;
  nfc_piano_result.new_result = false;

  const uint32_t now = millis();

  if (!got_card) {
    const int8_t old_note = piano_active_note_idx;
    piano_active_card_key = "";
    piano_active_note_idx = -1;
    piano_last_sustain_ms = 0;
    if (piano_stage == PianoStage::Play) {
      piano_last_note = "-";
      piano_status = "Tap card to play";
      drawPianoPlayDiff(old_note, piano_active_note_idx, true);
    }
    return;
  }

  const String card_key = buildPianoCardKey(card);
  if (card_key.isEmpty()) return;

  if (piano_stage == PianoStage::Config) {
    if (card_key == piano_active_card_key) return;
    piano_active_card_key = card_key;

    for (uint8_t i = 0; i < piano_config_step; ++i) {
      if (piano_card_map[i] == card_key) {
        piano_status = "Card already used";
        drawScreen();
        return;
      }
    }

    piano_card_map[piano_config_step] = card_key;
    playTone(kPianoFreq[piano_config_step], 120);
    piano_last_note = kPianoNoteName[piano_config_step];
    ++piano_config_step;

    if (piano_config_step >= kPianoNoteCount) {
      savePianoConfig();
      piano_stage = PianoStage::Menu;
      piano_menu_index = 0;
      piano_status = "Config saved 8/8";
    } else {
      piano_status = String("Saved ") + String(piano_config_step) + "/8";
    }
    drawScreen();
    return;
  }

  const int8_t note_idx = findPianoNoteByCard(card_key);
  const bool same_card = (card_key == piano_active_card_key);
  piano_active_card_key = card_key;

  if (note_idx < 0) {
    const int8_t old_note = piano_active_note_idx;
    piano_last_note = "-";
    piano_active_note_idx = -1;
    piano_status = "Card not mapped";
    drawPianoPlayDiff(old_note, piano_active_note_idx, true);
    return;
  }

  const int8_t old_note = piano_active_note_idx;
  const bool new_note = (note_idx != piano_active_note_idx);
  const bool need_retrigger = !same_card || new_note || (now - piano_last_sustain_ms >= kPianoSustainRetriggerMs);
  if (need_retrigger) {
    playTone(kPianoFreq[note_idx], kPianoSustainToneMs);
    piano_last_sustain_ms = now;
  }

  const bool note_changed = (piano_active_note_idx != note_idx) || (piano_last_note != kPianoNoteName[note_idx]);
  piano_active_note_idx = note_idx;
  piano_last_note = kPianoNoteName[note_idx];
  piano_status = "Play " + String(kPianoNoteName[note_idx]);
  if (note_changed) {
    drawPianoPlayDiff(old_note, piano_active_note_idx, true);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
#if defined(APP_TARGET_M5PAPER)
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(0);
  const int sw = M5.Display.width();
  const int sh = M5.Display.height();

  // A 540x960 16-bit frame is about 1 MiB, so it must live in PSRAM.
  // LGFX sprites default to internal DMA RAM unless this is requested.
  g_canvas.setPsram(true);
  g_canvas.setColorDepth(16);
  s_epd_ready = g_canvas.createSprite(sw, sh) != nullptr;
  Serial.printf("[BOOT] PSRAM=%u free=%u canvas=%s (%dx%d)\n",
                ESP.getPsramSize(),
                ESP.getFreePsram(),
                s_epd_ready ? "OK" : "FAIL",
                sw,
                sh);
  if (!s_epd_ready) {
    Serial.println("[BOOT] M5Paper canvas allocation failed");
  }

  // Set home state before drawing
  in_home = true;
  home_index = -1;
  menu_page = MenuPage::Reader;

  // Render home grid directly into g_canvas.
  // This mirrors the minimal test exactly (no helper layers) so it is
  // guaranteed to use the same push+display path that we know works.
  g_canvas.fillScreen(TFT_WHITE);
  // Status bar
  g_canvas.setFont(&fonts::Font0);
  g_canvas.setTextSize(2);
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  g_canvas.setCursor(4, 5);
  g_canvas.print("GroveNFC");
  g_canvas.drawLine(0, kStatusBarHeight, sw, kStatusBarHeight, TFT_BLACK);
  // Tiles
  for (int i = 0; i < homePageCount(); ++i) {
    const auto tile = getM5PaperHomeTile(i, sw, sh);
    const MenuPage pg = homePageAt(i);
    g_canvas.drawRoundRect(tile.x, tile.y, tile.w, tile.h, 8, TFT_BLACK);
    drawM5PaperHomeIcon(tile.icon_cx, tile.icon_cy, pg, TFT_BLACK);
    g_canvas.setFont(&fonts::Font0);
    g_canvas.setTextSize(2);
    g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    const String tileName = String(homePageName(pg));
    const int lw = g_canvas.textWidth(tileName);
    g_canvas.setCursor(tile.icon_cx - lw / 2, tile.y + tile.h - 28);
    g_canvas.print(tileName);
  }
  // Hint at bottom
  g_canvas.setFont(&fonts::Font0);
  g_canvas.setTextSize(1);
  g_canvas.setTextColor(TFT_BLACK, TFT_WHITE);

  // Show a useful screen immediately.  Initialization continues below so the
  // NFC module and its worker task are actually available to every UI page.
  pushCanvasToEink(true);
#else
 auto cfg = M5.config();
#if defined(APP_TARGET_STICKCPLUS)
  // Plus SE has no IMU. GroveNFC does not use motion sensing, so disabling the
  // probe keeps this same firmware compatible with both Plus and Plus SE.
  cfg.internal_imu = false;
#if defined(APP_TARGET_STICKCPLUS_SE)
  // Plus SE has no IMU but retains the GPIO2 passive PWM buzzer.
  cfg.internal_mic = false;
  cfg.internal_spk = true;
#else
  cfg.internal_spk = true;
#endif
#endif
#if defined(APP_TARGET_STICKS3)
  cfg.internal_spk = true;
#endif
#if defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
  M5Cardputer.begin(cfg, true);
#else
  M5.begin(cfg);
#endif
  Serial.println("[BOOT] M5 init complete");
  Serial.flush();
#if defined(APP_TARGET_STICKS3) || defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
  M5.Power.setExtOutput(true);
#endif
  M5.Speaker.setVolume(kSpeakerVolume);
#endif

  const char* target_name = "Unknown";
#if defined(APP_TARGET_STICKS3)
  target_name = "M5StickS3";
#elif defined(APP_TARGET_STICKCPLUS)
  target_name = "M5StickCPlus";
#elif defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
  target_name = "CardPuter/CardPuterADV";
#elif defined(APP_TARGET_ATOMS3)
  target_name = "AtomS3";
#elif defined(APP_TARGET_M5PAPER)
  target_name = "M5Paper";
#endif
  Serial.printf("[BOOT] Target=%s SDA=GPIO%d SCL=GPIO%d I2C=0x%02X\n",
                target_name,
                active_sda_pin,
                active_scl_pin,
                grove_nfc::I2C_SLAVE_ADDR);
  Wire.begin(active_sda_pin, active_scl_pin, kI2CFreq);

  // Canvas is created in M5Paper-specific setup above (grayscale_8bit)
  // For other targets, create standard 16-bit canvas here
#if !defined(APP_TARGET_M5PAPER)
  M5.Display.setRotation(
  #if defined(APP_TARGET_STICKS3) || defined(APP_TARGET_STICKCPLUS) || defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
    1
  #else
    0
  #endif
    );
  M5.Display.setFont(&fonts::Font0);
  g_canvas.setColorDepth(16);
  g_canvas.createSprite(M5.Display.width(), M5.Display.height());
  Serial.printf("[BOOT] Canvas %dx%d created\n", M5.Display.width(), M5.Display.height());
  Serial.flush();
#endif
  littlefs_ready = LittleFS.begin(true);
  Serial.printf("[BOOT] LittleFS=%s\n", littlefs_ready ? "OK" : "FAIL");
  Serial.flush();
  if (!littlefs_ready) {
    Serial.println("[FS] LittleFS init failed");
    emu_dump_status = "LittleFS fail";
  } else if (!ensureDumpDirExists()) {
    Serial.println("[FS] /dumps mkdir failed");
  } else {
    removeLegacyExampleDumps();
  }
  loadPianoConfig();

  nfc_ready = initNfcAtBoot();
  Serial.printf("[BOOT] NFC init=%s\n", nfc_ready ? "OK" : "FAIL");
  Serial.flush();
  nfc_module_name = nfc_ready ? String(nfc.deviceName()) : String("GroveNFC");
  hw_ver = nfc.hardwareVersion();
  fw_ver = nfc.firmwareVersion();
  last_card.protocol = "None";
  last_card.detail = nfc_ready ? "Waiting card..." : "No NFC module";
  diagnose_report = "Press hold to run check";
  last_reader_success_ms = millis();

#if defined(APP_TARGET_M5PAPER)
  // The boot diagnostic is a temporary text page designed for small
  // button-driven displays. M5Pager already has a full touch home screen, so
  // never expose Diagnose during startup.
  drawScreen();
#else
  runBootDebugFlow();
  // runBootDebugFlow() calls goHome() → drawScreen() when kAutoBootDebug=true.
  // If boot debug is disabled, draw the initial screen here.
  if (!kAutoBootDebug) drawScreen();
#endif

  // Start NFC worker task on Core 0
  nfc_mutex = xSemaphoreCreateMutex();
  nfc_cmd_queue = xQueueCreate(4, sizeof(NfcCmd));
  // Sync initial UI state to worker
  nfc_w_in_home = in_home;
  nfc_w_card_valid = last_card.valid;
  xTaskCreatePinnedToCore(
    nfcWorkerTask,    // task function
    "nfc_worker",     // name
    4096,             // stack size
    nullptr,          // param
    1,                // priority
    &nfc_task_handle, // handle
    0                 // Core 0
  );
  Serial.println("[BOOT] NFC worker started on Core 0");

  // Keep NFC Unit in reader mode at boot. Entering Emulator starts NTAG213;
  // entering Reader explicitly restores reader mode first.

#ifdef APP_TARGET_M5PAPER
  // Refresh after NFC detection so the status bar reports the real module.
  goHome();
  Serial.println("[BOOT] Final M5Paper screen refresh done");
#endif
}


void loop() {

#if defined(APP_TARGET_CARDPUTER) || defined(APP_TARGET_CARDPUTER_ADV)
  static uint32_t s_cardputer_last_update_ms = 0;
  const bool nfcunit_active_page = (!in_home && isNfcUnitMode());
  const bool critical_nfcunit_emu = (nfcunit_active_page && menu_page == MenuPage::Emulator && emu_started);
  uint32_t update_interval_ms = 0;
  if (critical_nfcunit_emu) {
    update_interval_ms = 220;
  } else if (nfcunit_active_page) {
    update_interval_ms = 70;
  }

  if (update_interval_ms == 0 || millis() - s_cardputer_last_update_ms >= update_interval_ms) {
    M5Cardputer.update();
    s_cardputer_last_update_ms = millis();
  }
#else
  M5.update();
#endif

  const KeyNavState key_nav = readKeyNavState();
  const bool nav_prev = key_nav.prev || key_nav.key_b;
  const bool nav_back = key_nav.back;
  const bool clicked_raw = mainButtonClicked();
  const bool released = mainButtonReleased();
  bool clicked = clicked_raw;
  if (clicked && ignore_click_after_hold) {
    clicked = false;
    ignore_click_after_hold = false;
  } else if (released && ignore_click_after_hold) {
    ignore_click_after_hold = false;
  }
  if (released) {
    btn_hold_latched = false;
  }

#ifdef APP_TARGET_M5PAPER
  if (handleM5PaperStatusTouch()) {
    delay(10);
    return;
  }
  if (handleM5PaperFeatureTouch()) {
    delay(10);
    return;
  }
  if (in_home && handleM5PaperHomeTouch()) {
    delay(10);
    return;
  }
  static uint32_t s_last_battery_refresh_ms = 0;
  if (in_home && millis() - s_last_battery_refresh_ms >= 60000) {
    s_last_battery_refresh_ms = millis();
    drawScreen();
  }
#endif

  // Some Unit NFC firmware revisions occasionally fail the first emulation start.
  // Retry periodically while staying on Emulator page until emulation is active.
  if (!in_home && menu_page == MenuPage::Emulator && nfc_ready && !emu_user_stopped &&
      !emu_switch_apply_pending && !emu_started) {
    const uint32_t now_retry = millis();
    if (now_retry - emu_last_start_retry_ms >= 1000) {
      startCurrentEmulation();
    }
  }

  emitHeartbeat();
  handleEmuApPortal();
  // NFC I2C ops now run on Core 0 worker — just process results here
  processHealthResult();
  processNdefResult();

  // Sync UI state to NFC worker
  nfc_w_in_home = in_home;
  nfc_w_is_reader_page = (!in_home && menu_page == MenuPage::Reader);
  nfc_w_is_ndef_page = (!in_home && menu_page == MenuPage::ReadNDEF);
  nfc_w_is_piano_page = (!in_home && menu_page == MenuPage::Piano);
  nfc_w_reader_14b_only = reader_14b_only;
  nfc_w_card_valid = last_card.valid;
  nfc_w_wifi_popup = wifi_popup;
  nfc_w_piano_stage = piano_stage;

  const bool reader_anim_active =
#ifdef APP_TARGET_M5PAPER
      false;
#else
      (!in_home && menu_page == MenuPage::Reader && !last_card.valid);
#endif
  if (!in_home && (menu_page == MenuPage::Reader || menu_page == MenuPage::ReadNDEF)) {
    const bool anim_running = reader_anim_active;
    if (anim_running) {
#ifndef APP_TARGET_M5PAPER
      auto& d = M5.Display;
      const int w = d.width();
      const int h = d.height();
#if defined(APP_TARGET_STICKS3) || defined(APP_TARGET_STICKCPLUS) || defined(APP_TARGET_CARDPUTER)
      if (w > h) {
        const int header_h = 18;
        const int content_top = header_h + 2;
        const int content_h = h - content_top - 2;
        drawReaderPixelCard(d, 4, content_top + 2, w - 8, content_h - 4, TFT_GREEN, last_card, reader_14b_only, true);
      } else {
        drawReaderPixelCard(d, 4, 26, w - 8, h - 30, TFT_GREEN, last_card, reader_14b_only, true);
      }
#elif defined(APP_TARGET_ATOMS3)
      drawReaderPixelCard(d, 4, 26, w - 8, h - 30, TFT_GREEN, last_card, reader_14b_only, true);
#elif !defined(APP_TARGET_M5PAPER)
      drawReaderPixelCard(d, 4, 26, w - 8, h - 30, TFT_GREEN, last_card, reader_14b_only, true);
#endif
#endif
    } else {
      last_reader_anim_us = 0;
      if (ui_marquee_active) {
#ifndef APP_TARGET_M5PAPER
        const uint32_t now = millis();
        if (now - last_ui_scroll_ms >= kUiScrollMs) {
          last_ui_scroll_ms = now;
          drawScreen();
        }
#endif
      }
    }
  }

  if (!in_home && menu_page == MenuPage::Diagnose && diagnose_ok) {
#ifndef APP_TARGET_M5PAPER
    const uint32_t now = millis();
    if (now - last_diag_scroll_ms >= kDiagScrollMs) {
      last_diag_scroll_ms = now;
      drawScreen();
    }
#endif
  }

  if (!in_home && menu_page == MenuPage::Emulator && emu_config_stage == EmuConfigStage::None) {
    if (emu_type_anim_active) {
#ifdef APP_TARGET_M5PAPER
      emu_type_anim_active = false;
      drawScreen();
#else
      const uint32_t now = millis();
      if (last_emu_anim_ms == 0 || now - last_emu_anim_ms >= 16) {
        last_emu_anim_ms = now;
        drawScreen();
      }
#endif
    } else if (emu_switch_apply_pending) {
      emu_switch_apply_pending = false;
      startCurrentEmulation();
    } else {
      // Periodic redraw for UID marquee scrolling
#ifndef APP_TARGET_M5PAPER
      const uint32_t now = millis();
      if (now - last_ui_scroll_ms >= kUiScrollMs) {
        last_ui_scroll_ms = now;
        drawScreen();
      }
#endif
    }
  }

  if (in_home && home_anim_active) {
#ifdef APP_TARGET_M5PAPER
    home_anim_active = false;
    drawScreen();
#else
    const uint32_t now = millis();
    if (now - home_anim_start_ms >= home_anim_duration_ms) {
      home_anim_active = false;
    }
    drawScreen();
    delay(10);
    return;
#endif
  }

  // Clear arrow flash when it expires
  if ((home_arrow_flash_right || home_arrow_flash_left) && millis() >= home_arrow_flash_until_ms) {
    home_arrow_flash_right = false;
    home_arrow_flash_left = false;
  }

  if (wifi_popup) {
    if (clicked || key_nav.cancel || nav_back) {
      wifi_popup = false;
      wifi_status = "Cancelled";
      drawScreen();
    }
    const bool hold_connect = mainButtonPressedFor(kHoldPressMs) && !btn_hold_latched;
    if (hold_connect || key_nav.confirm) {
      if (hold_connect) {
        btn_hold_latched = true;
        ignore_click_after_hold = true;
      }
      connectWifiNow();
    }
    delay(10);
    return;
  }

  if (in_home) {
    if (nav_prev) {
      home_arrow_flash_left = true;
      home_arrow_flash_until_ms = millis() + kHomeArrowFlashMs;
      const MenuPage to_page = homePageWithOffset(-1);
      if (!home_anim_active || home_anim_to != to_page) {
        home_anim_from = menu_page;
        home_anim_to = to_page;
        home_anim_dir = -1;
        home_anim_start_ms = millis();
        home_anim_active = true;
        home_index = (home_index + homePageCount() - 1) % homePageCount();
        menu_page = to_page;
      }
    } else if (clicked || key_nav.next) {
      home_arrow_flash_right = true;
      home_arrow_flash_until_ms = millis() + kHomeArrowFlashMs;
      const MenuPage to_page = homePageWithOffset(1);
      if (!home_anim_active || home_anim_to != to_page) {
        home_anim_from = menu_page;
        home_anim_to = to_page;
        home_anim_dir = 1;
        home_anim_start_ms = millis();
        home_anim_active = true;
        home_index = (home_index + 1) % homePageCount();
        menu_page = to_page;
      }
    }
    const bool hold_enter = mainButtonPressedFor(kHoldPressMs) && !btn_hold_latched;
    if (hold_enter || key_nav.confirm) {
      if (hold_enter) {
        btn_hold_latched = true;
        ignore_click_after_hold = true;
      }
      enterCurrentFeature();
    }
    delay(10);
    return;
  }

  if (menu_page == MenuPage::Emulator) {
    if (emu_config_stage == EmuConfigStage::TypeMenu) {
      const uint8_t list_count = dumpMenuCount();
      const uint8_t visible_count = static_cast<uint8_t>(min(static_cast<int>(kEmuMenuVisibleCount), max(1, static_cast<int>(list_count))));

      if (clicked || key_nav.next || nav_prev) {
        uint8_t selected = emu_type_scroll + emu_type_cursor;
        selected = static_cast<uint8_t>((selected + 1) % list_count);
        if (selected < emu_type_scroll) {
          emu_type_scroll = selected;
        } else if (selected >= emu_type_scroll + visible_count) {
          emu_type_scroll = static_cast<uint8_t>(selected - visible_count + 1);
        }
        emu_type_cursor = static_cast<uint8_t>(selected - emu_type_scroll);
        drawScreen(true);
      }

      const bool hold_type_select = mainButtonPressedFor(kHoldPressMs) && !btn_hold_latched;
      if (hold_type_select || key_nav.confirm) {
        if (hold_type_select) {
          btn_hold_latched = true;
          ignore_click_after_hold = true;
        }

        const uint8_t selected = emu_type_scroll + emu_type_cursor;
        if (selected == 0) {
          emu_config_stage = EmuConfigStage::None;
          emu_show_menu = false;
          dumps_pick_for_emu = true;
          dumps_stage = DumpsStage::Browse;
          dumps_menu_index = 0;
          dump_file_index = 0;
          menu_page = MenuPage::WebFiles;
          startEmuApPortal();
          refreshDumpFiles(true);
          drawScreen();
        } else if (selected == 1) {
          goHome();
          delay(10);
          return;
        } else {
          const uint8_t file_idx = static_cast<uint8_t>(selected - kEmuDumpMenuBaseItems);
          if (file_idx < emu_dump_count) {
            loadDumpIntoEmulator(emu_dump_files[file_idx]);
          }
          emu_config_stage = EmuConfigStage::None;
          emu_show_menu = false;
          drawScreen();
        }
      }

      if (nav_back) {
        emu_config_stage = EmuConfigStage::None;
        emu_show_menu = false;
        drawScreen();
      }
    } else {
      if (nav_prev) {
        switchEmuType(-1);
      } else if (clicked || key_nav.next) {
        switchEmuType(1);
      }

      const bool hold_open_type_menu = mainButtonPressedFor(kHoldPressMs) && !btn_hold_latched;
      if (hold_open_type_menu || key_nav.confirm) {
        if (hold_open_type_menu) {
          btn_hold_latched = true;
          ignore_click_after_hold = true;
        }

        refreshDumpFiles();
        emu_type_scroll = 0;
        emu_type_cursor = 0;
        emu_config_stage = EmuConfigStage::TypeMenu;
        emu_show_menu = false;
        drawScreen(true);
      }

      if (nav_back) {
        goHome();
        delay(10);
        return;
      }
    }
  } else if (menu_page == MenuPage::WebFiles) {
    const bool hold_dumps = mainButtonPressedFor(kHoldPressMs) && !btn_hold_latched;
    if (hold_dumps) {
      btn_hold_latched = true;
      ignore_click_after_hold = true;
    }

    if (dumps_pick_for_emu) {
      const uint8_t count = dumpsBrowseCount();
      if (nav_prev) {
        if (count > 0) dump_file_index = static_cast<uint8_t>((dump_file_index + count - 1) % count);
        drawScreen();
      } else if (clicked || key_nav.next) {
        if (count > 0) dump_file_index = static_cast<uint8_t>((dump_file_index + 1) % count);
        drawScreen();
      }
      if (hold_dumps || key_nav.confirm) {
        if (dumpsSelectionIsBack()) {
          dumps_pick_for_emu = false;
          dumps_stage = DumpsStage::Browse;
        } else if (dumpsHasSelectedFile()) {
          const uint8_t file_idx = dumpsSelectedFileIndex();
          if (file_idx < emu_dump_count && loadDumpIntoEmulator(emu_dump_files[file_idx])) {
            dumps_pick_for_emu = false;
            dumps_stage = DumpsStage::Browse;
            showDumpsEmuPopup(activeEmulatorDisplayId(emu_type), emuName(emu_type));
          }
        }
        drawScreen();
      }
      if (nav_back || key_nav.cancel) {
        dumps_pick_for_emu = false;
        dumps_stage = DumpsStage::Browse;
        drawScreen();
      }
      delay(10);
      return;
    }

    if (dumps_stage == DumpsStage::Preview) {
      if (key_nav.key_b) {
        dumps_preview_font_level = static_cast<uint8_t>((dumps_preview_font_level + 1u) & 0x03u);
        drawScreen();
      } else if (hold_dumps) {
        dumps_stage = DumpsStage::Browse;
        drawScreen();
      } else if (clicked || key_nav.next || key_nav.confirm) {
        const int view_w = g_canvas.width() - 4;
        const int view_h = g_canvas.height() - 18;
        int line_h = 10;
        dumpsPreviewApplyFont(g_canvas, dumps_preview_font_level, line_h);
        const size_t page_lines = dumpsPreviewPageLines(max(20, view_h), line_h);
        const size_t total_lines = dumpsPreviewCountLines(dumps_preview_text);
        if (total_lines <= page_lines) {
          dumps_preview_offset = 0;
        } else {
          dumps_preview_offset += page_lines;
          if (dumps_preview_offset >= total_lines) dumps_preview_offset = 0;
        }
        drawScreen();
      }
    } else if (dumps_stage == DumpsStage::PortalQr) {
      if (nav_prev) {
        prepareDumpsQrPayload(!dumps_qr_wifi);
        drawScreen();
      } else if (clicked || nav_back || key_nav.cancel || key_nav.confirm || hold_dumps) {
        dumps_stage = DumpsStage::Browse;
        drawScreen();
      }
    } else if (dumps_stage == DumpsStage::Menu) {
      const uint8_t menu_count = dumpsMenuCount();
      if (dumpsMenuItemDisabled(dumps_menu_index)) {
        dumps_menu_index = dumpsMenuNextEnabledIndex(dumps_menu_index, +1);
      }
      if (nav_prev) {
        dumps_menu_index = dumpsMenuNextEnabledIndex(dumps_menu_index, -1);
        drawScreen();
      } else if (clicked || key_nav.next) {
        dumps_menu_index = dumpsMenuNextEnabledIndex(dumps_menu_index, +1);
        drawScreen();
      }
      if (hold_dumps || key_nav.confirm) {
        if (dumps_menu_index == 0) {
          if (emu_dump_count > 0 && dump_file_index < emu_dump_count) {
            EmuType inferred = emu_type;
            const String type_hint = dumpTypeLabel(emu_dump_files[dump_file_index]);
            if (mapTypeHintToEmuType(type_hint, inferred)) {
              emu_type = inferred;
            }
            if (loadDumpIntoEmulator(emu_dump_files[dump_file_index])) {
              dumps_stage = DumpsStage::Browse;
              showDumpsEmuPopup(activeEmulatorDisplayId(emu_type), emuName(emu_type));
              drawScreen();
              delay(10);
              return;
            }
          }
        } else if (dumps_menu_index == 1) {
          if (emu_dump_count > 0 && dump_file_index < emu_dump_count) {
            dumps_preview_text = buildDumpPreview(emu_dump_files[dump_file_index]);
            dumps_preview_offset = 0;
            dumps_stage = DumpsStage::Preview;
            drawScreen();
            delay(10);
            return;
          }
        } else if (dumps_menu_index == 2) {
          if (!emu_ap_active) startEmuApPortal();
          prepareDumpsQrPayload(true);
          dumps_stage = DumpsStage::PortalQr;
          drawScreen();
        } else if (dumps_menu_index == 3) {
          goHome();
        }
      }
      if (nav_back || key_nav.cancel) {
        dumps_stage = DumpsStage::Browse;
        drawScreen();
      }
    } else {
      if (nav_prev) {
        if (emu_dump_count > 0) dump_file_index = static_cast<uint8_t>((dump_file_index + emu_dump_count - 1) % emu_dump_count);
        drawScreen();
      } else if (clicked || key_nav.next) {
        if (emu_dump_count > 0) dump_file_index = static_cast<uint8_t>((dump_file_index + 1) % emu_dump_count);
        drawScreen();
      }
      if (hold_dumps || key_nav.confirm) {
        dumps_menu_index = dumpsMenuItemDisabled(0) ? 1 : 0;
        dumps_stage = DumpsStage::Menu;
        drawScreen();
      }
      if (nav_back || key_nav.cancel) {
        if (dumps_pick_for_emu) {
          dumps_pick_for_emu = false;
          menu_page = MenuPage::Emulator;
          drawScreen();
        } else {
          goHome();
        }
      }
    }
  } else if (menu_page == MenuPage::Piano) {
    if (piano_stage == PianoStage::Menu) {
      if (nav_prev) {
        piano_menu_index = static_cast<uint8_t>((piano_menu_index + 2) % 3);
        drawScreen();
      } else if (clicked || key_nav.next) {
        piano_menu_index = static_cast<uint8_t>((piano_menu_index + 1) % 3);
        drawScreen();
      }
      const bool hold_piano_enter = mainButtonPressedFor(kHoldPressMs) && !btn_hold_latched;
      if (hold_piano_enter || key_nav.confirm) {
        if (hold_piano_enter) {
          btn_hold_latched = true;
          ignore_click_after_hold = true;
        }
        piano_active_card_key = "";
        piano_active_note_idx = -1;
        piano_last_sustain_ms = 0;
        if (piano_menu_index == 0) {
          setSpeaker5V(true);
          piano_stage = PianoStage::Play;
          piano_last_note = "-";
          piano_status = "Tap card to play";
        } else if (piano_menu_index == 1) {
          piano_stage = PianoStage::Config;
          piano_config_step = 0;
          for (uint8_t i = 0; i < kPianoNoteCount; ++i) {
            piano_card_map[i] = "";
          }
          piano_last_note = "-";
          piano_status = String("Scan ") + kPianoNoteName[piano_config_step];
        } else {
          goHome();
          delay(10);
          return;
        }
        drawScreen();
      }
      if (nav_back) {
        goHome();
        delay(10);
        return;
      }
    } else {
      const bool hold_piano_back = mainButtonPressedFor(kHoldPressMs) && !btn_hold_latched;
      if (hold_piano_back || nav_back) {
        if (hold_piano_back) {
          btn_hold_latched = true;
          ignore_click_after_hold = true;
        }
        piano_stage = PianoStage::Menu;
        setSpeaker5V(false);
        piano_menu_index = 0;
        piano_active_card_key = "";
        piano_active_note_idx = -1;
        piano_last_sustain_ms = 0;
        piano_last_note = "-";
        const uint8_t mapped = pianoMappedCount();
        piano_status = "Configured " + String(mapped) + "/8";
        drawScreen();
      }
    }
  } else {
    if (clicked || key_nav.next || key_nav.confirm) {
      if (menu_page == MenuPage::Reader) {
        reader_14b_only = !reader_14b_only;
        last_card.valid = false;
        last_card.protocol = "None";
        last_card.uid = "";
        last_card.detail = reader_14b_only ? "14B only" : "Scanning";
        reader_last_hold_log_ms = 0;
        reader_fail_streak = 0;
        reader_need_first_tone = true;
        last_poll_ms = 0;
        Serial.printf("[READER] mode=%s\n", reader_14b_only ? "14B_ONLY" : "AUTO");
        drawScreen();
      } else if (menu_page == MenuPage::ReadNDEF) {
        scanNdefNow();
      } else if (menu_page == MenuPage::WebFiles) {
        if (emu_ap_active) {
          stopEmuApPortal();
        } else {
          startEmuApPortal();
        }
        refreshDumpFiles(dumps_pick_for_emu);
        drawScreen();
      } else if (menu_page == MenuPage::About) {
        about_page_idx = (about_page_idx + 1) % 2;
        drawScreen();
      } else if (menu_page == MenuPage::Diagnose) {
        runDiagnose();
      }
    }
    const bool hold_go_home = mainButtonPressedFor(kHoldPressMs) && !btn_hold_latched;
    if (hold_go_home || nav_back) {
      if (hold_go_home) {
        btn_hold_latched = true;
        ignore_click_after_hold = true;
      }
      goHome();
    }
  }

  // Process NFC worker results for Reader / Piano (non-blocking)
  if (menu_page == MenuPage::Reader && !in_home) {
    processReaderResult();
  } else if (menu_page == MenuPage::Piano && !in_home) {
    processPianoResult();
  }

  delay(reader_anim_active ? 1 : 10);
}
