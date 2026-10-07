// ============================================================
// 車速取り込みツール - 本番仕様 統合ファームウェア
// XIAO ESP32-S3 + ADS1015(アナログ) + MCP2562FD-E/P(CAN)
//        + BLE(内蔵) + WiFi(内蔵、Webページ配信つき)
//
// speed_tracer_pc.html (計測入力ソース選択パネル対応版) と対で使用します。
//
// できること:
//   - アプリ側のボタンで「アナログ / CAN」を選択できる
//     (DLCコネクタを電源だけに使っていてCAN未配線の人はアナログのままでOK)
//   - CAN選択時、さらに「代表(車速のみ) / 全部(RPM・水温・スロットル・負荷も)」を選択できる
//   - **CANが選ばれていない間は、CANへのリクエスト送信を一切行いません**
//     (配線していない/電源専用の人のバスに余計な負荷をかけない設計)
//   - 設定はNVS(内蔵不揮発メモリ)に保存され、電源を切っても前回の設定を覚えている
//   - USB/BLE/WiFiの3経路全部からコマンドを受け付けて設定を変更できる
//
// コマンド一覧(USBシリアル/BLE RXキャラクタリスティック/WebSocketいずれからでも):
//   SRC:ANALOG      → 計測ソースをアナログに切替
//   SRC:CAN         → 計測ソースをCANに切替
//   CANMODE:REP     → CAN代表モード(車速のみ)
//   CANMODE:ALL     → CAN全部モード(RPM等も取得)
//   dtc             → DTC読取(シリアルのみ、デバッグ用)
//   clear           → DTCクリア(シリアルのみ、デバッグ用)
//   log on/log off  → 生CANフレームログ(シリアルのみ、デバッグ用)
//   status          → 現在の設定を表示
//   canstat         → CANバスの実際の通信状態(エラーカウンタ等)を表示
//   canreset        → CANドライバをソフトウェア的に再初期化(BUS_OFFからの復帰用)
//   BAUD:500        → CAN通信速度を500kbpsに設定し再初期化(デフォルト)
//   BAUD:250        → CAN通信速度を250kbpsに設定し再初期化(低速CAN車両向け)
//   LISTEN:ON       → リスニングオンリーモード(送信せず受信のみ)。車のECUが
//                      本当にバスへ何か発信しているかを、自分の送信エラーの
//                      影響を受けずに確認できる。切り分けの決め手になる
//   LISTEN:OFF      → 通常モード(送受信両方)に戻す
//   SELFTEST:ON     → 自己診断モード(NO_ACK)ON。車不要でMCU⇔チップ間の
//                      通信ループが正常か検証できる(最優先で試すべきテスト)
//   SELFTEST:OFF    → 自己診断モードOFF、通常設定に戻す
//   SELFTEST:SEND   → 自己診断モード中にテストフレームを1つ送信し、直後の
//                      canstatを自動表示(RUNNINGのままならMCU⇔チップ間は正常)
//   gpiotest        → TWAIプロトコル層を使わず、D8/D9を素のGPIOとして
//                      TXD=L/HでRXDが追従するか直接確認(最も原始的な検証)
//   PID:0B          → 任意のPID(16進数)を1回だけ手動照会。例: PID:0B でブースト圧(0x0B)を直接テスト
//                      応答が8バイトに収まらないPID(7A/78/85等)もマルチフレームで受けて全バイト返す
//   PID22:700:1F87  → UDS Mode22(拡張DID)を任意ヘッダー宛てに1回だけ手動照会。
//                      書式: PID22:<ヘッダー16進>:<DID16進>。例はヘッダー0x700宛てにDID 0x1F87を照会
//   ファーム更新(OTA): SpeedTracerのWiFiにつないだブラウザで http://192.168.4.1/update を開き、
//      Arduino IDEの「スケッチ」メニューの Export Compiled Binary でできる .ino.bin を送ると書き換わる(USB不要)。
//      ただしこの版を最初に入れる1回だけは、これまでどおりUSBで書き込む。
//   ELM327互換ポート(WiFi、port 35000): Torque/Car Scanner等の市販アプリから
//      「WiFi ELM327アダプター」として接続可能。AT系コマンドは概ねOKを返すのみの簡易実装。
//      Mode22を使う場合はアプリ側から先に "ATSH <ヘッダー16進>" を送ってヘッダーを指定すること。
//   ディープスリープ(バッテリー上がり対策): WiFi/BLE/ELMいずれの接続も無い状態が15分続くと
//      自動でディープスリープに入る。自動起床はせず、復帰は電源の抜き差し(リセット)のみ。
//      SLEEP:STATUS → 現在の接続状況・アイドル時間を確認
//      SLEEP:NOW    → 動作確認用。タイムアウトを待たず即座にディープスリープへ入る
//   raw             → アナログの現在の分圧後電圧/実電圧を表示(校正作業用)
//   CAL:V0=0.30         → 0km/h時の実電圧をセット(CDY個体差のオフセット調整)
//   CAL:VMAX=9.80       → 最大速度時の実電圧をセット(CDY個体差の傾き調整)
//   CAL:SPEEDMAX=180    → 上記VMAXに対応する車速(km/h)をセット
//
// 校正の手順の例:
//   1. 車速0km/h(停車)の状態で "raw" を入力し、表示された「センサー実電圧」を確認
//   2. その値を CAL:V0=xxx でセット
//   3. 既知の速度(法定の速度計や別の基準)で走行しながら "raw" を確認し、
//      その時の実電圧とその時の実際の速度を CAL:VMAX= / CAL:SPEEDMAX= にセット
//   4. 設定は自動でNVSに保存されるので、電源を切っても覚えています
//
// 出力形式(既存アプリと互換):
//   SPEED:xx.x                         ← 常に送信(アナログ or CANの車速)
//   EXTRA:rpm,coolant,throttle,load    ← CAN全部モードの時だけ追加送信
//
// BLE/WiFi仕様: 既存(speed_tracer_analog_ble.ino)のUUID/SSID/ポートと完全一致。
//   ただしBLEには新たに「RXキャラクタリスティック」(コマンド受信用, Write)を追加。
//   旧アプリ(コマンド送信機能なし)と接続しても、Notifyの受信自体は問題なく動きます。
//
// 配線: speed_tracer_hybrid.ino と同じ(アナログ・CAN両方の配線が必要)
//   [アナログ] D4->ADS1015 SDA, D5->ADS1015 SCL
//   [CAN]     D8->MCP2562FD TXD, D9->MCP2562FD RXD, CANH/CANL->OBD2 Pin6/14
//   DLCを電源専用で使う場合は、+12V(Pin16)とGND(Pin4/5)だけ配線すればOKです
//   (CANH/CANLを配線しなくても、アナログソースを選んでいれば問題なく動作します)
//
// 必要ライブラリ: Adafruit ADS1X15, WebSockets(Links2004)
//   (BLE/WiFi/CAN/Preferences/Updateは標準搭載)
//
// 変更履歴:
//   2026-10-07.1
//     - マルチフレーム(ISO-TP)応答の受信に対応。PID: / PID22: / 監視チャンネル / ELM327互換ポート / DTC読取で有効
//     - WiFi経由のファーム更新ページ(/update)を追加
//     - CANの照会を1件ずつ直列化(BLE経由のコマンドとloop()の照会が応答を横取りし合わないように)
//     - PID: の len を、ECUが宣言した実際のデータ長に変更(以前は詰め物込みで常に5)
//     - DTC読取で、応答の2バイト目(件数)をDTCの一部として読んでいた不具合を修正
// ============================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include "driver/twai.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include <Update.h>
#include "esp_ota_ops.h"
#include "webpage_html.h"

// ファームウェアの版。更新ページ(http://192.168.4.1/update)と status コマンドに表示される
#define FW_VERSION "2026-10-07.1"

Preferences prefs;

// ============ 型の定義(必ず最初の関数より前に置く) ============
// Arduino IDEは関数のプロトタイプを自動生成して「最初の関数の直前」に差し込む。
// 関数の引数に使う型がそれより後ろにあるとコンパイルエラーになるため、ここにまとめる。
#define ISOTP_MAX_PAYLOAD 64   // 1応答で保持する最大バイト数(SID含む)。超えた分は数えるだけで捨てる。
                               // BLE経由のコマンドは通信タスクの小さなスタック上で動くので、むやみに大きくしない
#define ISOTP_MAX_ECUS    4    // 機能アドレス(0x7DF)照会で同時に受ける応答元ECUの上限
struct IsoTpResp {
  uint32_t id;      // 応答元のCAN ID
  uint16_t total;   // ECUが宣言した応答の全長(SID含む)
  uint16_t got;     // 受信済みバイト数(全長まで数える)
  uint16_t len;     // dataに格納済みのバイト数(ISOTP_MAX_PAYLOADで頭打ち)
  uint8_t  nextSn;  // 次に来るはずのConsecutive Frameの連番
  bool     multi;   // マルチフレーム応答だったか
  bool     done;    // 受信完了
  uint8_t  data[ISOTP_MAX_PAYLOAD]; // SIDから始まる応答本体(例: 41 7A xx xx ...)
};
enum Pid22Status { PID22_OK, PID22_NEGATIVE, PID22_NORESP };

// CANの照会(送信→応答待ち)を1件ずつ直列にするための鍵。
// BLE経由のコマンドはloop()とは別のタスクで実行されるため、鍵が無いと互いの応答フレームを
// 横取りしてしまう(マルチフレームでは途中のフレームが1つ欠けるだけで応答全体が失われる)。
SemaphoreHandle_t g_canMutex = nullptr;
struct CanLock {
  CanLock()  { if (g_canMutex) xSemaphoreTake(g_canMutex, portMAX_DELAY); }
  ~CanLock() { if (g_canMutex) xSemaphoreGive(g_canMutex); }
};

// ============ モード管理(NVSに保存) ============
enum SrcType { SRC_ANALOG, SRC_CAN };
enum CanDetail { CAN_REP, CAN_ALL };
SrcType   g_src        = SRC_ANALOG;
CanDetail g_canDetail  = CAN_REP;
float g_calV0       = 0.0;   // 0km/h時の実電圧(オフセット、CDY個体差でここが変わる)
float g_calVMax     = 10.0;  // 最大速度時の実電圧(LSB/傾き相当、CDY個体差でここも変わる)
float g_calSpeedMax = 200.0; // g_calVMaxに対応する車速(km/h)
int   g_canBaud     = 500;   // CAN通信速度(kbps): 500 or 250
bool  g_listenOnly  = false; // true: 送信せず受信のみ(リスニングオンリーモード)
bool  g_selfTest    = false; // true: NO_ACKモード(車不要でMCU⇔チップ間の自己診断)

void loadSettings() {
  prefs.begin("speedtracer", true);
  g_src       = (SrcType)prefs.getUChar("src", SRC_ANALOG);
  g_canDetail = (CanDetail)prefs.getUChar("canDetail", CAN_REP);
  g_calV0        = prefs.getFloat("calV0", 0.0);
  g_calVMax      = prefs.getFloat("calVMax", 10.0);
  g_calSpeedMax  = prefs.getFloat("calSpeedMax", 200.0);
  g_canBaud      = prefs.getInt("canBaud", 500);
  g_listenOnly   = prefs.getBool("listenOnly", false);
  prefs.end();
}
void saveSettings() {
  prefs.begin("speedtracer", false);
  prefs.putUChar("src", (uint8_t)g_src);
  prefs.putUChar("canDetail", (uint8_t)g_canDetail);
  prefs.putFloat("calV0", g_calV0);
  prefs.putFloat("calVMax", g_calVMax);
  prefs.putFloat("calSpeedMax", g_calSpeedMax);
  prefs.putInt("canBaud", g_canBaud);
  prefs.putBool("listenOnly", g_listenOnly);
  prefs.end();
}

// ============ アナログ(ADS1015) ============
Adafruit_ADS1015 ads;
bool g_adsOk = false;

const int PIN_SDA = D4;
const int PIN_SCL = D5;

const float DIVIDER_R1 = 10000.0;
const float DIVIDER_R2 = 3300.0;
const float DIVIDER_RATIO = DIVIDER_R2 / (DIVIDER_R1 + DIVIDER_R2);

const int SAMPLES = 8;

float g_lastDividedVoltage = 0, g_lastActualVoltage = 0; // "raw"コマンドでの確認用

float readAnalogSpeed() {
  if (!g_adsOk) return -1;
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int16_t raw = ads.readADC_SingleEnded(0);
    sum += raw;
    delay(2);
  }
  float avgRaw = (float)sum / SAMPLES;
  float dividedVoltage = avgRaw * 2.0 / 1000.0;
  float actualVoltage = dividedVoltage / DIVIDER_RATIO;
  g_lastDividedVoltage = dividedVoltage;
  g_lastActualVoltage  = actualVoltage;

  // 校正値(オフセット・傾き)を反映した線形変換。CDY個体差はここで吸収する。
  float ratio = (actualVoltage - g_calV0) / (g_calVMax - g_calV0);
  float speedKmh = ratio * g_calSpeedMax;
  if (speedKmh < 0) speedKmh = 0;
  return speedKmh;
}

// ============ CAN(TWAI) ============
const gpio_num_t CAN_TX_PIN = (gpio_num_t)D8;
const gpio_num_t CAN_RX_PIN = (gpio_num_t)D9;

const uint32_t OBD_REQUEST_ID = 0x7DF;
const uint32_t OBD_RESP_MIN   = 0x7E8;
const uint32_t OBD_RESP_MAX   = 0x7EF;
bool g_canOk = false;
bool g_rawLogEnabled = false;

void setupTWAI() {
  twai_mode_t mode = g_selfTest ? TWAI_MODE_NO_ACK :
                     (g_listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, mode);
  // マルチフレーム応答の後続フレームは約0.3ms間隔で連続して届く。既定(5フレーム)では
  // 取りこぼすことがあるため、受信キューを広げる
  g_config.rx_queue_len = 32;
  twai_timing_config_t t_config = (g_canBaud == 250) ?
      (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS() :
      (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  g_canOk = (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) &&
            (twai_start() == ESP_OK);
  Serial.print("CAN: ");
  Serial.print(g_canBaud);
  Serial.print("kbps ");
  Serial.print(g_selfTest ? "[SELF-TEST NO_ACK] " : (g_listenOnly ? "[LISTEN ONLY] " : "[NORMAL] "));
  Serial.println(g_canOk ? "初期化成功(SRC:CAN選択時のみ通信します)" : "初期化失敗");
}

void printRawFrame(const twai_message_t& msg) {
  String s = String(millis()) + ",0x" + String(msg.identifier, HEX) + "," + String(msg.data_length_code);
  for (int i = 0; i < msg.data_length_code; i++) {
    s += ",";
    if (msg.data[i] < 0x10) s += "0";
    s += String(msg.data[i], HEX);
  }
  Serial.println(s);
  broadcastText(s);
}

// ============ ISO-TP(ISO 15765-2)による照会 ============
// 応答が1フレーム(SID含め7バイト)に収まらないPID(DPF差圧 7A、排気温 78、NOx還元剤 85 など)は、
// ECUが次の順で分割して送ってくる:
//   First Frame(先頭6バイト) → [こちらからFlow Controlを返す] → Consecutive Frame(7バイトずつ)
// Flow Controlを返さない限り続きは送られてこないので、ここで返して組み立てる。
const unsigned long ISOTP_CF_TIMEOUT_MS = 150;   // First/Consecutive Frameの後、次のフレームを待つ時間
const unsigned long ISOTP_PENDING_MS    = 500;   // 否定応答0x78(処理中、後で本応答を送る)を受けた時に待つ時間
const unsigned long ISOTP_HARD_LIMIT_MS = 1500;  // 1回の照会でloop()を止めてよい上限

// Flow Controlの宛先。機能アドレス(0x7DF)で聞いた場合は、応答してきたECUの物理アドレスへ返す。
uint32_t isoTpFlowControlId(uint32_t reqId, bool extended, uint32_t respId) {
  if (!extended) {
    if (reqId == OBD_REQUEST_ID) return respId - 8;   // 例: 0x7E8 → 0x7E0
    return reqId;                                     // 物理アドレス宛ての照会は、その宛先へ
  }
  if (((reqId >> 16) & 0xFF) == 0xDB) {               // 29bitの機能アドレス(0x18DB33F1)
    // 0x18DAF1xx(xx=ECU) → 0x18DAxxF1
    return (respId & 0xFFFF0000UL) | ((respId & 0xFFUL) << 8) | ((respId >> 8) & 0xFFUL);
  }
  return reqId;
}

void isoTpExtendDeadline(unsigned long& deadline, unsigned long hardLimit, unsigned long ms) {
  unsigned long d = millis() + ms;
  if ((long)(d - hardLimit) > 0) d = hardLimit;
  if ((long)(d - deadline) > 0) deadline = d;
}

// 照会を1件送り、応答を組み立てて out[] に応答元ECUごとに格納する。戻り値は受信完了した応答の数。
//   req/reqLen   : 送るデータ(SIDから。例 Mode01 PID 7A なら {0x01,0x7A})。7バイトまで
//   obdRangeOnly : true=応答IDを0x7E8-0x7EFに限る(Mode01等) / false=IDを問わず中身で判定(Mode22)
//   maxOut       : 1なら最初に応答し始めたECUだけを最後まで受けて即戻る。2以上なら時間いっぱい集める
//   negId/negNrc : 否定応答(7F ...)を受けた場合、その応答元IDとNRCが入る(不要ならnullptr)
int isoTpQuery(uint32_t reqId, bool extended, const uint8_t* req, uint8_t reqLen, bool obdRangeOnly,
               IsoTpResp* out, int maxOut, unsigned long timeoutMs, uint32_t* negId, uint8_t* negNrc) {
  if (negId) *negId = 0;
  if (negNrc) *negNrc = 0;
  if (!g_canOk || reqLen == 0 || reqLen > 7 || maxOut < 1) return 0;
  if (maxOut > ISOTP_MAX_ECUS) maxOut = ISOTP_MAX_ECUS;
  CanLock lock;

  // 前回の照会の残りや、遅れて届いた応答を捨てる(今回の応答と混ざらないようにする)
  twai_message_t rx;
  for (int i = 0; i < 64 && twai_receive(&rx, 0) == ESP_OK; i++) {
    if (g_rawLogEnabled) printRawFrame(rx);
  }

  twai_message_t msg = {};
  msg.identifier = reqId;
  msg.extd = extended ? 1 : 0;
  msg.data_length_code = 8;
  msg.data[0] = reqLen;                                  // Single Frame: 上位4bit=0、下位4bit=長さ
  for (int i = 0; i < 7; i++) msg.data[1 + i] = (i < reqLen) ? req[i] : 0x00;
  if (twai_transmit(&msg, pdMS_TO_TICKS(20)) != ESP_OK) return 0;

  const uint8_t posSid  = req[0] + 0x40;                 // 肯定応答のSID(01→41、22→62)
  const uint8_t echoLen = reqLen - 1;                    // SIDの後ろにそのまま返ってくるバイト数(PIDやDID)
  int slots = 0, doneCount = 0;
  bool giveUp = false;
  const unsigned long start = millis();
  unsigned long deadline = start + timeoutMs;
  const unsigned long hardLimit = start + ISOTP_HARD_LIMIT_MS;

  while (!giveUp && doneCount < maxOut && (long)(deadline - millis()) > 0) {
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) != ESP_OK) continue;
    if (g_rawLogEnabled) printRawFrame(rx);
    if (rx.rtr || rx.data_length_code < 2) continue;
    if (obdRangeOnly && (rx.extd || rx.identifier < OBD_RESP_MIN || rx.identifier > OBD_RESP_MAX)) continue;
    const uint8_t dlc = (rx.data_length_code > 8) ? 8 : rx.data_length_code;
    const uint8_t pciType = rx.data[0] >> 4;

    int si = -1;                                         // このIDから受信中/受信済みの枠
    for (int i = 0; i < slots; i++) if (out[i].id == rx.identifier) { si = i; break; }

    if (pciType == 0) {
      // ---- Single Frame: [長さ][SID][...] ----
      const uint8_t n = rx.data[0] & 0x0F;
      if (n == 0 || n > dlc - 1) continue;
      const uint8_t* p = &rx.data[1];
      if (p[0] == 0x7F) {                                // 否定応答: 7F [聞いたSID] [NRC]
        if (n >= 3 && p[1] == req[0]) {
          if (p[2] == 0x78) {
            isoTpExtendDeadline(deadline, hardLimit, ISOTP_PENDING_MS);
          } else {
            if (negId && *negId == 0) { *negId = rx.identifier; if (negNrc) *negNrc = p[2]; }
            if (maxOut == 1 && slots == 0) giveUp = true;
          }
        }
        continue;
      }
      if (p[0] != posSid || n < 1 + echoLen) continue;
      bool match = true;
      for (uint8_t k = 0; k < echoLen; k++) if (p[1 + k] != req[1 + k]) { match = false; break; }
      if (!match || si >= 0 || slots >= maxOut) continue;
      IsoTpResp &s = out[slots++];
      s.id = rx.identifier; s.total = n; s.got = n; s.len = n;
      s.nextSn = 0; s.multi = false; s.done = true;
      for (uint8_t k = 0; k < n; k++) s.data[k] = p[k];
      doneCount++;

    } else if (pciType == 1) {
      // ---- First Frame: [1 | 全長の上位4bit][全長の下位8bit][先頭6バイト] ----
      if (dlc < 8) continue;
      const uint16_t total = ((uint16_t)(rx.data[0] & 0x0F) << 8) | rx.data[1];
      if (total <= 6) continue;                          // 1フレームに収まる長さのFirst Frameは不正
      const uint8_t* p = &rx.data[2];
      if (p[0] != posSid) continue;
      bool match = true;
      for (uint8_t k = 0; k < echoLen && k < 5; k++) if (p[1 + k] != req[1 + k]) { match = false; break; }
      if (!match || si >= 0 || slots >= maxOut) continue;
      IsoTpResp &s = out[slots++];
      s.id = rx.identifier; s.total = total; s.got = 6; s.len = 6;
      s.nextSn = 1; s.multi = true; s.done = false;
      for (uint8_t k = 0; k < 6; k++) s.data[k] = p[k];

      twai_message_t fc = {};                            // Flow Control: 続きを全部、間隔を空けずに送ってよい
      fc.identifier = isoTpFlowControlId(reqId, extended, rx.identifier);
      fc.extd = rx.extd;
      fc.data_length_code = 8;
      fc.data[0] = 0x30;                                 // 3=Flow Control、0=送信続行
      fc.data[1] = 0x00;                                 // ブロックサイズ0=最後まで続けて送る
      fc.data[2] = 0x00;                                 // フレーム間の最小間隔0ms
      twai_transmit(&fc, pdMS_TO_TICKS(20));
      isoTpExtendDeadline(deadline, hardLimit, ISOTP_CF_TIMEOUT_MS);

    } else if (pciType == 2) {
      // ---- Consecutive Frame: [2 | 連番(1,2,...,F,0,1,...)][続きの7バイト] ----
      if (si < 0 || out[si].done || !out[si].multi) continue;
      IsoTpResp &s = out[si];
      if ((rx.data[0] & 0x0F) != s.nextSn) {             // 連番が飛んだ=途中のフレームを取りこぼした
        s.multi = false;                                 // この応答は捨てる(doneにならないので数に入らない)
        if (maxOut == 1) giveUp = true;
        continue;
      }
      uint16_t n = dlc - 1;
      if (n > s.total - s.got) n = s.total - s.got;
      for (uint16_t k = 0; k < n; k++) if (s.len < ISOTP_MAX_PAYLOAD) s.data[s.len++] = rx.data[1 + k];
      s.got += n;
      s.nextSn = (s.nextSn + 1) & 0x0F;
      if (s.got >= s.total) { s.done = true; doneCount++; }
      else isoTpExtendDeadline(deadline, hardLimit, ISOTP_CF_TIMEOUT_MS);
    }
    // pciType 3(他ノードのFlow Control)などは無視
  }

  // 受信完了したものだけを前に詰めて返す
  int w = 0;
  for (int i = 0; i < slots; i++) {
    if (!out[i].done) continue;
    if (w != i) out[w] = out[i];
    w++;
  }
  return w;
}

// Mode01を1件照会し、最初に応答したECUのデータ部(41 PID の後ろ)を返す。マルチフレーム応答にも対応。
// outLenはECUが宣言した実際のデータ長(以前は詰め物込みで常に5だった)。outMaxを超える分は切り捨てる。
bool requestPID01(uint8_t pid, uint8_t* outData, uint8_t outMax, uint8_t& outLen, unsigned long timeoutMs = 50) {
  const uint8_t req[2] = {0x01, pid};
  IsoTpResp r;
  outLen = 0;
  if (isoTpQuery(OBD_REQUEST_ID, false, req, 2, true, &r, 1, timeoutMs, nullptr, nullptr) < 1) return false;
  uint16_t n = (r.len > 2) ? (r.len - 2) : 0;
  if (n > outMax) n = outMax;
  for (uint16_t i = 0; i < n; i++) outData[i] = r.data[2 + i];
  outLen = (uint8_t)n;
  return true;
}

// ELM327ブリッジ専用: Mode01の機能アドレス指定(0x7DF)は複数ECUが同時応答することがあるため、
// タイムアウト内に届いた応答元(ECU)ごとにまとめて返す(実ELM327が複数行で返す挙動に合わせる)。
int requestPID01Multi(uint8_t pid, IsoTpResp* out, int maxOut, unsigned long timeoutMs = 100) {
  const uint8_t req[2] = {0x01, pid};
  return isoTpQuery(OBD_REQUEST_ID, false, req, 2, true, out, maxOut, timeoutMs, nullptr, nullptr);
}

// UDS Mode22(拡張PID/DID)を任意のヘッダー宛てに1回だけ送る照会。マルチフレーム応答にも対応。
// 応答IDは車種・ECUごとに慣習が異なるため固定せず、SID一致(正常=0x62/否定応答=0x7F 0x22)で判定する。
// outDataには 62 DID の後ろのデータ部が入る。
// extended: false=11bit標準CAN(0x000-0x7FF)、true=29bit拡張CAN(例:Car Scannerが使うISO15765-4 CAN29/500)
Pid22Status requestPID22(uint32_t reqId, uint16_t did, uint8_t* outData, uint8_t outMax, uint8_t& outLen,
                          uint32_t& respId, uint8_t& nrc, bool extended = false, unsigned long timeoutMs = 150) {
  const uint8_t req[3] = {0x22, (uint8_t)((did >> 8) & 0xFF), (uint8_t)(did & 0xFF)};
  IsoTpResp r;
  uint32_t negId = 0; uint8_t negNrc = 0;
  outLen = 0;
  if (isoTpQuery(reqId, extended, req, 3, false, &r, 1, timeoutMs, &negId, &negNrc) >= 1) {
    uint16_t n = (r.len > 3) ? (r.len - 3) : 0;
    if (n > outMax) n = outMax;
    for (uint16_t i = 0; i < n; i++) outData[i] = r.data[3 + i];
    outLen = (uint8_t)n;
    respId = r.id;
    return PID22_OK;
  }
  if (negId != 0) { respId = negId; nrc = negNrc; return PID22_NEGATIVE; }
  return PID22_NORESP;
}

// ============ 監視パラメータ(チャンネル)の汎用設定機構 ============
// 以前はRPM/水温/スロットル等が固定のcase文だったが、車種によって使えるPID/DIDが違うため、
// 「Mode(01/22)・PID(またはヘッダー+DID)・読み取るバイト位置・換算式(倍率+オフセット)・名前・単位」を
// 10チャンネル分、アプリから自由に設定できるようにする。デフォルト値は従来のRPM等と同じ設定。
struct ChanConfig {
  uint8_t mode;        // 1 = Mode01, 22 = Mode22
  uint16_t pidOrDid;    // Mode01ならPID、Mode22ならDID
  uint16_t header;      // Mode22の時だけ使うリクエストヘッダー(Mode01は常にOBD_REQUEST_ID)
  uint8_t byteOffset;   // 応答データの何バイト目から読むか(0始まり)
  uint8_t byteCount;    // 読むバイト数(1〜3、ビッグエンディアンとして結合)
  float scale;          // 生値 × scale + offset = 表示値
  float offset;
  char label[12];       // 表示名(例: "RPM","水温")
  char unit[6];         // 単位(例: "rpm","C","kPa")
};
#define NUM_CHANNELS 10
ChanConfig g_chan[NUM_CHANNELS];
float g_chanVal[NUM_CHANNELS] = {0};
int g_extraCycleIndex = 0;
const int EXTRA_SIGNAL_COUNT = NUM_CHANNELS;
Preferences chanPrefs;

void setDefaultChannels() {
  // 元の固定10項目(RPM/水温/スロットル/負荷/ブースト/吸気温/MAF/点火時期/電圧/燃料)と同じ設定
  g_chan[0] = {1, 0x0C, 0, 0, 2, 0.25f,    0,    "RPM",     "rpm"};
  g_chan[1] = {1, 0x05, 0, 0, 1, 1.0f,   -40,    "Coolant", "C"};
  g_chan[2] = {1, 0x11, 0, 0, 1, 100.0f/255, 0,  "Throttle","%"};
  g_chan[3] = {1, 0x04, 0, 0, 1, 100.0f/255, 0,  "Load",    "%"};
  g_chan[4] = {1, 0x87, 0, 1, 2, 1.0f/32,  0,    "MAP",     "kPa"};
  g_chan[5] = {1, 0x0F, 0, 0, 1, 1.0f,   -40,    "IAT",     "C"};
  g_chan[6] = {1, 0x10, 0, 0, 2, 0.01f,    0,    "MAF",     "g/s"};
  g_chan[7] = {1, 0x0E, 0, 0, 1, 0.5f,   -64,    "Timing",  "deg"};
  g_chan[8] = {1, 0x42, 0, 0, 2, 0.001f,   0,    "Battery", "V"};
  g_chan[9] = {22, 0x1022, 0x7C0, 0, 2, 0.01f, 0, "Fuel",   "L"};
}
void loadChannels() {
  chanPrefs.begin("chcfg", true);
  size_t n = chanPrefs.getBytesLength("chan");
  if (n == sizeof(g_chan)) {
    chanPrefs.getBytes("chan", g_chan, sizeof(g_chan));
  } else {
    setDefaultChannels();
  }
  chanPrefs.end();
}
void saveChannels() {
  chanPrefs.begin("chcfg", false);
  chanPrefs.putBytes("chan", g_chan, sizeof(g_chan));
  chanPrefs.end();
}
// 現在の全チャンネル設定をアプリへ送る(接続時のUI初期化・確認用)
void broadcastChannelConfig() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    ChanConfig &c = g_chan[i];
    char buf[96];
    snprintf(buf, sizeof(buf), "CHCFGVAL:%d:%d:%04X:%04X:%d:%d:%.6f:%.4f:%s:%s",
      i, c.mode, c.pidOrDid, c.header, c.byteOffset, c.byteCount, c.scale, c.offset, c.label, c.unit);
    Serial.println(buf);
    broadcastText(String(buf));
  }
}

bool updateCanSpeed(float& outSpeed) {
  uint8_t d[5]; uint8_t len;
  if (requestPID01(0x0D, d, sizeof(d), len) && len >= 1) { outSpeed = d[0]; return true; }
  return false;
}
void updateOneExtraSignal() {
  int i = g_extraCycleIndex;
  ChanConfig &c = g_chan[i];
  if (c.pidOrDid != 0xFFFF) {
    long raw = 0; bool ok = false;
    // マルチフレーム対応により、開始バイト(byteOffset)は5バイト目以降も指定できる
    uint8_t d[ISOTP_MAX_PAYLOAD]; uint8_t len = 0;
    if (c.mode == 1) {
      ok = requestPID01((uint8_t)c.pidOrDid, d, sizeof(d), len);
    } else if (c.mode == 22) {
      uint32_t respId; uint8_t nrc;
      ok = (requestPID22(c.header, c.pidOrDid, d, sizeof(d), len, respId, nrc) == PID22_OK);
    }
    if (ok && (int)len >= (int)c.byteOffset + (int)c.byteCount) {
      for (int k = 0; k < c.byteCount; k++) raw = raw * 256 + d[c.byteOffset + k];
    } else {
      ok = false;
    }
    if (ok) g_chanVal[i] = raw * c.scale + c.offset;
  }
  g_extraCycleIndex = (g_extraCycleIndex + 1) % NUM_CHANNELS;
}

void printCanStatus() {
  twai_status_info_t info;
  if (twai_get_status_info(&info) == ESP_OK) {
    String state;
    switch (info.state) {
      case TWAI_STATE_STOPPED:    state = "STOPPED"; break;
      case TWAI_STATE_RUNNING:    state = "RUNNING"; break;
      case TWAI_STATE_BUS_OFF:    state = "BUS_OFF"; break;
      case TWAI_STATE_RECOVERING: state = "RECOVERING"; break;
      default: state = "UNKNOWN";
    }
    String s = "CANSTAT:state=" + state +
               " tx_err=" + String(info.tx_error_counter) +
               " rx_err=" + String(info.rx_error_counter) +
               " tx_failed=" + String(info.tx_failed_count) +
               " rx_missed=" + String(info.rx_missed_count) +
               " arb_lost=" + String(info.arb_lost_count) +
               " bus_err=" + String(info.bus_error_count);
    Serial.println(s);
    broadcastText(s);
  } else {
    Serial.println("CANSTAT取得失敗");
    broadcastText("CANSTAT:ERROR");
  }
}

void resetCanDriver() {
  CanLock lock; // 別タスクの照会の最中にドライバを外さない
  twai_stop();
  twai_driver_uninstall();
  delay(50);
  setupTWAI(); // 再インストール+再スタート(g_canOkもここで更新される)
  Serial.println(g_canOk ? "CANドライバをリセットしました" : "CANドライバのリセットに失敗しました");
  broadcastText(g_canOk ? "CANRESET:OK" : "CANRESET:FAIL");
}

// ============ 自己診断(NO_ACKモードでのテストフレーム送信) ============
void sendSelfTestFrame() {
  twai_message_t msg;
  msg.identifier = 0x123; // 適当なテスト用ID
  msg.extd = 0; msg.rtr = 0; msg.data_length_code = 2;
  msg.data[0] = 0xAA; msg.data[1] = 0x55;

  if (twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
    Serial.println("SELFTEST:送信成功");
    broadcastText("SELFTEST:TX_OK");
  } else {
    Serial.println("SELFTEST:送信失敗");
    broadcastText("SELFTEST:TX_FAIL");
  }
  delay(50);
  printCanStatus(); // 直後の状態を表示(RUNNINGのままかを見る)
}

// ============ GPIOレベルのTXD/RXDループバックテスト ============
// TWAIドライバ(CANプロトコル層)を使わず、D8/D9を素のGPIOとして直接
// 制御し、電気的な往復だけを確認する。ビットタイミング・ACK等の
// プロトコル要素を一切使わない、最も原始的で確実なテスト。
void runGpioLoopbackTest() {
  CanLock lock;
  twai_stop();
  twai_driver_uninstall();
  delay(30);

  pinMode(CAN_TX_PIN, OUTPUT);
  pinMode(CAN_RX_PIN, INPUT);
  delay(10);

  int passCount = 0;
  String detail = "";
  for (int i = 0; i < 5; i++) {
    digitalWrite(CAN_TX_PIN, LOW);
    delayMicroseconds(300);
    int r1 = digitalRead(CAN_RX_PIN);

    digitalWrite(CAN_TX_PIN, HIGH);
    delayMicroseconds(300);
    int r2 = digitalRead(CAN_RX_PIN);

    bool ok = (r1 == LOW && r2 == HIGH);
    if (ok) passCount++;
    detail += String(i) + ":L->" + String(r1) + "/H->" + String(r2) + (ok ? "OK " : "NG ");
  }

  Serial.println("GPIOTEST詳細: " + detail);
  String result = "GPIOTEST:" + String(passCount) + "/5 " + String(passCount == 5 ? "PASS" : "FAIL");
  Serial.println(result);
  broadcastText(result);
  broadcastText("GPIOTEST_DETAIL:" + detail);

  // TWAIドライバを元の設定で再インストール
  setupTWAI();
}

// ============ GPIOレベルのTXD/RXDループバックテスト(別ピン版、切り分け用) ============
// D8/D9固有の問題か、もっと根本的な問題かを切り分けるため、
// 全く別のGPIOペア(D0/D1)でも同じテストができるようにする
void runGpioLoopbackTestAltPins() {
  CanLock lock;
  twai_stop();
  twai_driver_uninstall();
  delay(30);

  const gpio_num_t altTx = (gpio_num_t)D0;
  const gpio_num_t altRx = (gpio_num_t)D1;

  pinMode(altTx, OUTPUT);
  pinMode(altRx, INPUT);
  delay(10);

  int passCount = 0;
  String detail = "";
  for (int i = 0; i < 5; i++) {
    digitalWrite(altTx, LOW);
    delayMicroseconds(300);
    int r1 = digitalRead(altRx);

    digitalWrite(altTx, HIGH);
    delayMicroseconds(300);
    int r2 = digitalRead(altRx);

    bool ok = (r1 == LOW && r2 == HIGH);
    if (ok) passCount++;
    detail += String(i) + ":L->" + String(r1) + "/H->" + String(r2) + (ok ? "OK " : "NG ");
  }

  Serial.println("GPIOTEST(D0/D1)詳細: " + detail);
  String result = "GPIOTEST_ALT:" + String(passCount) + "/5 " + String(passCount == 5 ? "PASS" : "FAIL") +
                  " (D0とD1をジャンパー線で直結してから実行してください)";
  Serial.println(result);
  broadcastText(result);
  broadcastText("GPIOTEST_ALT_DETAIL:" + detail);

  setupTWAI();
}

// ============ DTC(デバッグ用、シリアル専用) ============
String decodeDTC(uint8_t byteA, uint8_t byteB) {
  if (byteA == 0x00 && byteB == 0x00) return "";
  const char letters[4] = {'P','C','B','U'};
  char letter = letters[(byteA>>6)&0x03];
  uint8_t d1=(byteA>>4)&0x03, d2=byteA&0x0F, d3=(byteB>>4)&0x0F, d4=byteB&0x0F;
  char buf[6]; sprintf(buf,"%c%01X%01X%01X%01X",letter,d1,d2,d3,d4);
  return String(buf);
}
// DTC読取(Mode03)。CAN(ISO 15765-4)の応答は「43 [件数] [DTC 2バイト]×件数」で、3件以上あると
// マルチフレームになる。エンジン以外のECUも応答するので、ECUごとに組み立ててから読む。
void requestDTC() {
  Serial.println("=== DTC読取 ===");
  const uint8_t req[1] = {0x03};
  IsoTpResp r[ISOTP_MAX_ECUS];
  int n = isoTpQuery(OBD_REQUEST_ID, false, req, 1, true, r, ISOTP_MAX_ECUS, 1000, nullptr, nullptr);
  bool found = false;
  String codes = "";
  for (int e = 0; e < n; e++) {
    for (int i = 2; i + 1 < r[e].len; i += 2) { // data[0]=43、data[1]=件数、data[2]からDTC
      String c = decodeDTC(r[e].data[i], r[e].data[i + 1]);
      if (c.length() > 0) {
        Serial.println("DTC: " + c);
        found = true;
        if (codes.length() > 0) codes += ",";
        codes += c;
      }
    }
  }
  if (n == 0) { Serial.println("応答なし"); broadcastText("DTC:NORESP"); }
  else if (!found) { Serial.println("故障なし"); broadcastText("DTC:NONE"); }
  else { broadcastText("DTC:" + codes); }
}
void clearDTC() {
  Serial.println("=== DTCクリア ===");
  const uint8_t req[1] = {0x04};
  IsoTpResp r;
  if (isoTpQuery(OBD_REQUEST_ID, false, req, 1, true, &r, 1, 1000, nullptr, nullptr) >= 1) {
    Serial.println("クリア成功");
    broadcastText("DTCCLEAR:OK");
    return;
  }
  Serial.println("応答タイムアウト");
  broadcastText("DTCCLEAR:TIMEOUT");
}

// ============ BLE設定 ============
#define BLE_DEVICE_NAME   "SpeedTracer-ESP32"
#define BLE_SERVICE_UUID  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_TX_CHAR_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Notify: ESP32->App
#define BLE_RX_CHAR_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Write : App->ESP32(コマンド)

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;
bool bleDeviceConnected = false;

void processCommand(String cmd); // 前方宣言
void noteActivity(); // 前方宣言(スリープ管理用、定義は後方)

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* srv) override { bleDeviceConnected = true; noteActivity(); }
  void onDisconnect(BLEServer* srv) override {
    bleDeviceConnected = false;
    delay(200);
    srv->getAdvertising()->start();
  }
};
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    String v = ch->getValue();
    v.trim();
    if (v.length() > 0) { processCommand(v); noteActivity(); }
  }
};

void setupBle() {
  BLEDevice::init(BLE_DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
      BLE_TX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(
      BLE_RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  Serial.println("BLE: アドバタイズ開始");
}
void sendSpeedBle(float speedKmh) {
  if (!bleDeviceConnected) return;
  char buf[16]; snprintf(buf,sizeof(buf),"SPEED:%.1f",speedKmh);
  pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
  pTxCharacteristic->notify();
}
void sendExtraBle() {
  if (!bleDeviceConnected) return;
  char buf[112];
  snprintf(buf,sizeof(buf),"EXTRA:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
    g_chanVal[0],g_chanVal[1],g_chanVal[2],g_chanVal[3],g_chanVal[4],g_chanVal[5],g_chanVal[6],g_chanVal[7],g_chanVal[8],g_chanVal[9]);
  pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
  pTxCharacteristic->notify();
}

// ============ WiFi/WebSocket ============
const char* WIFI_AP_SSID     = "SpeedTracer";
const char* WIFI_AP_PASSWORD = "speed1234";
const uint16_t WS_PORT = 81;
WebServer httpServer(80);
WebSocketsServer webSocket = WebSocketsServer(WS_PORT);

// ============ ELM327エミュレーション(Torque/Car Scanner等の市販アプリ向け) ============
// 市販のWiFi版ELM327アダプター互換のTCPポート(35000)を追加で立て、
// AT初期化コマンドに適当に応答しつつ、Mode01/Mode22のPIDリクエストは
// 既存のrequestPID01/requestPID22へそのまま橋渡しする。
// CAN側の配線・トランシーバーは1個のまま(バス上は論理的な別プロトコルの追加のみ)。
const uint16_t ELM_PORT = 35000;
WiFiServer elmServer(ELM_PORT);
WiFiClient elmClient;
String elmBuf;
uint32_t g_elmHeader = OBD_REQUEST_ID; // ATSHで変更可能(Mode22で特定ECU宛てに送る際に使用)
bool g_elmExtended = false; // ATSP7等29bit拡張CANプロトコル選択時にtrue(Car Scannerはこちらを使うことがある)
bool g_elmHeadersOn = false; // ATH1で有効化。有効時は応答の先頭に応答元ECUのCAN IDを付与する(Car Scanner等が複数ECU識別に使用)

String elmHeaderPrefix(uint32_t id, uint8_t payloadLen) {
  if (!g_elmHeadersOn) return "";
  char hdrbuf[10];
  if (g_elmExtended) snprintf(hdrbuf, sizeof(hdrbuf), "%08lX", (unsigned long)id);
  else snprintf(hdrbuf, sizeof(hdrbuf), "%03lX", (unsigned long)id);
  char lenbuf[4]; snprintf(lenbuf, sizeof(lenbuf), "%02X", payloadLen);
  return String(hdrbuf) + " " + String(lenbuf) + " ";
}

String elmFormatBytes(const uint8_t* d, uint8_t len) {
  String s;
  for (int i = 0; i < len; i++) {
    if (d[i] < 0x10) s += "0";
    s += String(d[i], HEX);
    if (i < len - 1) s += " ";
  }
  s.toUpperCase();
  return s;
}

// 応答1件をELM327の書式にする。1フレームに収まる応答は従来どおり1行。
// マルチフレーム応答は実ELM327と同じ書式:
//   ATH0(ヘッダーなし): 1行目に全長(16進3桁)、以降「0: 6バイト」「1: 7バイト」...
//   ATH1(ヘッダーあり): フレームごとに「ID + 生の8バイト」
String elmFormatResp(const IsoTpResp& r) {
  if (!r.multi) return elmHeaderPrefix(r.id, (uint8_t)r.len) + elmFormatBytes(r.data, (uint8_t)r.len);
  String out;
  char buf[16];
  if (!g_elmHeadersOn) { snprintf(buf, sizeof(buf), "%03X", (unsigned)r.total); out = buf; }
  uint16_t pos = 0;
  uint8_t line = 0;
  while (pos < r.len) {
    const uint8_t chunk = (line == 0) ? 6 : 7;
    uint8_t frame[8]; uint8_t fl = 0;
    if (g_elmHeadersOn) {
      if (line == 0) { frame[fl++] = 0x10 | ((r.total >> 8) & 0x0F); frame[fl++] = r.total & 0xFF; }
      else frame[fl++] = 0x20 | (line & 0x0F);
    }
    for (uint8_t k = 0; k < chunk; k++) frame[fl++] = (pos + k < r.len) ? r.data[pos + k] : 0x00;
    pos += chunk;
    if (out.length() > 0) out += "\r";
    if (g_elmHeadersOn) {
      if (g_elmExtended) snprintf(buf, sizeof(buf), "%08lX ", (unsigned long)r.id);
      else snprintf(buf, sizeof(buf), "%03lX ", (unsigned long)r.id);
    } else {
      snprintf(buf, sizeof(buf), "%X: ", (unsigned)(line & 0x0F));
    }
    out += buf;
    out += elmFormatBytes(frame, fl);
    line++;
  }
  return out;
}

void processElmCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) { elmClient.print(">"); return; }
  String upper = cmd; upper.toUpperCase();
  String resp;
  // Car Scanner等が何を要求しているかを丸ごと可視化する(この機能の本来の目的)
  String rxLog = "ELM:RX:" + upper;
  Serial.println(rxLog);
  broadcastText(rxLog);

  if (upper.startsWith("AT")) {
    if (upper == "ATZ") {
      resp = "ELM327 v2.1";
    } else if (upper == "ATI") {
      resp = "ELM327 v2.1";
    } else if (upper == "ATRV") {
      resp = "12.6V";
    } else if (upper == "ATDP") {
      resp = "ISO 15765-4 (CAN 11/500)";
    } else if (upper == "ATDPN") {
      resp = "6";
    } else if (upper.startsWith("ATSH")) {
      // 例: ATSH700 (11bit) / ATSHDA00E0 (29bit拡張、ATSP7選択時は先頭に18を補完して0x18DA00E0とする)
      String hexPart = upper.substring(4); hexPart.trim();
      uint32_t h = (uint32_t)strtol(hexPart.c_str(), nullptr, 16);
      if (h != 0) {
        if (g_elmExtended && hexPart.length() <= 6) g_elmHeader = 0x18000000UL | h;
        else g_elmHeader = h;
      }
      resp = "OK";
    } else if (upper.startsWith("ATSP")) {
      // 例: ATSP7 / ATSPA7 → 29bit拡張CAN(ISO15765-4 CAN29/500)。ATSP6等は11bit標準に戻す
      if (upper.indexOf('7') >= 0) g_elmExtended = true;
      else if (upper.indexOf('6') >= 0) g_elmExtended = false;
      resp = "OK";
    } else if (upper == "ATH0") {
      g_elmHeadersOn = false; resp = "OK";
    } else if (upper == "ATH1") {
      g_elmHeadersOn = true; resp = "OK";
    } else {
      // ATE0/ATE1, ATL0/ATL1, ATH0/ATH1, ATS0/ATS1, ATAT.., ATCAF.. 等は
      // 動作に大きく影響しないため一律OKを返し、なるべく多くのアプリの初期化を通す
      resp = "OK";
    }
  } else {
    // PIDリクエスト(16進文字列。スペースは無視)。例: "010C"(Mode01 RPM) / "221021"(Mode22 DID 0x1021)
    String hex = upper;
    hex.replace(" ", "");
    if (hex.startsWith("01") && hex.length() >= 4) {
      uint8_t pid = (uint8_t)strtol(hex.substring(2, 4).c_str(), nullptr, 16);
      IsoTpResp results[ISOTP_MAX_ECUS];
      int n = requestPID01Multi(pid, results, ISOTP_MAX_ECUS);
      if (n == 0) {
        resp = "NO DATA";
      } else {
        resp = "";
        for (int i = 0; i < n; i++) {
          if (i > 0) resp += "\r";
          resp += elmFormatResp(results[i]);
        }
      }
    } else if (hex.startsWith("22") && hex.length() >= 6) {
      uint16_t did = (uint16_t)strtol(hex.substring(2, 6).c_str(), nullptr, 16);
      const uint8_t req[3] = {0x22, (uint8_t)((did >> 8) & 0xFF), (uint8_t)(did & 0xFF)};
      IsoTpResp r;
      uint32_t negId = 0; uint8_t nrc = 0;
      if (isoTpQuery(g_elmHeader, g_elmExtended, req, 3, false, &r, 1, 150, &negId, &nrc) >= 1) {
        resp = elmFormatResp(r);
      } else if (negId != 0) {
        char nb[4]; snprintf(nb, sizeof(nb), "%02X", nrc);
        resp = elmHeaderPrefix(negId, 3) + "7F 22 " + String(nb);
      } else {
        resp = "NO DATA";
      }
    } else {
      resp = "?"; // ELM327の「未対応コマンド」応答
    }
  }
  elmClient.print(resp + "\r>");
  String txLog = "ELM:TX:" + resp;
  Serial.println(txLog);
  broadcastText(txLog);
}

void handleElmClient() {
  if (!elmClient || !elmClient.connected()) {
    if (elmServer.hasClient()) {
      elmClient = elmServer.available();
      elmBuf = "";
      Serial.println("ELM327互換ポート: クライアント接続");
      noteActivity();
    }
    return;
  }
  while (elmClient.available()) {
    char c = elmClient.read();
    if (c == '\r' || c == '\n') {
      if (elmBuf.length() > 0) { processElmCommand(elmBuf); elmBuf = ""; }
    } else {
      elmBuf += c;
      if (elmBuf.length() > 64) elmBuf = ""; // 異常に長い入力は破棄(バッファ保護)
    }
  }
}

// ============ WiFi経由のファーム更新(OTA) ============
// SpeedTracerのWiFiにつないだブラウザで http://192.168.4.1/update を開き、.bin を送ると書き換わる。
// 前提: アプリ領域が2面あるパーティション設定で書き込まれていること(Arduino IDEの
//   ツール → Partition Scheme で「OTA」を含むもの、または XIAO ESP32S3 既定の「Default with spiffs」)。
//   1面しかない設定(Huge APP / No OTA)では、更新ページに「OTA用の領域なし」と表示される。
// 書き込みは今動いていない側の面に行い、検証が通った時だけ起動先を切り替える。途中で失敗・中断しても
// 今のファームはそのまま残る。
// 更新ページのHTML(ota_page.htmlから生成。JavaScriptを含むため、Arduino IDEのプロトタイプ自動生成が
// 誤認しないよう、生文字列 R"( )" ではなく通常の文字列で持つ)
static const char UPDATE_HTML_A[] =
  "<!doctype html>\n"
  "<html lang='ja'>\n"
  "<head>\n"
  "<meta charset='utf-8'>\n"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
  "<title>SpeedTracer ファーム更新</title>\n"
  "<style>\n"
  "body{margin:0;padding:16px;background:#0d1117;color:#e6edf3;font:16px/1.6 system-ui,sans-serif}\n"
  "h1{font-size:20px;margin:0 0 12px}\n"
  ".card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:14px;margin-bottom:14px}\n"
  "table{border-collapse:collapse;width:100%}\n"
  "td{padding:3px 0;vertical-align:top}\n"
  "td:first-child{color:#8b949e;width:8.5em}\n"
  "input[type=file]{width:100%;margin:0 0 12px;color:#e6edf3;font-size:16px}\n"
  "button{width:100%;padding:14px;font-size:17px;border:0;border-radius:8px;background:#238636;color:#fff}\n"
  "button:disabled{background:#30363d;color:#8b949e}\n"
  "#bar{height:10px;background:#30363d;border-radius:5px;overflow:hidden;margin:14px 0 8px}\n"
  "#fill{height:100%;width:0;background:#2f81f7}\n"
  "#msg{min-height:1.6em}\n"
  ".bad{color:#ff7b72}\n"
  ".ok{color:#56d364}\n"
  "ul{margin:0;padding-left:1.2em;color:#8b949e;font-size:14px}\n"
  "a{color:#58a6ff}\n"
  "</style>\n"
  "</head>\n"
  "<body>\n"
  "<h1>SpeedTracer ファーム更新</h1>\n"
  "<div class='card'><table>\n";
static const char UPDATE_HTML_B[] =
  "</table></div>\n"
  "<div class='card'>\n"
  "<input type='file' id='f' accept='.bin'>\n"
  "<button id='go' disabled>書き込む</button>\n"
  "<div id='bar'><div id='fill'></div></div>\n"
  "<div id='msg'>.bin ファイルを選んでください。</div>\n"
  "</div>\n"
  "<div class='card'><ul>\n"
  "<li>選ぶのは、Arduino IDEの「スケッチ」メニューにある Export Compiled Binary(コンパイルしたバイナリの出力)でできる <b>speed_tracer_production.ino.bin</b> です。</li>\n"
  "<li>名前に merged / bootloader / partitions が付くファイルは使えません。</li>\n"
  "<li>書き込み中は車速などの送信が止まります。終わると自動で再起動します。</li>\n"
  "<li>起動しなくなった場合は、USBケーブルで書き直せば元に戻せます。</li>\n"
  "</ul></div>\n"
  "<p><a href='/'>← アプリに戻る</a></p>\n"
  "<script>\n"
  "var MAXSZ=";
static const char UPDATE_HTML_C[] =
  ";\n"
  "var f=document.getElementById('f'),go=document.getElementById('go'),msg=document.getElementById('msg'),fill=document.getElementById('fill');\n"
  "function say(t,c){msg.textContent=t;msg.className=c||'';}\n"
  "if(!MAXSZ){f.disabled=true;say('このファームにはOTA用の領域がないため、ここからは書き込めません。','bad');}\n"
  "f.onchange=function(){\n"
  "  go.disabled=true;fill.style.width='0';\n"
  "  var x=f.files[0];\n"
  "  if(!x){say('.bin ファイルを選んでください。');return;}\n"
  "  if(x.size>MAXSZ){say('ファイルが大きすぎます('+Math.round(x.size/1024)+' KB)。','bad');return;}\n"
  "  var r=new FileReader();\n"
  "  r.onerror=function(){say('ファイルを読めませんでした。','bad');};\n"
  "  r.onload=function(){\n"
  "    var b=new Uint8Array(r.result);\n"
  "    if(b.length<36||b[0]!=0xE9){say('ESP32のファームウェアではありません。','bad');return;}\n"
  "    if(!(b[32]==0x32&&b[33]==0x54&&b[34]==0xCD&&b[35]==0xAB)){say('アプリ本体の .bin ではありません。merged / bootloader / partitions の付くファイルは使えません。','bad');return;}\n"
  "    say(x.name+'('+Math.round(x.size/1024)+' KB)を書き込めます。');\n"
  "    go.disabled=false;\n"
  "  };\n"
  "  r.readAsArrayBuffer(x.slice(0,36));\n"
  "};\n"
  "go.onclick=function(){\n"
  "  var x=f.files[0];\n"
  "  if(!x)return;\n"
  "  go.disabled=true;f.disabled=true;\n"
  "  var fd=new FormData();\n"
  "  fd.append('firmware',x,x.name);\n"
  "  var q=new XMLHttpRequest();\n"
  "  q.open('POST','/update');\n"
  "  q.upload.onprogress=function(e){\n"
  "    if(!e.lengthComputable)return;\n"
  "    var p=Math.round(e.loaded*100/e.total);\n"
  "    fill.style.width=p+'%';\n"
  "    say(p<100?('送信中 '+p+'%。電源を切らないでください。'):'確認しています。電源を切らないでください。');\n"
  "  };\n"
  "  q.onload=function(){\n"
  "    if(q.status==200&&q.responseText.indexOf('OK')==0){\n"
  "      fill.style.width='100%';\n"
  "      say('書き込みました。再起動しています。10秒ほど待ってからWiFiにつなぎ直してください。','ok');\n"
  "    }else{\n"
  "      say('書き込めませんでした: '+q.responseText.replace(/^NG:/,''),'bad');\n"
  "      go.disabled=false;f.disabled=false;\n"
  "    }\n"
  "  };\n"
  "  q.onerror=function(){\n"
  "    say('通信が切れました。WiFiにつなぎ直してこのページを開き、バージョンが変わったか確認してください。','bad');\n"
  "    f.disabled=false;\n"
  "  };\n"
  "  q.send(fd);\n"
  "};\n"
  "</script>\n"
  "</body>\n"
  "</html>\n";
bool   g_otaBegun = false;  // Update.begin()済みか
bool   g_otaOk    = false;  // 書き込みと検証が通ったか
String g_otaMsg;            // 失敗した時の理由(空でなければ失敗)

// 受け取ったファイルの先頭を見て、このボードで動くアプリ本体かを確かめる。問題なければnullptr、あれば理由。
const char* otaCheckImage(const uint8_t* b, size_t n) {
  if (n < 36) return "ファイルが小さすぎます";
  if (b[0] != 0xE9) return "ESP32のファームウェアではありません";
#ifdef CONFIG_IDF_FIRMWARE_CHIP_ID
  if ((uint16_t)(b[12] | (b[13] << 8)) != CONFIG_IDF_FIRMWARE_CHIP_ID) return "別の種類のESP32用のファームウェアです";
#endif
  // アプリ本体は先頭から32バイト目にアプリ情報の目印(0xABCD5432)を持つ。
  // merged.bin や bootloader.bin も先頭は0xE9だが、この目印は無い(書くと起動しなくなるので弾く)
  if (!(b[32] == 0x32 && b[33] == 0x54 && b[34] == 0xCD && b[35] == 0xAB))
    return "アプリ本体の .bin ではありません(merged / bootloader / partitions の付くファイルは使えません)";
  return nullptr;
}

void handleUpdatePage() {
  noteActivity();
  const esp_partition_t* run = esp_ota_get_running_partition();
  const uint32_t maxSz = ESP.getFreeSketchSpace();   // 書き込み先(もう一方の面)の大きさ。面が無ければ0
  String rows;
  rows += "<tr><td>バージョン</td><td>" FW_VERSION "</td></tr>";
  rows += "<tr><td>ビルド日時</td><td>" __DATE__ " " __TIME__ "</td></tr>";
  rows += "<tr><td>実行中の領域</td><td>";
  rows += (run ? run->label : "?");
  rows += "</td></tr><tr><td>今のサイズ</td><td>";
  rows += String(ESP.getSketchSize() / 1024);
  rows += " KB</td></tr>";
  if (maxSz > 0) {
    rows += "<tr><td>書き込める上限</td><td>";
    rows += String(maxSz / 1024);
    rows += " KB</td></tr>";
  } else {
    rows += "<tr><td>OTA用の領域</td><td class='bad'>なし(Partition SchemeをOTA対応のものにしてUSBで書き直してください)</td></tr>";
  }
  String html;
  html.reserve(sizeof(UPDATE_HTML_A) + sizeof(UPDATE_HTML_B) + sizeof(UPDATE_HTML_C) + rows.length() + 16);
  html += UPDATE_HTML_A;
  html += rows;
  html += UPDATE_HTML_B;
  html += String(maxSz);
  html += UPDATE_HTML_C;
  httpServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  httpServer.send(200, "text/html; charset=utf-8", html);
}

// ファイル本体が届くたびに呼ばれる(受信中はloop()がここで止まるので、CANの照会も止まる)
void handleUpdateUpload() {
  HTTPUpload& up = httpServer.upload();
  noteActivity();   // 送信に時間がかかっても途中でスリープに入らないようにする
  if (up.status == UPLOAD_FILE_START) {
    if (g_otaBegun) Update.abort();
    g_otaBegun = false; g_otaOk = false; g_otaMsg = "";
    Serial.println("OTA: 受信開始 " + up.filename);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_otaMsg.length() > 0) return;   // すでに失敗している。残りは読み捨てる
    if (!g_otaBegun) {
      const char* why = otaCheckImage(up.buf, up.currentSize);
      if (why) { g_otaMsg = why; return; }
      if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
        g_otaMsg = String("書き込みを始められません(") + Update.errorString() + ")";
        return;
      }
      g_otaBegun = true;
    }
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      g_otaMsg = String("書き込みに失敗しました(") + Update.errorString() + ")";
      Update.abort();
      g_otaBegun = false;
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_otaMsg.length() == 0) {
      if (!g_otaBegun) {
        g_otaMsg = "ファイルが空です";
      } else if (Update.end(true)) {
        g_otaOk = true;
      } else {
        g_otaMsg = String("検証に失敗しました(") + Update.errorString() + ")";
      }
      g_otaBegun = false;
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (g_otaBegun) Update.abort();
    g_otaBegun = false;
    g_otaMsg = "送信が途中で切れました";
  }
}

// 受信がすべて終わった後に呼ばれる。結果を返し、成功していれば再起動して新しいファームに切り替える
void handleUpdateDone() {
  httpServer.sendHeader("Connection", "close");
  if (g_otaOk) {
    httpServer.send(200, "text/plain; charset=utf-8", "OK");
    Serial.println("OTA: 書き込み成功。再起動します");
    broadcastText("OTA:OK REBOOT");
    delay(800);   // 応答がブラウザに届くのを待つ
    ESP.restart();
  } else {
    if (g_otaMsg.length() == 0) g_otaMsg = "ファイルが届きませんでした";
    Serial.println("OTA: 失敗 " + g_otaMsg);
    httpServer.send(200, "text/plain; charset=utf-8", "NG:" + g_otaMsg);
  }
}

void handleRoot() {
  httpServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  httpServer.send_P(200, "text/html", INDEX_HTML);
}

// ============ ディープスリープ(バッテリー上がり対策) ============
// DLC(OBD2)のpin16(常時電源)から給電しているため、車が使われていない間もつけっぱなしだと
// バッテリー上がりを起こす。接続が一定時間無い場合はディープスリープに入り、そのまま
// 何もせず眠り続ける(タイマー等による自動起床はしない)。復帰は物理的な抜き差し(電源再投入)
// による通常のリセットのみ。スリープ中の消費電流はごく僅か(数µA程度)。
unsigned long g_lastActivityMs = 0; // 最後に何らかの接続/コマンドがあった時刻(millis基準)
int g_wsClientCount = 0;            // 現在接続中のWebSocketクライアント数

const unsigned long SLEEP_IDLE_TIMEOUT_MS = 15UL * 60 * 1000; // 接続が無い状態がこの時間続いたらスリープ(15分)

void noteActivity() { g_lastActivityMs = millis(); }

void enterDeepSleep() {
  Serial.println("SLEEP: 接続なしのためディープスリープに入ります。復帰は電源の抜き差しが必要です");
  Serial.flush();
  esp_deep_sleep_start(); // 起床用タイマー等は設定しない = 電源再投入(リセット)のみが復帰手段
}

void maybeDeepSleep() {
  bool anyConnected = (g_wsClientCount > 0) || bleDeviceConnected || (elmClient && elmClient.connected());
  if (anyConnected) { noteActivity(); return; }
  if (millis() - g_lastActivityMs > SLEEP_IDLE_TIMEOUT_MS) enterDeepSleep();
}

void onWsEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    g_wsClientCount++;
    noteActivity();
  } else if (type == WStype_DISCONNECTED) {
    if (g_wsClientCount > 0) g_wsClientCount--;
  } else if (type == WStype_TEXT) {
    String cmd = String((char*)payload).substring(0, length);
    cmd.trim();
    if (cmd.length() > 0) { processCommand(cmd); noteActivity(); }
  }
}
void setupWifi() {
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  Serial.print("WiFi: AP起動 IP="); Serial.println(WiFi.softAPIP());
  httpServer.on("/", HTTP_GET, handleRoot);
  httpServer.on("/update", HTTP_GET, handleUpdatePage);
  httpServer.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  httpServer.begin();
  webSocket.begin();
  webSocket.onEvent(onWsEvent);
  elmServer.begin();
  Serial.print("ELM327互換ポート(Torque/Car Scanner等向け): port="); Serial.println(ELM_PORT);
}
void sendSpeedWifi(float speedKmh) {
  char buf[16]; snprintf(buf,sizeof(buf),"SPEED:%.1f",speedKmh);
  webSocket.broadcastTXT(buf);
}
void sendExtraWifi() {
  char buf[112];
  snprintf(buf,sizeof(buf),"EXTRA:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
    g_chanVal[0],g_chanVal[1],g_chanVal[2],g_chanVal[3],g_chanVal[4],g_chanVal[5],g_chanVal[6],g_chanVal[7],g_chanVal[8],g_chanVal[9]);
  webSocket.broadcastTXT(buf);
}

String getResetReasonString() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON(通常の電源投入)";
    case ESP_RST_EXT:       return "EXT(外部リセットピン)";
    case ESP_RST_SW:        return "SW(ソフトウェアリセット)";
    case ESP_RST_PANIC:     return "PANIC(プログラムクラッシュ) ★要注意";
    case ESP_RST_INT_WDT:   return "INT_WDT(割り込みウォッチドッグ) ★要注意";
    case ESP_RST_TASK_WDT:  return "TASK_WDT(タスクウォッチドッグ、loop()がハングした疑い) ★要注意";
    case ESP_RST_WDT:       return "WDT(その他ウォッチドッグ) ★要注意";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP(スリープからの復帰)";
    case ESP_RST_BROWNOUT:  return "BROWNOUT(電圧不足によるリセット) ★★電源系を疑ってください";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN(" + String((int)reason) + ")";
  }
}

void printHeapInfo() {
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();   // 起動から今までで最も少なかった時の値
  uint32_t maxAlloc = ESP.getMaxAllocHeap();     // 一度に確保できる最大の連続ブロックサイズ
  float fragPct = (freeHeap > 0) ? (100.0f - (100.0f * maxAlloc / freeHeap)) : 0;

  String s = "HEAP:free=" + String(freeHeap) + "B min=" + String(minFreeHeap) +
             "B maxAlloc=" + String(maxAlloc) + "B frag=" + String(fragPct, 1) +
             "% uptime=" + String(millis()/1000) + "s";
  Serial.println(s);
  broadcastText(s);
}

// ============ 汎用ブロードキャスト(USB/BLE/WiFi全部に同じ文字列を送る) ============
// DTC結果など、通常のSPEED:/EXTRA:以外の応答をアプリ側に返すために使用
void broadcastText(String s) {
  if (bleDeviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue((uint8_t*)s.c_str(), s.length());
    pTxCharacteristic->notify();
  }
  webSocket.broadcastTXT(s);
}

// ============ コマンド処理(共通、USB/BLE/WiFiどこからでも呼ばれる) ============
void printStatus() {
  String s1 = "STATUS:SRC=" + String(g_src==SRC_CAN?"CAN":"ANALOG") +
              " CANMODE=" + String(g_canDetail==CAN_ALL?"ALL":"REP") +
              " CAN_HW=" + String(g_canOk?"OK":"NG") +
              " ANALOG_HW=" + String(g_adsOk?"OK":"NG") +
              " BAUD=" + String(g_canBaud) + "kbps" +
              " MODE=" + String(g_listenOnly?"LISTEN_ONLY":"NORMAL");
  String s2 = "STATUS:CAL V0=" + String(g_calV0,3) + "V VMAX=" + String(g_calVMax,3) +
              "V SPEEDMAX=" + String(g_calSpeedMax,1) + "km/h";
  String s3 = "STATUS:LASTRESET=" + getResetReasonString();
  String s4 = "FWINFO:ver=" FW_VERSION " build=" __DATE__ " " __TIME__ " ota=" +
              String(ESP.getFreeSketchSpace() > 0 ? "OK" : "NG(OTA用の領域なし)");
  Serial.println(s1); Serial.println(s2); Serial.println(s3); Serial.println(s4);
  broadcastText(s1); broadcastText(s2); broadcastText(s3); broadcastText(s4);
}
void printRawAnalog() {
  Serial.print("分圧後電圧: "); Serial.print(g_lastDividedVoltage,3); Serial.print("V  ");
  Serial.print("センサー実電圧: "); Serial.print(g_lastActualVoltage,3); Serial.println("V");
}
void processCommand(String cmd) {
  if (cmd == "SRC:CAN") {
    g_src = SRC_CAN; saveSettings();
    Serial.println("SRCをCANに切替");
  } else if (cmd == "SRC:ANALOG") {
    g_src = SRC_ANALOG; saveSettings();
    Serial.println("SRCをANALOGに切替");
  } else if (cmd == "CANMODE:ALL") {
    g_canDetail = CAN_ALL; saveSettings();
    Serial.println("CANMODEをALLに切替");
  } else if (cmd == "CANMODE:REP") {
    g_canDetail = CAN_REP; saveSettings();
    Serial.println("CANMODEをREPに切替");
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "canstat") {
    printCanStatus();
  } else if (cmd == "canreset") {
    resetCanDriver();
  } else if (cmd == "BAUD:500") {
    g_canBaud = 500; saveSettings();
    Serial.println("通信速度を500kbpsに変更、再初期化します");
    broadcastText("BAUD:500 SET");
    resetCanDriver();
  } else if (cmd == "BAUD:250") {
    g_canBaud = 250; saveSettings();
    Serial.println("通信速度を250kbpsに変更、再初期化します");
    broadcastText("BAUD:250 SET");
    resetCanDriver();
  } else if (cmd == "LISTEN:ON") {
    g_listenOnly = true; saveSettings();
    Serial.println("リスニングオンリーモードON(送信しません)、再初期化します");
    broadcastText("LISTEN:ON SET");
    resetCanDriver();
  } else if (cmd == "LISTEN:OFF") {
    g_listenOnly = false; saveSettings();
    Serial.println("通常モードに戻します、再初期化します");
    broadcastText("LISTEN:OFF SET");
    resetCanDriver();
  } else if (cmd == "SELFTEST:ON") {
    g_selfTest = true; // NVSには保存しない(診断専用の一時モード)
    Serial.println("自己診断モード(NO_ACK)ON、再初期化します");
    broadcastText("SELFTEST:ON SET");
    resetCanDriver();
  } else if (cmd == "SELFTEST:OFF") {
    g_selfTest = false;
    Serial.println("自己診断モードOFF、通常設定で再初期化します");
    broadcastText("SELFTEST:OFF SET");
    resetCanDriver();
  } else if (cmd == "SELFTEST:SEND") {
    sendSelfTestFrame();
  } else if (cmd == "gpiotest") {
    runGpioLoopbackTest();
  } else if (cmd == "gpiotest2") {
    runGpioLoopbackTestAltPins();
  } else if (cmd == "heap") {
    printHeapInfo();
  } else if (cmd == "SLEEP:STATUS") {
    bool anyConnected = (g_wsClientCount > 0) || bleDeviceConnected || (elmClient && elmClient.connected());
    unsigned long idleFor = millis() - g_lastActivityMs;
    String s = "SLEEPSTATUS:connected=" + String(anyConnected ? "1" : "0")
      + " idleMs=" + String(idleFor) + " timeoutMs=" + String(SLEEP_IDLE_TIMEOUT_MS);
    Serial.println(s);
    broadcastText(s);
  } else if (cmd == "SLEEP:NOW") {
    // 動作確認用: タイムアウトを待たず即座にディープスリープへ入る
    Serial.println("SLEEP: 手動コマンドによりディープスリープへ入ります");
    Serial.flush();
    broadcastText("SLEEP:ENTERING");
    delay(200); // broadcastが送信バッファに乗る猶予
    enterDeepSleep();
  } else if (cmd == "CHCFG:GET") {
    broadcastChannelConfig();
  } else if (cmd == "CHCFG:RESET") {
    setDefaultChannels();
    saveChannels();
    Serial.println("CHCFG:RESET OK");
    broadcastText("CHCFG:RESET OK");
    broadcastChannelConfig();
  } else if (cmd.startsWith("CHCFG:") && cmd.indexOf(':', 6) > 0) {
    // 書式: CHCFG:<idx>:<mode>:<pidOrDidHex>:<headerHex>:<byteOffset>:<byteCount>:<scale>:<offset>:<label>:<unit>
    String rest = cmd.substring(6);
    int found = 0;
    String parts[10];
    int startIdx = 0;
    for (int i = 0; i <= rest.length() && found < 10; i++) {
      if (i == rest.length() || rest[i] == ':') {
        parts[found++] = rest.substring(startIdx, i);
        startIdx = i + 1;
      }
    }
    if (found >= 9) {
      int idx = parts[0].toInt();
      if (idx >= 0 && idx < NUM_CHANNELS) {
        ChanConfig &c = g_chan[idx];
        c.mode = (uint8_t)parts[1].toInt();
        c.pidOrDid = (uint16_t)strtol(parts[2].c_str(), nullptr, 16);
        c.header = (uint16_t)strtol(parts[3].c_str(), nullptr, 16);
        c.byteOffset = (uint8_t)parts[4].toInt();
        c.byteCount = (uint8_t)parts[5].toInt();
        c.scale = parts[6].toFloat();
        c.offset = parts[7].toFloat();
        parts[8].toCharArray(c.label, sizeof(c.label));
        if (found >= 10) parts[9].toCharArray(c.unit, sizeof(c.unit));
        saveChannels();
        String s = "CHCFG:" + String(idx) + " SET OK";
        Serial.println(s);
        broadcastText(s);
      } else {
        broadcastText("CHCFG:ERR 不正なチャンネル番号");
      }
    } else {
      broadcastText("CHCFG:ERR 書式は CHCFG:<idx>:<mode>:<pid/did>:<header>:<byteOffset>:<byteCount>:<scale>:<offset>:<label>:<unit>");
    }
  } else if (cmd.startsWith("PID:")) {
    uint8_t pid = (uint8_t)strtol(cmd.substring(4).c_str(), nullptr, 16);
    uint8_t d[ISOTP_MAX_PAYLOAD]; uint8_t len;
    if (requestPID01(pid, d, sizeof(d), len)) {
      String s = "PIDRESULT:0x" + String(pid, HEX) + " len=" + String(len) + " data=";
      for (int i = 0; i < len; i++) { s += String(d[i], HEX) + " "; }
      Serial.println(s);
      broadcastText(s);
    } else {
      String s = "PIDRESULT:0x" + String(pid, HEX) + " NORESP(応答なし、対応PIDでない可能性)";
      Serial.println(s);
      broadcastText(s);
    }
  } else if (cmd.startsWith("PID22:")) {
    // 書式: PID22:<ヘッダー16進>:<DID16進> 例: PID22:700:1F87
    String rest = cmd.substring(6);
    int sep = rest.indexOf(':');
    if (sep < 0) {
      String s = "PID22RESULT:ERR 書式は PID22:<ヘッダー>:<DID> 例:PID22:700:1F87";
      Serial.println(s); broadcastText(s);
    } else {
      uint32_t hdr = (uint32_t)strtol(rest.substring(0, sep).c_str(), nullptr, 16);
      uint16_t did = (uint16_t)strtol(rest.substring(sep + 1).c_str(), nullptr, 16);
      uint8_t d[ISOTP_MAX_PAYLOAD]; uint8_t len; uint32_t respId; uint8_t nrc;
      Pid22Status st = requestPID22(hdr, did, d, sizeof(d), len, respId, nrc);
      String tag = "PID22RESULT:hdr=0x" + String(hdr, HEX) + " did=0x" + String(did, HEX) + " ";
      if (st == PID22_OK) {
        String s = tag + "resp=0x" + String(respId, HEX) + " len=" + String(len) + " data=";
        for (int i = 0; i < len; i++) { s += String(d[i], HEX) + " "; }
        Serial.println(s); broadcastText(s);
      } else if (st == PID22_NEGATIVE) {
        String s = tag + "resp=0x" + String(respId, HEX) + " NEGATIVE(NRC=0x" + String(nrc, HEX) + ", 対応DIDでない可能性)";
        Serial.println(s); broadcastText(s);
      } else {
        String s = tag + "NORESP(応答なし。ヘッダー違いの可能性あり)";
        Serial.println(s); broadcastText(s);
      }
    }
  } else if (cmd == "i2cscan") {
    String s = "I2CSCAN:";
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        s += "0x" + String(addr, HEX) + " ";
        found++;
      }
    }
    if (found == 0) s += "デバイスなし(配線・電源を確認)";
    Serial.println(s);
    broadcastText(s);
  } else if (cmd == "raw") {
    printRawAnalog();
  } else if (cmd.startsWith("CAL:V0=")) {
    g_calV0 = cmd.substring(7).toFloat(); saveSettings();
    Serial.print("CAL V0を"); Serial.print(g_calV0,3); Serial.println("Vに設定");
  } else if (cmd.startsWith("CAL:VMAX=")) {
    g_calVMax = cmd.substring(9).toFloat(); saveSettings();
    Serial.print("CAL VMAXを"); Serial.print(g_calVMax,3); Serial.println("Vに設定");
  } else if (cmd.startsWith("CAL:SPEEDMAX=")) {
    g_calSpeedMax = cmd.substring(13).toFloat(); saveSettings();
    Serial.print("CAL SPEEDMAXを"); Serial.print(g_calSpeedMax,1); Serial.println("km/hに設定");
  } else if (cmd == "dtc") {
    requestDTC();
  } else if (cmd == "clear") {
    clearDTC();
  } else if (cmd == "log on") {
    g_rawLogEnabled = true; Serial.println("生ログ有効化");
  } else if (cmd == "log off") {
    g_rawLogEnabled = false; Serial.println("生ログ停止");
  }
}
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() > 0) { processCommand(cmd); noteActivity(); }
}

// ============ メイン処理 ============
const unsigned long SEND_INTERVAL_MS = 100;
unsigned long lastSendTime = 0;
float lastKnownSpeed = 0;
unsigned long lastHeapReportTime = 0;
const unsigned long HEAP_REPORT_INTERVAL_MS = 30000; // 30秒ごとに自動報告

void setup() {
  Serial.begin(115200);
  delay(500);
  g_canMutex = xSemaphoreCreateMutex(); // BLE/WiFiを始める前に作る

  Serial.println("=== リセット理由: " + getResetReasonString() + " ===");

  loadSettings();
  loadChannels();
  Serial.println("=== 車速取り込みツール 本番仕様 起動 ===");
  printStatus();

  Wire.begin(PIN_SDA, PIN_SCL);
  g_adsOk = ads.begin(0x48, &Wire);
  if (g_adsOk) { ads.setGain(GAIN_ONE); Serial.println("アナログ(ADS1015): OK"); }
  else Serial.println("アナログ(ADS1015): 初期化失敗(CANのみ利用可)");

  setupBle();
  setupWifi();
  setupTWAI();
  g_lastActivityMs = millis(); // 起動直後は必ずここからアイドル時間を数え始める

  Serial.println("コマンド: SRC:CAN / SRC:ANALOG / CANMODE:ALL / CANMODE:REP / status / dtc / clear / log on / log off / SLEEP:STATUS / SLEEP:NOW");
}

void loop() {
  maybeDeepSleep();
  handleSerialCommands();
  httpServer.handleClient();
  webSocket.loop();
  handleElmClient();

  // 30秒ごとに空きメモリ/稼働時間を自動報告(長時間動作時の異常検知用)
  if (millis() - lastHeapReportTime >= HEAP_REPORT_INTERVAL_MS) {
    lastHeapReportTime = millis();
    printHeapInfo();
  }

  // 生ログ用: PIDリクエストの成否に関係なく、常にキューをドレインして記録
  // (LISTEN ONLYモードでは送信が失敗するため、これが無いとログが一切出ない)
  // ※バス流量が多いと際限なく処理し続けてloop()が専有されるのを防ぐため、
  //   1回のloop()で処理するフレーム数に上限を設ける(残りは次回以降で処理)
  // 別タスク(BLE経由のコマンド)が照会中の時は、その応答を横取りしないよう今回は見送る
  if (g_rawLogEnabled && g_canMutex && xSemaphoreTake(g_canMutex, 0) == pdTRUE) {
    twai_message_t rxMsg;
    int drainCount = 0;
    while (drainCount < 15 && twai_receive(&rxMsg, 0) == ESP_OK) {
      printRawFrame(rxMsg);
      drainCount++;
    }
    xSemaphoreGive(g_canMutex);
  }

  unsigned long now = millis();
  if (now - lastSendTime < SEND_INTERVAL_MS) return;
  lastSendTime = now;

  float speedKmh;
  bool extraUpdatedThisTick = false;

  if (g_src == SRC_CAN) {
    if (updateCanSpeed(speedKmh)) lastKnownSpeed = speedKmh;
    // CAN全部モードの時だけ、毎周1つずつ追加信号も取得(車速の更新頻度を落とさないため)
    if (g_canDetail == CAN_ALL) {
      updateOneExtraSignal();
      extraUpdatedThisTick = true;
    }
  } else {
    float a = readAnalogSpeed();
    if (a >= 0) lastKnownSpeed = a;
    // アナログ選択時はCANへ一切送信しない(未配線でも安全)
  }

  Serial.print("SPEED:");
  Serial.println(lastKnownSpeed, 1);
  sendSpeedBle(lastKnownSpeed);
  sendSpeedWifi(lastKnownSpeed);

  if (g_src == SRC_CAN && g_canDetail == CAN_ALL && extraUpdatedThisTick) {
    Serial.print("EXTRA:");
    for (int i = 0; i < NUM_CHANNELS; i++) {
      Serial.print(g_chanVal[i], 3);
      if (i < NUM_CHANNELS - 1) Serial.print(",");
    }
    Serial.println();
    sendExtraBle();
    sendExtraWifi();
  }
}
