#define e_lcd 0x27                        // endereço do display.
#define e_reles 0x20                      // endereço do PCF8574.
#define PCF8574_INITIAL_VALUE 0b00000000  // 76543210 = 0b00000000 ... 0b11111111 -> 0 ... 255
#define sdcard_pin 27
#define bdir_pin 17  // 17 bugado usando dac1
#define bfun_pin 16
#define besq_pin 4
#define dac1_pin 26
//#define dac2_pin 25
#define POWER_FAIL_PIN 25
#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // pin number is specific to your esp32 board
#endif
#define cs_t1 15      // chip select tela 1
#define cs_t2 12      // chip select tela 2
#define cs_t3 5       // chip select tela 3
#define cs_1256 13    // ads1256
#define drdy_1256 14  // ads1256
#define e_1115A 0x48  //gnd
#define e_1115B 0x49  //vdd
#define e_1115C 0x4b  //scl
#define e_1115D 0x4a  //sda
#define W25Q16_CS_PIN 32
#define hspi2_pin 33

#include "ui_data.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>
#include <PCF8574.h>
#include <RTClib.h>
#include <ADS1115_WE.h>
#include <ADS1256.h>
#include <CircularBuffer.hpp>
#include <ArduinoJson.h>
#include <LittleFS.h>   // Para o fallback interno (na flash do ESP32)
#include <SPIMemory.h>  // 2. O DRIVER de baixo nível (de Marzogh)
#undef ID
#include <Preferences.h>        // Para salvar o ponteiro de escrita
#include <WiFi.h>               // <-- ADICIONE
#include <DNSServer.h>          // <-- ADICIONE
#include <ESPAsyncWebServer.h>  // <-- ADICIONE
#include <esp_task_wdt.h>       // <-- ADICIONE ESTA LINHA
//--- Classes

// --- INÍCIO: Definição da Classe VirtLCD ---

// Esta classe "envolve" o LCD real
// Ela replica as funções que usamos, mas também armazena o estado
// para enviar via WebSocket.

class VirtLCD {
private:
  LiquidCrystal_I2C &_real_lcd;  // Referência para o LCD físico
  AsyncWebSocket &_ws;           // <-- ADICIONE ESTA LINHA
  char _line0[17];               // Buffer da linha 0 (16 chars + NUL)
  char _line1[17];               // Buffer da linha 1 (16 chars + NUL)
  uint8_t _cur_x;
  uint8_t _cur_y;
  bool _backlight;
  bool _dirty;  // Flag para enviar WS update

  /**
   * @brief Função interna para escrever um char no buffer virtual
   */
  void virtualWrite(char c) {
    char *target_buffer = (_cur_y == 0) ? _line0 : _line1;
    if (_cur_x < 16) {
      target_buffer[_cur_x] = c;
      _cur_x++;
    }
  }

public:
  // Construtor: Recebe o objeto do LCD real
  VirtLCD(LiquidCrystal_I2C &physical_lcd, AsyncWebSocket &websocket)
    : _real_lcd(physical_lcd), _ws(websocket) {
    _cur_x = 0;
    _cur_y = 0;
    _backlight = true;
    _dirty = false;
    memset(_line0, ' ', 16);
    _line0[16] = 0;
    memset(_line1, ' ', 16);
    _line1[16] = 0;
  }

  // --- Funções Wrappers ---

  void init() {
    _real_lcd.init();
    this->setBacklight(HIGH);  // Garante que o estado virtual e real comecem iguais
    this->clear();             // Garante que os buffers virtuais comecem limpos
  }

  void clear() {
    _real_lcd.clear();
    memset(_line0, ' ', 16);
    memset(_line1, ' ', 16);
    _cur_x = 0;
    _cur_y = 0;
    _dirty = true;
  }

  void setCursor(uint8_t x, uint8_t y) {
    _real_lcd.setCursor(x, y);
    _cur_x = x;
    _cur_y = (y > 0) ? 1 : 0;
  }

  void setBacklight(uint8_t val) {
    _real_lcd.setBacklight(val);
    _backlight = (val == HIGH);
    _dirty = true;  // Atualiza o WS sobre o backlight
  }

  // Função "mágica" (template) para capturar todos os tipos de print
  template<typename T>
  size_t print(T msg) {
    size_t len = _real_lcd.print(msg);

    // Agora, replica a escrita no buffer virtual
    String s = String(msg);
    for (int i = 0; i < s.length(); i++) {
      virtualWrite(s[i]);
    }
    _dirty = true;
    return len;
  }

  // Captura a escrita de bytes (ex: caracteres customizados)
  size_t write(uint8_t c) {
    size_t len = _real_lcd.write(c);
    virtualWrite((char)c);  // Trata como um char no buffer virtual
    _dirty = true;
    return len;
  }

  // --- Funções do WebSocket ---

  /**
   * @brief Envia o estado atual do LCD para os clientes WS, se houver mudança.
   */
  void sendUpdate() {
    if (!_dirty) return;  // Nada mudou

    String json = "{\"type\":\"lcd\", \"line0\":\"";
    json += _line0;
    json += "\", \"line1\":\"";
    json += _line1;
    json += "\", \"bl\":";  // 'bl' = backlight
    json += _backlight ? "true" : "false";
    json += "}";

    _ws.textAll(json);
    _dirty = false;
  }

  /**
   * @brief Força um envio imediato (para novos clientes).
   */
  void forceSendUpdate() {
    _dirty = true;
    sendUpdate();
  }
};

// --- FIM: Definição da Classe VirtLCD ---

//--- Variaveis ---

//ext vars

struct State {
  bool a_run;          //rodando?
  unsigned int n_run;  //numero do run
  double t_run;        //tempo absoluto do run
  bool r[8];           // estado reles
};

State st;

struct Config {
  int s_time;              //intervalo de amostragem
  int s_num;               //numero de amostras
  bool a_1256;             // ADS1256 ativo?
  bool a_1115A;            // ADS1115-A ativo?
  bool a_1115B;            // ADS1115-B ativo?
  bool a_1115C;            // ADS1115-C ativo?
  bool a_1115D;            // ADS1115-D ativo?
  bool GFX_0;              // Tela 0 ativa?
  bool GFX_1;              // Tela 2 ativa?
  bool GFX_2;              // Tela 3 ativa?
  bool time_abs;           // log tempo abs
  bool time_abs_raw;       // log tempo abs raw
  bool time_rel_h;         // log tempo rel em horas
  bool time_rel_s;         // log tempo rel em s
  bool time_rel_raw;       // log tempo rel raw
  char loop_title[8][64];  // sX titulo
  bool loop_a[8];          // sX ativo = true passivo = false
  bool loop_log_raw[8];    // log raw
  bool loop_log_4_20[8];   // log 4-20mA
  bool loop_log_conv[8];   // log conv
  double loop_min[8];      // Conv min
  double loop_max[8];      // Conv max
  double loop_cal[8][2];   // Calib a+bx 0 = a 1 = b
  char volt_title[8][64];  // sX titulo
  bool volt_a[8];          // ch 5V = true 12V = false
  bool volt_log_raw[8];    // log raw
  bool volt_log_conv[8];   // log conv
  double volt_cal[8][4];   // Calib 12V: a+bx 0 = a 1 = b;5V: a+bx 2 = a 3 = b
};

Config cfg;

// --- INÍCIO: Estrutura de Status do Sistema ---
struct SystemStatus {
  bool lcd = false;
  bool sd_card = false;
  bool state_file = false;
  bool config_file = false;
  bool reles = false;
  bool rtc = false;
  bool a_1115A = false;
  bool a_1115B = false;
  bool a_1115C = false;
  bool a_1115D = false;
  bool a_1256 = false;
  bool gfx0 = false;
  bool gfx1 = false;
  bool gfx2 = false;
  bool ext_flash = false;
  bool int_flash = false;
};
SystemStatus sysStatus;
// --- FIM: Estrutura de Status do Sistema ---

//aux

int n_reboot = 0;

// --- INÍCIO: Variáveis do WebServer e AP ---
const char *ssid = "ePAC_Wifi";  // Nome da rede Wi-Fi
AsyncWebServer server(80);       // Cria o objeto do servidor na porta 80
DNSServer dnsServer;             // Cria o objeto do DNS Server
TaskHandle_t h_WebServerTask;    // Handle para nossa tarefa do Core 0

AsyncWebSocket ws("/ws");  // Cria o objeto WebSocket na rota /ws

// --- INÍCIO: Globais do WiFi Manager ---
String wifi_ssid = "";
String wifi_pass = "";
// --- FIM: Globais do WiFi Manager ---

// --- INÍCIO: Globais para Timeout do Flash Init ---
volatile bool g_flashBeginDone = false;
volatile bool g_flashBeginSuccess = false;
TaskHandle_t h_flashInitTask = NULL;
// --- FIM: Globais para Timeout do Flash Init ---

// --- Globais para Simulação de UI ---
volatile bool g_wsBtnEsq = false;
volatile bool g_wsBtnFun = false;
volatile bool g_wsBtnDir = false;

// --- INÍCIO: Globais de Calibração (ATUALIZADO) ---
volatile int g_calStreamChannel = -1;  // Canal para stream ao vivo (-1 = desligado)
volatile int g_calStreamADCType = 0;   // 0=Loop(1256), 1=Volt(1115)
volatile int g_calReadChannel = -1;    // Canal para leitura de precisão
volatile int g_calReadADCType = 0;     // 0=Loop(1256), 1=Volt(1115)
volatile int g_calReadTarget_i = -1;   // Qual ponto (0, 1, 2...)
volatile int g_calReadSamples_n = 10;  // Default 2^10 = 1024 amostras

// --- Globais para Cálculo de Regressão (ATUALIZADO) ---
volatile bool g_calCalculateRequest = false;
volatile int g_calCalc_ADCType = 0;  // 0=Loop(1256), 1=Volt(1115)
int g_calCalc_ch = 0;
int g_calCalc_size = 0;
// ATENÇÃO: Agora armazena o RAW (long) e não o (double) convertido
long g_calCalc_x_raw[257];  // Armazena os X (raw)
double g_calCalc_y[257];    // Armazena os Y (target)
// --- FIM: Globais de Calibração ---
// --- Fim Globais WebSocket ---

// --- FIM: Variáveis do WebServer e AP ---

//harware
LiquidCrystal_I2C real_lcd(e_lcd, 16, 2);  // 1. Renomeia o objeto do LCD real
VirtLCD lcd(real_lcd, ws);                 // 2. Cria nosso VirtLCD com o nome "lcd"
PCF8574 reles(e_reles);
RTC_DS1307 rtc;

ADS1115_WE ads1115A = ADS1115_WE(e_1115A);
ADS1115_WE ads1115B = ADS1115_WE(e_1115B);
ADS1115_WE ads1115C = ADS1115_WE(e_1115C);
ADS1115_WE ads1115D = ADS1115_WE(e_1115D);
ADS1256 ads1256(drdy_1256, ADS1256::PIN_UNUSED, ADS1256::PIN_UNUSED, cs_1256, 2.500, &SPI);  //DRDY, RESET, SYNC(PDWN), CS, VREF(float).//(cs_1256, drdy_1256);
U8G2_ST7920_128X64_1_HW_SPI u8g2_t1(U8G2_R2, cs_t1, U8X8_PIN_NONE);                          // spi1
U8G2_ST7920_128X64_1_HW_SPI u8g2_t2(U8G2_R2, cs_t2, U8X8_PIN_NONE);                          // spi2
U8G2_ST7920_128X64_1_HW_SPI u8g2_t3(U8G2_R2, cs_t3, U8X8_PIN_NONE);                          // spi3
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2_s1(U8G2_R0, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2_s2(U8G2_R0, U8X8_PIN_NONE);

//botoes
byte bs = 0b00000000;
int bt = 0;
bool lo = false;
int tlo = 15;

//tempo
DateTime now;
DateTime ini;
TimeSpan trun;
unsigned long t = 0;
unsigned long tbut = 0;
unsigned long tmenu = 0;
unsigned long r8 = 0;
int t_delay = 500;

// --- ADICIONE ESTAS DUAS LINHAS ---
unsigned long t_cal_stream = 0;
const long CAL_STREAM_INTERVAL = 100;  // 100ms = 10 updates por segundo
// --- FIM DA ADIÇÃO ---

//gfx
double gfx_val = 0;
int tick = 62;
CircularBuffer<long, 62> xs;
int res = 125 / tick;
int xc = 0;
int o_xc = 0;
int dgfx = 0;
uint8_t draw_state = 0;
long apre = 8388607L;  // precisao do ADC arduino = 1023 ESP32 = 4096 ADS1115 = 32767 ADS1256 = 8388607

//menu
int mstate = 0;
char mtxt[100];

// datalog
int last_s_time = 0;
bool sampling = false;
int s_count = 0;
double s_loop[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
double s_volt[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
double val_loop_420mV[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
double val_loop_conv[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
double val_volt_conv[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int ADS1256Ch[8] = { SING_0, SING_1, SING_2, SING_3, SING_4, SING_5, SING_6, SING_7 };

CircularBuffer<long, 100> v_t;  //tensao
double v_t_avg = 0;
double i_t = 0;  //corrente
double t_t = 0;  //temp
int adc = 0;
int ch = 0;
double vref = 2.5f;

//calib
int cal_n = 0;         //numero de pts (0..8)
int cal_i = 0;         //pt selecionado
int cal_size = 3;      //numero de pts 2 ^ n + 1 (3..257)
int cal_ok = 0;        //numero de pontos calibrados
double cal_x[257];     // medicoes
double cal_y[257];     // range max min
bool cal_a[257];       // medido?
double *cal_alpha;     //a+bx = y
double *cal_beta;      //a+bx = y
double cal_y_max = 0;  // loop = 20 volt5 = 5 volt12 = 12
double cal_y_min = 0;  // loop = 4 volt5 = 0 volt12 = 0
double cal_val = 0;    //media cumulativa
double cal_err = 0;    // ordem de grandeza do erro
int cal_sam = 0;       // numero de amostras para calibracao
double cal_eps = 0;    // erro da regressao linear
double cal_r = 0;      // correlacao da regressao linear
bool *opt_a;           //mudar loop_a ou volt_a

// --- Definições do Chip Flash Externo ---
SPIFlash flash(W25Q16_CS_PIN);  // Objeto do Driver SPIMemory
Preferences preferences;        // Objeto para NVS

const char *NVS_NAMESPACE = "ePAC";
const char *NVS_WRITE_PTR_KEY = "write_ptr";
uint32_t cache_write_ptr = 0;       // Nosso "ponteiro" de onde escrever
uint32_t chip_capacity = 0;         // Capacidade total do chip
const uint32_t SECTOR_SIZE = 4096;  // 4KB para chips W25Qxx

// --- Flags de Controle e Buffer (NÃO MUDAM) ---
bool useExternalFlash = false;
bool useInternalFlash = false;
const char *CACHE_FILE = "/log_cache.csv";  // Usado APENAS para o fallback interno
String ramBuffer = "";
int ramBufferCount = 0;
const int MAX_RAM_LINES = 20;

// Variáveis de tempo para o "dump"
unsigned long t_dump_sd = 0;
const long DUMP_SD_INTERVAL = 300000;  // Faz o dump para o SD a cada 5 min (300.000 ms)

//download e upload de arquivos
File uploadFile;
SemaphoreHandle_t sdMutex;
SemaphoreHandle_t i2cMutex;

// ------------------------------------

// volatile = "avise o compilador que esta variável pode mudar a qualquer momento"
volatile bool g_powerFailed = false;



//--- Funcoes ---

// --- INÍCIO: Protótipos de Funções (ATUALIZADO) ---
// Adicione estes para corrigir erros de "not declared in this scope"

// Funções do Datalog e SD
void dumpRamToFlash();
void dumpFlashToSD();

// Funções de Leitura e UI
int readADC(int fadc, int fch, bool fcc);
void refresh_menu();
void fatal_error();
void checkb();

// Funções de Configuração
bool saveConfiguration();
bool isADCEnabled(int fadc);

// Funções de Handlers do WebServer
void handleConfigPage(AsyncWebServerRequest *request);
void handleConfigSave(AsyncWebServerRequest *request);
void handleConfigLoopPage(AsyncWebServerRequest *request);
void handleConfigLoopSave(AsyncWebServerRequest *request);
void handleConfigVoltPage(AsyncWebServerRequest *request);
void handleConfigVoltSave(AsyncWebServerRequest *request);
void handleWifiPage(AsyncWebServerRequest *request);
void handleWifiSave(AsyncWebServerRequest *request);

// --- FIM: Protótipos de Funções ---

/**
 * @brief Envia uma mensagem (qualquer tipo) para o Serial Monitor E para todos os clientes WebSocket.
 */
template<typename T>
void log_print(T msg) {
  // 1. Envia para o Serial Monitor
  Serial.print(msg);

  // 2. Converte para String e envia para o WebSocket
  // O construtor String() é sobrecarregado para int, double, IPAddress, etc.
  String str_msg = String(msg);
  String json = "{\"type\":\"serial\", \"msg\":\"";
  json += str_msg;
  json += "\"}";
  ws.textAll(json);
}

/**
 * @brief Envia uma mensagem (qualquer tipo) com newline para Serial e WebSocket.
 */
template<typename T>
void log_println(T msg) {
  // 1. Envia para o Serial Monitor
  Serial.println(msg);

  // 2. Converte para String e envia para o WebSocket
  String str_msg = String(msg);
  String json = "{\"type\":\"serial\", \"msg\":\"";
  json += str_msg;
  json += "\\r\\n\"}";  // Adiciona newline
  ws.textAll(json);
}

/**
 * @brief Trata o caso de log_println() vazio (apenas newline).
 */
void log_println() {
  Serial.println();
  ws.textAll("{\"type\":\"serial\", \"msg\":\"\\r\\n\"}");
}

/**
 * @brief Escreve o buffer de RAM (Nível 1) no buffer de Flash (Nível 2).
 */
void dumpRamToFlash() {
  if (ramBufferCount == 0) return;
  log_print("Dumping ");
  log_print(ramBufferCount);
  log_println(" linhas da RAM para a Flash...");

  if (useExternalFlash) {
    // Lógica para SPIMemory (Raw "Array" Access)
    int len = ramBuffer.length();

    // 1. Verifica se temos espaço no chip
    if (cache_write_ptr + len >= chip_capacity) {
      log_println("ERRO: Cache da Flash Externa CHEIO. Aguardando dump para o SD.");
      // Não limpa o buffer de RAM. Ele tentará novamente no próximo ciclo.
      return;
    }

    // 2. Lógica de Apagar-Antes-de-Escrever (CORRIGIDA COM LOOP)
    uint32_t startSector = cache_write_ptr / SECTOR_SIZE;
    uint32_t endSector = (cache_write_ptr + len - 1) / SECTOR_SIZE;

    // Se o ponteiro for 0, ou se cruzarmos para novos setores, precisamos apagar TODOS os setores envolvidos.
    if (cache_write_ptr == 0 || startSector != endSector) {
      log_print("Cruzou limite de setor. Apagando do setor ");
      log_print(startSector == endSector ? endSector : startSector + 1);
      log_print(" ate o setor ");
      log_println(endSector);

      // Loop para apagar todos os setores necessários (previne corrupção de buffers grandes)
      uint32_t eraseStart = (cache_write_ptr == 0) ? 0 : startSector + 1;
      for (uint32_t s = eraseStart; s <= endSector; s++) {
        if (!flash.eraseSector(s * SECTOR_SIZE)) {
          log_println("ERRO: Falha ao apagar setor. Tentara novamente.");
          return;  // Aborta a escrita, tenta de novo no proximo ciclo
        }
      }
    }

    // 3. Agora podemos escrever os dados (CORREÇÃO DO STACK OVERFLOW)
    // Em vez de usar writeStr que consome muita pilha, usamos ponteiros diretos:
    uint8_t *rawData = (uint8_t *)ramBuffer.c_str();

    if (!flash.writeByteArray(cache_write_ptr, rawData, len)) {
      log_println("ERRO: Falha ao escrever na flash externa!");
      return;  // Aborta, tenta de novo
    }

    // 4. Atualiza o ponteiro na RAM e salva na NVS
    cache_write_ptr += len;
    preferences.putULong(NVS_WRITE_PTR_KEY, cache_write_ptr);

  } else if (useInternalFlash) {
    // Lógica para LittleFS (Fallback interno) - esta parte estava correta
    File cacheFile = LittleFS.open(CACHE_FILE, "a");  // [cite: 817]
    if (!cacheFile) {
      log_println("ERRO: Nao foi possivel abrir o arquivo de cache (LittleFS)!");  // [cite: 818]
      return;                                                                      // Não limpa o buffer
    } else {
      cacheFile.print(ramBuffer);
      cacheFile.close();
    }
  } else {
    return;  // Nenhum sistema de buffer
  }

  // Se a escrita foi bem-sucedida (externa ou interna), limpa o buffer de RAM
  ramBuffer = "";
  ramBufferCount = 0;
}

/**
 * @brief Move os dados do buffer de Flash (Nível 2) para o SD Card (Nível 3).
 * Usa a estratégia "Abre-Anexa-Fecha" para robustez.
 */
void dumpFlashToSD() {

  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
    // --- Início do Bloco Protegido ---

    log_println("Iniciando DUMP da Flash para o SD Card...");
    sprintf(mtxt, "/data%05d.csv", st.n_run);

    // 3. Verifica se o cache da flash interna (fallback) tem dados
    if (useInternalFlash) {
      File cacheFile = LittleFS.open(CACHE_FILE, "r");
      File file = SD.open(mtxt, FILE_APPEND);
      if (!file) {
        log_println("ERRO CRITICO: Nao foi possivel abrir o arquivo no SD Card!");
        xSemaphoreGive(sdMutex);
        return;  // Aborta o dump, os dados permanecem no cache da flash
      }
      if (cacheFile && cacheFile.size() > 0) {
        log_println("Movendo cache INTERNO para SD...");
        uint8_t buf[64];
        while (cacheFile.available()) {
          int bytesRead = cacheFile.read(buf, sizeof(buf));
          file.write(buf, bytesRead);
        }
        cacheFile.close();
        LittleFS.remove(CACHE_FILE);
      } else if (cacheFile) {
        cacheFile.close();
      }
      file.flush();
      file.close();
    }
    // 4. Verifica se o cache da flash externa tem dados
    else if (useExternalFlash) {
      if (cache_write_ptr > 0) {
        log_println("Movendo cache EXTERNO para SD...");

        // 1. Buffer aumentado para 1024 bytes (acelera MUITO o processo)
        uint8_t readBuf[1024];
        uint32_t read_ptr = 0;

        while (read_ptr < cache_write_ptr) {
          uint32_t bytesToRead = min((uint32_t)sizeof(readBuf), cache_write_ptr - read_ptr);

          // 2. Lê os dados da Flash Externa PRIMEIRO (SD Card está fechado e "quieto")
          if (!flash.readByteArray(read_ptr, readBuf, bytesToRead)) {
            log_println("ERRO: Falha na leitura da flash externa! Abortando dump.");
            return;
          }

          // 3. Abre o SD Card, escreve o bloco de 1024 bytes e fecha
          File file = SD.open(mtxt, FILE_APPEND);
          if (!file) {
            log_println("ERRO CRITICO: Nao foi possivel abrir o arquivo no SD Card!");
            xSemaphoreGive(sdMutex);
            return;
          }

          file.write(readBuf, bytesToRead);
          file.flush();
          file.close();  // Força a gravação física (FAT)
          log_println(read_ptr);
          read_ptr += bytesToRead;

          // 4. O DELAY ADEQUADO (20 milissegundos)
          // Dá tempo para o cartão SD respirar e para o RTOS não estourar o Watchdog
          vTaskDelay(20 / portTICK_PERIOD_MS);
          esp_task_wdt_reset();  // Alimenta o watchdog explicitamente
        }

        // Sucesso! Limpa o cache da flash
        cache_write_ptr = 0;
        preferences.putULong(NVS_WRITE_PTR_KEY, cache_write_ptr);
        flash.eraseSector(0);
        // --- TRUQUE DE LIMPEZA DO BARRAMENTO SPI ---
        // Garante que a Flash está desativada antes de falar com o SD
        //digitalWrite(W25Q16_CS_PIN, HIGH);
        //SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
        //SPI.transfer(0xFF);
        //SPI.endTransaction();
        // -------------------------------------------
        vTaskDelay(20 / portTICK_PERIOD_MS);
      }
    }

    log_println("DUMP da Flash para o SD Card CONCLUIDO.");

    // --- INÍCIO DA ATUALIZAÇÃO DO HW WATCHDOG ---
    esp_task_wdt_reset();  // "Alimenta o cão" (Reset no timer de 15 min)
    log_println("[HW_WATCHDOG] Alimentado (Sucesso no SD).");
    // --- FIM DA ATUALIZAÇÃO DO HW WATCHDOG ---

    // --- Fim do Bloco Protegido ---
    xSemaphoreGive(sdMutex);
  }
}

int readADC(int fadc, int fch, bool fcc) {
  switch (fadc) {
    case 0:
      {
        if (fcc) {
          ads1256.setMUX(ADS1256Ch[fch]);
        }
        return ads1256.readSingle();
      }
      break;
    case 1:
      {
        if (fcc) {
          ads1115A.setCompareChannels((fch == 0 ? ADS1115_COMP_0_1 : ADS1115_COMP_2_3));
        }
        ads1115A.startSingleMeasurement();
        while (ads1115A.isBusy()) {}
        return ads1115A.getRawResult();
      }
      break;
    case 2:
      {
        if (fcc) {
          ads1115B.setCompareChannels((fch == 0 ? ADS1115_COMP_0_1 : ADS1115_COMP_2_3));
        }
        ads1115B.startSingleMeasurement();
        while (ads1115B.isBusy()) {}
        return ads1115B.getRawResult();
      }
      break;
    case 3:
      {
        if (fcc) {
          ads1115C.setCompareChannels((fch == 0 ? ADS1115_COMP_0_1 : ADS1115_COMP_2_3));
        }
        ads1115C.startSingleMeasurement();
        while (ads1115C.isBusy()) {}
        return ads1115C.getRawResult();
      }
      break;
    case 4:
      {
        if (fcc) {
          ads1115D.setCompareChannels((fch == 0 ? ADS1115_COMP_0_1 : ADS1115_COMP_2_3));
        }
        ads1115D.startSingleMeasurement();
        while (ads1115D.isBusy()) {}
        return ads1115D.getRawResult();
      }
      break;
    default:
      {
        log_print("(");
        log_print(fadc);
        log_print(")");
        log_print("(");
        log_print(fch);
        log_print(")");
        log_print("(");
        log_print(fcc);
        log_println(")");
        lcd.setCursor(0, 0);  // Coloca o cursor do display na coluna 1 e linha 1
        lcd.print(" ERRO NO ADC ");
        delay(t_delay);
        lcd.setCursor(0, 1);
        lcd.print(" Back to dev... ");
        fatal_error();
      }
      break;
  }
}

void refresh_menu() {
  lcd.setCursor(0, 0);
  lcd.print(mbase[mstate][0]);
  lcd.setCursor(0, 1);
  lcd.print(mbase[mstate][1]);
}

// IRAM_ATTR = Coloca esta função na RAM para execução ultra-rápida
void IRAM_ATTR isr_powerFail() {
  // ISRs DEVEM ser o mais rápido possível.
  // Apenas levante a bandeira.
  g_powerFailed = true;
}

void handlePowerFailure() {
  // 1. Trava de segurança (só executa uma vez)
  static bool hasRun = false;
  if (hasRun) return;
  hasRun = true;

  // 2. DESLIGUE PERIFÉRICOS IMEDIATAMENTE!
  lcd.clear();
  lcd.print("FALHA ENERGIA!");
  lcd.setBacklight(LOW); // Economiza muita energia

  if (cfg.GFX_0) u8g2_t1.setPowerSave(1);  // Desliga GFX 0
  if (cfg.GFX_1) u8g2_s1.setPowerSave(1);  // Desliga GFX 1
  if (cfg.GFX_2) u8g2_s2.setPowerSave(1);  // Desliga GFX 2

  log_println("!!! FALHA DE ENERGIA DETECTADA !!!");

  // 3. FAÇA O DUMP DA RAM PRIMEIRO (Enquanto o SPI ainda está 100% OK)
  log_println("Salvando buffer de RAM para Flash Externa...");
  lcd.setCursor(0, 1);
  lcd.print("Salvando RAM...");
  dumpRamToFlash();  

  // 4. DESLIGA O SD CARD E ISOLA O BARRAMENTO SPI
  log_println("Desligando e isolando SD card...");
  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
    SD.end(); // Desmonta o sistema de arquivos
    
    // Garante que o Cartão SD seja ignorado fisicamente
    pinMode(sdcard_pin, OUTPUT);
    digitalWrite(sdcard_pin, HIGH); 
    
    // Envia clocks vazios para desativar qualquer transação fantasma
    SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
    SPI.transfer(0xFF);
    SPI.endTransaction();
    
    SPI.end(); // Desliga o periférico SPI completamente do ESP32
    xSemaphoreGive(sdMutex);
  }
  // --- DRENO AGRESSIVO PARA MATAR O ESTADO "ZUMBI" DO SD ---
  // Transforma os pinos que enviam sinal ao SD em drenos para o GND
  pinMode(sdcard_pin, OUTPUT);
  pinMode(18, OUTPUT); // SCK padrão do VSPI
  pinMode(23, OUTPUT); // MOSI padrão do VSPI
  
  digitalWrite(sdcard_pin, LOW);
  digitalWrite(18, LOW);
  digitalWrite(23, LOW);

  log_println("Buffer de RAM salvo. Desligando.");
  lcd.setCursor(0, 1);
  lcd.print("RAM Salva. Bye.");
  

  // Configura para acordar (resetar) quando o pino voltar para LOW (Energia ligada)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)POWER_FAIL_PIN, 0); 

  delay(t_delay);
  esp_deep_sleep_start();
}

/**
 * @brief Tarefa dedicada para executar o flash.begin() (que pode travar)
 */
void flashBeginTask(void *pvParameters) {
  // Esta tarefa *apenas* faz a chamada bloqueante
  g_flashBeginSuccess = flash.begin();

  // Avisa o setup() que terminou
  g_flashBeginDone = true;

  // A tarefa terminou seu trabalho, se auto-deleta
  vTaskDelete(NULL);
}

/**
 * @brief Callback para eventos do WebSocket (Conexão, Desconexão, Dados).
 */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
  switch (type) {
      // (Dentro de onWsEvent)
    case WS_EVT_CONNECT:
      log_println("WebSocket client conectado.");
      // Envia o estado ATUAL do LCD assim que ele conecta
      lcd.forceSendUpdate();  // <-- SUBSTITUA a linha g_lcdDirty
      break;
    case WS_EVT_DISCONNECT:
      log_printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      // Recebemos dados (ex: JSON de um botão)
      try {
        DynamicJsonDocument doc(1024);  // Aumentado para 1024 para cal_calculate
        deserializeJson(doc, (char *)data, len);

        const char *type = doc["type"];
        if (strcmp(type, "btn") == 0) {
          const char *btn = doc["btn"];
          const char *action = doc["action"];
          bool press = (strcmp(action, "press") == 0);
          log_print("btnweb: [");
          log_print(action);
          log_print("], [");
          log_print(btn);
          log_println("]");
          if (strcmp(btn, "esq") == 0 && g_wsBtnEsq != true) g_wsBtnEsq = press;
          else if (strcmp(btn, "fun") == 0 && g_wsBtnFun != true) g_wsBtnFun = press;
          else if (strcmp(btn, "dir") == 0 && g_wsBtnDir != true) g_wsBtnDir = press;
        } else if (strcmp(type, "reset") == 0) {
          log_println("Comando de reset recebido via WebSocket. Resetando...");
          if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
            SD.end();
            xSemaphoreGive(sdMutex);
          }
          delay(t_delay);
          ESP.restart();

        } else if (strcmp(type, "cal_stream_start") == 0) {
          g_calStreamChannel = doc["ch"];
          g_calReadSamples_n = doc["sam"];

          // --- INÍCIO DA CORREÇÃO ---
          // Verifica se a chave existe. Se não, assume 0 (Loop)
          if (doc.containsKey("cal_type") && strcmp(doc["cal_type"], "volt") == 0) {
            g_calStreamADCType = 1;  // Volt
          } else {
            g_calStreamADCType = 0;  // Loop (Default)
          }
          // --- FIM DA CORREÇÃO ---

          log_print("Iniciando stream. ADC: ");
          log_print(g_calStreamADCType == 0 ? "Loop" : "Volt");
          log_print(", Canal: ");
          log_print(g_calStreamChannel);
          log_print(", Amostras n: ");
          log_println(g_calReadSamples_n);

        } else if (strcmp(type, "cal_read") == 0) {
          g_calReadChannel = doc["ch"];
          g_calReadTarget_i = doc["target_i"];
          g_calReadSamples_n = doc["sam"];

          // --- INÍCIO DA CORREÇÃO ---
          if (doc.containsKey("cal_type") && strcmp(doc["cal_type"], "volt") == 0) {
            g_calReadADCType = 1;  // Volt
          } else {
            g_calReadADCType = 0;  // Loop (Default)
          }
          // --- FIM DA CORREÇÃO ---

          log_println("Recebido pedido de leitura de precisão...");

        } else if (strcmp(type, "cal_calculate") == 0) {
          log_println("Recebido pedido de cálculo de regressão...");
          g_calCalc_ch = doc["ch"];

          // --- INÍCIO DA CORREÇÃO ---
          if (doc.containsKey("cal_type") && strcmp(doc["cal_type"], "volt") == 0) {
            g_calCalc_ADCType = 1;  // Volt
          } else {
            g_calCalc_ADCType = 0;  // Loop (Default)
          }
          // --- FIM DA CORREÇÃO ---

          JsonArray x_raw = doc["x_raw"];
          JsonArray y_target = doc["y"];
          g_calCalc_size = x_raw.size();

          for (int i = 0; i < g_calCalc_size; i++) {
            g_calCalc_x_raw[i] = x_raw[i].as<long>();
            g_calCalc_y[i] = y_target[i].as<double>();
          }
          g_calCalculateRequest = true;
        }  // --- INÍCIO DO NOVO BLOCO ---
        else if (strcmp(type, "cal_set_mode") == 0) {
          log_println("Recebido comando para mudar modo de calibração...");
          int ch = doc["ch"];

          if (strcmp(doc["cal_type"], "loop") == 0) {
            bool isActive = doc["active"];
            cfg.loop_a[ch] = isActive;  // [cite: 38]
            log_print("Canal LOOP ");
            log_print(ch);
            log_print(" definido para: ");
            log_println(isActive ? "ATIVO" : "PASSIVO");

          } else if (strcmp(doc["cal_type"], "volt") == 0) {
            bool is_5v = doc["is_5v"];
            cfg.volt_a[ch] = is_5v;  // [cite: 41]
            log_print("Canal VOLT ");
            log_print(ch);
            log_print(" definido para: ");
            log_println(is_5v ? "5V" : "12V");
          }

          // Salva a configuração imediatamente
          if (saveConfiguration()) {  // [cite: 228]
            log_println("Configuração de modo salva.");
            ws.textAll("{\"type\":\"cal_mode_saved\"}");
          } else {
            log_println("ERRO: Falha ao salvar configuração de modo.");
            ws.textAll("{\"type\":\"cal_error\", \"msg\":\"Falha ao salvar modo no SD!\"}");
          }
        }  // --- ADICIONE ESTE NOVO BLOCO ---
        else if (strcmp(type, "cal_stream_stop") == 0) {
          log_println("Parando stream de calibração.");
          g_calStreamChannel = -1;  // Desliga o stream
        }

        // --- FIM DO NOVO BLOCO ---
      } catch (...) {
        log_println("Erro ao processar JSON do WebSocket");
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void fatal_error() {  // sinalizar erro fatal no led da placa
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}


// Loads the configuration from a file (adapt. de ArduinoJason)
bool loadState() {

  File file = SD.open("/state.json");

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  DynamicJsonDocument doc(256);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    return false;

  st.a_run = doc["run"];
  st.n_run = doc["n"];
  st.t_run = doc["run_time"];
  JsonObject reles = doc["reles"];

  for (int i = 0; i < 8; i++) {
    st.r[i] = reles["r" + i];
  }

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
  return true;  // (ou false se der erro)
}

// Saves the configuration to a file
// Saves the configuration to a file
bool saveState() {
  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
    if (SD.exists("/state.json")) {
      if (!SD.rename("/state.json", "/state_old.json")) {
        xSemaphoreGive(sdMutex); // Devolve o mutex se der erro
        return false;
      }
    }
    // 4. Abre o arquivo para escrita
    File file = SD.open("/state.json", FILE_WRITE);
    if (!file) {
      xSemaphoreGive(sdMutex);
      return false;
    }

    DynamicJsonDocument doc(256);
    doc["run"] = st.a_run;
    doc["n"] = st.n_run;
    doc["run_time"] = st.t_run;
    JsonObject reles = doc.createNestedObject("reles");

    for (int i = 0; i < 8; i++) {
      sprintf(mtxt, "r%01d", i);
      reles[mtxt] = st.r[i];
    }
    
    // 5. Serializa e previne vazamento de arquivo aberto
    if (serializeJson(doc, file) == 0) {
      file.close();             // É obrigatório fechar antes de abortar!
      xSemaphoreGive(sdMutex);  // É obrigatório soltar o Mutex!
      return false;
    }

    file.close();

    // 6. Limpa o backup
    if (SD.exists("/state_old.json")) {
      SD.remove("/state_old.json");
    }
    
    xSemaphoreGive(sdMutex);
    log_println("Run state salvo com sucesso!");
    return true;
  }
  return false;
}

// Loads the configuration from a file (adapt. de ArduinoJason)
bool loadConfiguration() {


  File file = SD.open("/config.json");
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.

  DynamicJsonDocument doc(4096);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    return false;

  cfg.s_time = doc["sampling"]["time"];
  cfg.s_num = doc["sampling"]["number"];

  JsonObject ADCs = doc["ADCs"];
  cfg.a_1256 = ADCs["1256"];    // true
  cfg.a_1115A = ADCs["1115A"];  // true
  cfg.a_1115B = ADCs["1115B"];  // true
  cfg.a_1115C = ADCs["1115C"];  // false
  cfg.a_1115D = ADCs["1115D"];  // false

  JsonArray GFX = doc["GFX"];
  cfg.GFX_0 = GFX[0];  // true
  cfg.GFX_1 = GFX[1];  // false
  cfg.GFX_2 = GFX[2];  // false

  JsonObject time = doc["time"];
  cfg.time_abs = time["abs"];          // false
  cfg.time_abs_raw = time["abs_raw"];  // false
  cfg.time_rel_h = time["rel_h"];      // false
  cfg.time_rel_s = time["rel_s"];      // true
  int i = 0;
  for (JsonPair loop_item : doc["loop"].as<JsonObject>()) {
    strlcpy(cfg.loop_title[i], loop_item.value()["title"], sizeof(cfg.volt_title[i]));
    cfg.loop_a[i] = loop_item.value()["active"];
    cfg.loop_log_raw[i] = loop_item.value()["log_raw"];
    cfg.loop_log_4_20[i] = loop_item.value()["log_4-20"];
    cfg.loop_log_conv[i] = loop_item.value()["log_conv"];
    cfg.loop_min[i] = loop_item.value()["min"];
    cfg.loop_max[i] = loop_item.value()["max"];
    cfg.loop_cal[i][0] = loop_item.value()["cal"][0];
    cfg.loop_cal[i][1] = loop_item.value()["cal"][1];
    i++;
  }
  i = 0;
  for (JsonPair volt_item : doc["volt"].as<JsonObject>()) {
    strlcpy(cfg.volt_title[i], volt_item.value()["title"], sizeof(cfg.volt_title[i]));
    cfg.volt_a[i] = volt_item.value()["low"];
    cfg.volt_log_raw[i] = volt_item.value()["log_raw"];
    cfg.volt_log_conv[i] = volt_item.value()["log_conv"];
    cfg.volt_cal[i][0] = volt_item.value()["cal"][0];
    cfg.volt_cal[i][1] = volt_item.value()["cal"][1];
    cfg.volt_cal[i][2] = volt_item.value()["cal"][2];
    cfg.volt_cal[i][3] = volt_item.value()["cal"][3];

    i++;
  }
  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
  return true;  // (ou false se der erro)
}

// Saves the configuration to a file
bool saveConfiguration() {

  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
    if (!SD.rename("/config.json", "/config_old.json")) {
      xSemaphoreGive(sdMutex);
      return false;
    }

    // Open file for writing
    File file = SD.open("/config.json", FILE_WRITE);
    if (!file) {
      return false;
    }

    // Allocate a temporary JsonDocument
    // Don't forget to change the capacity to match your requirements.
    // Use arduinojson.org/assistant to compute the capacity.
    DynamicJsonDocument doc(4096);

    JsonObject sampling = doc.createNestedObject("sampling");
    sampling["time"] = cfg.s_time;
    sampling["number"] = cfg.s_num;

    JsonObject ADCs = doc.createNestedObject("ADCs");
    ADCs["1256"] = cfg.a_1256;
    ADCs["1115A"] = cfg.a_1115A;
    ADCs["1115B"] = cfg.a_1115B;
    ADCs["1115C"] = cfg.a_1115C;
    ADCs["1115D"] = cfg.a_1115D;

    JsonArray GFX = doc.createNestedArray("GFX");
    GFX.add(cfg.GFX_0);
    GFX.add(cfg.GFX_1);
    GFX.add(cfg.GFX_2);

    JsonObject time = doc.createNestedObject("time");
    time["abs"] = cfg.time_abs;
    time["abs_raw"] = cfg.time_abs_raw;
    time["rel_h"] = cfg.time_rel_h;
    time["rel_s"] = cfg.time_rel_s;

    JsonObject loop = doc.createNestedObject("loop");
    for (int i = 0; i < 8; i++) {
      sprintf(mtxt, "s%01d", i);
      JsonObject loop_si = loop.createNestedObject(mtxt);
      loop_si["title"] = cfg.loop_title[i];
      loop_si["active"] = cfg.loop_a[i];
      loop_si["log_raw"] = cfg.loop_log_raw[i];
      loop_si["log_4-20"] = cfg.loop_log_4_20[i];
      loop_si["log_conv"] = cfg.loop_log_conv[i];
      loop_si["min"] = cfg.loop_min[i];
      loop_si["max"] = cfg.loop_max[i];
      JsonArray loop_si_cal = loop_si.createNestedArray("cal");
      loop_si_cal.add(cfg.loop_cal[i][0]);
      loop_si_cal.add(cfg.loop_cal[i][1]);
    }

    JsonObject volt = doc.createNestedObject("volt");
    for (int i = 0; i < 8; i++) {
      sprintf(mtxt, "x%01d", i);
      JsonObject volt_xi = volt.createNestedObject(mtxt);
      volt_xi["title"] = cfg.volt_title[i];
      volt_xi["low"] = cfg.volt_a[i];
      volt_xi["log_raw"] = cfg.volt_log_raw[i];
      volt_xi["log_conv"] = cfg.volt_log_conv[i];

      JsonArray volt_xi_cal = volt_xi.createNestedArray("cal");
      volt_xi_cal.add(cfg.volt_cal[i][0]);
      volt_xi_cal.add(cfg.volt_cal[i][1]);
      volt_xi_cal.add(cfg.volt_cal[i][2]);
      volt_xi_cal.add(cfg.volt_cal[i][3]);
    }

    // Serialize JSON to file
    if (serializeJson(doc, file) == 0) {
      return false;
    }

    // Close the file
    file.close();
    if (!SD.remove("/config_old.json")) {
      xSemaphoreGive(sdMutex);
      return false;
    }
    xSemaphoreGive(sdMutex);
    return true;
  }
  return false;
}

void navigate() {  // navegar menus
  if (!lo) {
    mstate = mnav[mstate][bs];
    log_print("click:");
    log_println(bs);
    switch (mstate) {
      case 0:
      case 1:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
      case 19:
        {
        }
        break;
      case 2:
        {
          if (bs == 4) {
            if (adc < 4) {
              adc++;
              ch = 0;
            } else {
              adc = 0;
              ch = 0;
            }
            draw_state = 0;
            for (int i = 0; i < xs.size(); i++) xs.push(0);
            for (int i = 0; i < v_t.size(); i++) v_t.push(0);
            switch (adc) {
              case 0:
                {
                  apre = 8388607L;
                  vref = 2.5f;  //2.44f
                }
                break;
              case 1:
                {
                  apre = 32767L;
                  vref = 2.048f;
                }
                break;
              case 2:
                {
                  apre = 32767L;
                  vref = 2.048f;
                }
                break;
              case 3:
                {
                  apre = 32767L;
                  vref = 2.048f;
                }
                break;
              case 4:
                {
                  apre = 32767L;
                  vref = 2.048f;
                }
                break;
              default:
                {
                  apre = 32767L;
                  vref = 2.048f;
                }
                break;
            }
          }
          if (bs == 1) {
            switch (adc) {
              case 0:
                {
                  if (ch < 7) {
                    ch++;
                  } else {
                    ch = 0;
                  }
                }
                break;
              case 1:
                {
                  if (ch < 1) {
                    ch++;
                  } else {
                    ch = 0;
                  }
                }
                break;
              case 2:
                {
                  if (ch < 1) {
                    ch++;
                  } else {
                    ch = 0;
                  }
                }
                break;
              case 3:
                {
                  if (ch < 1) {
                    ch++;
                  } else {
                    ch = 0;
                  }
                }
                break;
              case 4:
                {
                  if (ch < 1) {
                    ch++;
                  } else {
                    ch = 0;
                  }
                }
                break;
              default:
                {
                  log_print("Erro: adc desconhecido");
                }
                break;
            }
          }
        }
        break;
      case 8:
      case 9:
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
        {
          adc = 0;
          ch = mstate - 8;
          cal_alpha = &cfg.loop_cal[ch][0];
          cal_beta = &cfg.loop_cal[ch][1];
          cal_y_max = 20;
          cal_y_min = 4;
        }
        break;
      case 20:
      case 21:
      case 22:
      case 23:
      case 24:
      case 25:
      case 26:
      case 27:
        {
          ch = (mstate - 20) % 2;
          adc = int((mstate - 20) / 2.0) + 1;
          cal_alpha = &cfg.volt_cal[2 * (adc - 1) + ch][0 + 2 * (cfg.volt_a[2 * (adc - 1) + ch] ? 1 : 0)];
          cal_beta = &cfg.volt_cal[2 * (adc - 1) + ch][1 + 2 * (cfg.volt_a[2 * (adc - 1) + ch] ? 1 : 0)];
          cal_y_max = 12 - (7 * (cfg.volt_a[2 * (adc - 1) + ch] ? 1 : 0));
          cal_y_min = 0;
        }
        break;
      case 16:
        {
          if (bs == 1) {
            if (cal_n < 8) {
              cal_n++;
            } else {
              cal_n = 0;
            }
          }
          if (bs == 4) {
            if (cal_n > 0) {
              cal_n--;
            } else {
              cal_n = 8;
            }
          }

          cal_i = 0;
          cal_size = pow(2, cal_n) + 1;
          double cal_dy = (cal_y_max - cal_y_min) / (cal_size - 1);
          for (int i = 0; i < 257; i++) {
            cal_x[i] = 0;
            cal_y[i] = 0;
            cal_a[i] = true;
          }
          for (int i = 0; i < cal_size; i++) {
            cal_y[i] = cal_y_min + cal_dy * i;
            cal_a[i] = false;
          }
        }
        break;
      case 17:
        {
          if (bs == 1) {
            if (cal_i < cal_size - 1) {
              cal_i++;
            } else {
              cal_i = 0;
            }
          }
          if (bs == 4) {
            if (cal_i > 0) {
              cal_i--;
            } else {
              cal_i = cal_size - 1;
            }
          }

          cal_val = 0;
          cal_err = 0;
        }
        break;
      case 18:
        {
          if (bs == 1) {
            if (cal_sam < 20) {
              cal_sam++;
            } else {
              cal_sam = 0;
            }
          }
          if (bs == 4) {

            double cal_S1 = 0;
            double cal_S2 = 0;
            double cal_np = pow(2, cal_sam);
            lcd.setBacklight(LOW);
            double cal_m = readADC(adc, ch, true) * (adc == 0 ? 2.98023e-7 : 6.25e-5);
            for (int i = 0; i < cal_np; i++) {
              cal_S1 += cal_m;
              cal_S2 += cal_m * cal_m;
              cal_m = readADC(adc, ch, false) * (adc == 0 ? 2.98023e-7 : 6.25e-5);
              if (i % 10 == 0) {
                lcd.setBacklight(HIGH);
              } else {
                lcd.setBacklight(LOW);
              }
            }
            lcd.setBacklight(HIGH);
            cal_val = cal_S1 / cal_np;
            cal_err = sqrt(cal_S2 / cal_np - (cal_S1 / cal_np) * (cal_S1 / cal_np));
            log_print("{v:");
            log_print(cal_val);
            log_println("}");
            log_print("{e10:");
            log_print(log10(cal_err));
            log_println("}");
            /*
              cal_val = 0;//k=0
              cal_err = 0;//k=0
              lcd.setBacklight(LOW);
              double cal_m = readADC(adc, ch, true) * (adc == 0 ? 2.98023e-7 : 6.25e-5);
              cal_val += cal_m; //k=1
              cal_err = 0; //k=1
              for (int i = 1; i < pow(2, cal_sam); i++) { //k = i+1
              cal_m = readADC(adc,    , false) * (adc == 0 ? 2.98023e-7 : 6.25e-5);
              cal_err = cal_err + ((i / (i + 1)) * (cal_m - cal_val) * (cal_m - cal_val));
              cal_val = cal_val + ((float(cal_m) - cal_val) / (i + 1));
              //https://en.wikipedia.org/wiki/Standard_deviation
              if (i % 10 == 0) {
                lcd.setBacklight(HIGH);
              } else {
                lcd.setBacklight(LOW);
              }
              }
              lcd.setBacklight(HIGH);
              //cal_err = log10(cal_err / (pow(2, cal_sam) - 1));*/
          }
          cal_x[cal_i] = cal_val;
          cal_a[cal_i] = true;
          cal_ok = cal_size;
          for (int i = 0; i < cal_size; i++) {
            if (cal_a[i] == false) {
              cal_ok--;
            }
          }
        }
        break;
      case 28:
        {
          saveConfiguration();
        }
        break;
      case 29:
      case 30:
      case 31:
      case 32:
      case 33:
      case 34:
        {
          opt_a = &cfg.loop_a[mstate - 29];  //seta opt
        }
        break;
      case 35:
      case 36:
      case 37:
      case 38:
      case 39:
      case 40:
      case 41:
      case 42:
        {
          opt_a = &cfg.volt_a[mstate - 35];  //seta opt
        }
        break;
      case 43:
      case 44:
        {
          if (bs == 1 or bs == 4) {
            *opt_a = !*opt_a;  //muda opt
          }
        }
        break;
      default:
        {
          log_print("ec:");
          log_println(mstate);
          lcd.setCursor(0, 0);  // Coloca o cursor do display na coluna 1 e linha 1
          lcd.print("  ERRO NO NAVC  ");
          delay(t_delay);
          lcd.setCursor(0, 1);
          lcd.print(" Back to dev... ");
          fatal_error();
        }
        break;
    }
  } else {
    switch (mstate) {
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
      case 18:
      case 19:
      case 28:
      case 29:
      case 30:
      case 31:
      case 32:
      case 33:
      case 34:
      case 35:
      case 36:
      case 37:
      case 38:
      case 39:
      case 40:
      case 41:
      case 42:
      case 43:
      case 44:
        {
        }
        break;
      case 8:
      case 9:
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
        {
          adc = 0;
          ch = mstate - 8;
          cal_alpha = &cfg.loop_cal[ch][0];
          cal_beta = &cfg.loop_cal[ch][1];
          cal_eps = 0;
          cal_r = 0;
        }
        break;
      case 16:
        {
        }
        break;
      case 17:
        {
          if (cal_ok == cal_size) {
            double Sx = 0;
            double Sxx = 0;
            double Sxy = 0;
            double Sy = 0;
            double Syy = 0;
            lcd.setBacklight(LOW);
            for (int i = 0; i < cal_size; i++) {
              Sx += cal_x[i];
              Sxx += cal_x[i] * cal_x[i];
              Sxy += cal_x[i] * cal_y[i];
              Sy += cal_y[i];
              Syy += cal_y[i] * cal_y[i];
              if (i % 2 == 0) {
                lcd.setBacklight(HIGH);
              } else {
                lcd.setBacklight(LOW);
              }
            }
            lcd.setBacklight(HIGH);
            delay(50);
            lcd.setBacklight(LOW);
            double t_beta = (cal_size * Sxy - Sx * Sy) / (cal_size * Sxx - Sx * Sx);
            double t_alpha = Sy / cal_size - t_beta * Sx / cal_size;
            //log_print("{b"); log_print(t_beta); log_print("}");
            //log_print("{a"); log_print(t_alpha); log_println("}");
            *cal_beta = t_beta;
            *cal_alpha = t_alpha;
            if (cal_size > 2) {
              cal_eps = (1 / (cal_size * (cal_size - 2))) * (cal_size * Syy - Sy * Sy - t_beta * t_beta * (cal_size * Sxx - Sx * Sx));
            } else {
              cal_eps = 0;
            }
            cal_r = (cal_size * Sxy - Sx * Sy) / sqrt((cal_size * Sxx - Sx * Sx) * (cal_size * Syy - Sy * Sy));
            lcd.setBacklight(HIGH);
            delay(50);
            lcd.setBacklight(LOW);
            saveConfiguration();
            lcd.setBacklight(HIGH);
          } else {
            lcd.setBacklight(LOW);
            delay(50);
            lcd.setBacklight(HIGH);
            delay(50);
            lcd.setBacklight(LOW);
            delay(50);
            lcd.setBacklight(HIGH);
            delay(50);
            lcd.setBacklight(LOW);
            delay(50);
            lcd.setBacklight(HIGH);
            delay(50);
            bs = 0;
          }
        }
        break;
      case 20:
      case 21:
      case 22:
      case 23:
      case 24:
      case 25:
      case 26:
      case 27:
        {
          ch = mstate - 20;
          adc = int(ch / 2.0f) + 1;
          cal_alpha = &cfg.volt_cal[ch][0 + 2 * (cfg.volt_a[ch] ? 1 : 0)];
          cal_beta = &cfg.volt_cal[ch][1 + 2 * (cfg.volt_a[ch] ? 1 : 0)];
          cal_eps = 0;
          cal_r = 0;
        }
        break;
      default:
        {
          log_print("eh:");
          log_println(mstate);
          lcd.setCursor(0, 0);  // Coloca o cursor do display na coluna 1 e linha 1
          lcd.print("  ERRO NO NAVH  ");
          delay(t_delay);
          lcd.setCursor(0, 1);
          lcd.print(" Back to dev... ");
          fatal_error();
        }
        break;
    }
    mstate = mnav[mstate][bs + 8];
    log_print("hold:");
    log_println(bs);
  }
  refresh_menu();
}

void draw_t1(void) {
  u8g2_t1.setFont(u8g2_font_6x10_tf);
  u8g2_t1.setFontRefHeightExtendedText();
  u8g2_t1.setDrawColor(1);
  u8g2_t1.setFontPosTop();
  u8g2_t1.setFontDirection(0);
  u8g2_t1.drawFrame(0, 0, u8g2_t1.getDisplayWidth(), u8g2_t1.getDisplayHeight());
  if (adc == 0) {
    u8g2_t1.drawStr(2, 1, cfg.loop_title[ch]);
  } else {
    u8g2_t1.drawStr(2, 1, cfg.volt_title[2 * (adc - 1) + ch]);
  }
  apre = (adc == 0 ? 8388607L : 32767L);
  int maxv = map(4000, 0, apre - 1, 10, 60);
  u8g2_t1.drawLine(2, 10, 2, 61);
  u8g2_t1.drawLine(2, 61, 125, 61);
  u8g2_t1.drawLine(2, 70 - maxv, 125, 70 - maxv);
  for (int i = 1; i < tick; ++i) {
    int x0 = map(xs[i], 0, apre - 1, 10, 60);
    int x1 = map(xs[i - 1], 0, apre - 1, 10, 60);
    u8g2_t1.drawLine((1 + i) * res, (70 - x0), (i)*res, (70 - x1));
  }
}
void draw_s1(void) {
  u8g2_s1.setFont(u8g2_font_6x10_tf);
  u8g2_s1.setFontRefHeightExtendedText();
  u8g2_s1.setDrawColor(1);
  u8g2_s1.setFontPosTop();
  u8g2_s1.setFontDirection(0);
  u8g2_s1.drawStr(2, 1, cfg.volt_title[0]);
  u8g2_s1.drawStr(2, u8g2_s1.getDisplayHeight() / 2 + 2, cfg.volt_title[1]);
  u8g2_s1.setFont(u8g2_font_10x20_tf);
  if (cfg.volt_log_conv[0]) {
    sprintf(mtxt, "%+-.7f", val_volt_conv[0]);
    u8g2_s1.drawStr(2, 11, mtxt);
  } else {
    u8g2_s1.drawStr(2, 11, "0.00");
  }
  if (cfg.volt_log_conv[1]) {
    sprintf(mtxt, "%+-.7f", val_volt_conv[1]);
    u8g2_s1.drawStr(2, u8g2_s1.getDisplayHeight() / 2 + 13, mtxt);
  } else {
    u8g2_s1.drawStr(2, u8g2_s1.getDisplayHeight() / 2 + 13, "0.00");
  }
  u8g2_s1.drawFrame(0, 0, u8g2_s1.getDisplayWidth(), u8g2_s1.getDisplayHeight() / 2 - 1);
  u8g2_s1.drawFrame(0, u8g2_s1.getDisplayHeight() / 2 + 1, u8g2_s1.getDisplayWidth(), u8g2_s1.getDisplayHeight() - (u8g2_s2.getDisplayHeight() / 2 + 1));
}
void draw_s2(void) {

  u8g2_s2.setFont(u8g2_font_6x10_tf);
  u8g2_s2.setFontRefHeightExtendedText();
  u8g2_s2.setDrawColor(1);
  u8g2_s2.setFontPosTop();
  u8g2_s2.setFontDirection(0);
  u8g2_s2.drawStr(2, 1, cfg.volt_title[2]);
  u8g2_s2.drawStr(2, u8g2_s2.getDisplayHeight() / 2 + 2, cfg.volt_title[3]);
  u8g2_s2.setFont(u8g2_font_10x20_tf);
  if (cfg.volt_log_conv[2]) {
    sprintf(mtxt, "%+-.7f", val_volt_conv[2]);
    u8g2_s2.drawStr(2, 11, mtxt);
  } else {
    u8g2_s2.drawStr(2, 11, "0.00");
  }
  if (cfg.volt_log_conv[3]) {
    sprintf(mtxt, "%+-.7f", val_volt_conv[3]);
    u8g2_s2.drawStr(2, u8g2_s2.getDisplayHeight() / 2 + 13, mtxt);
  } else {
    u8g2_s2.drawStr(2, u8g2_s2.getDisplayHeight() / 2 + 13, "0.00");
  }
  u8g2_s2.drawFrame(0, 0, u8g2_s2.getDisplayWidth(), u8g2_s2.getDisplayHeight() / 2 - 1);
  u8g2_s2.drawFrame(0, u8g2_s2.getDisplayHeight() / 2 + 1, u8g2_s2.getDisplayWidth(), u8g2_s2.getDisplayHeight() - (u8g2_s2.getDisplayHeight() / 2 + 1));
}



void checkb() {  // controle de botoes bits 4 - esquerda / 2 - func / 1 - direita; bt = tempo de aperto (max 10 seg = 100)
  byte a = 0b00000000;
  byte b = 0b00000000;
  byte c = 0b00000000;

  if (g_wsBtnEsq) {  //if (digitalRead(besq_pin) == LOW || g_wsBtnEsq) {
    a = 0b00000100;
    g_wsBtnEsq = false;
  } else {
    a = 0b00000000;
  }
  if (g_wsBtnFun) {  //if (digitalRead(bfun_pin) == LOW || g_wsBtnFun) {
    b = 0b00000010;
    g_wsBtnFun = false;
  } else {
    b = 0b00000000;
  }
  if (g_wsBtnDir) {  //if (digitalRead(bdir_pin) == LOW || g_wsBtnDir) {
    c = 0b00000001;
    g_wsBtnDir = false;
  } else {
    c = 0b00000000;
  }
  byte bm = a | b | c;
  //log_print("btfis: [");
  //log_print(uint32_t(bm));
  //log_println("]");
  if (bs == bm) {
    if (bt < 100) {
      bt++;
      if (bs != 0 && bt > tlo) {
        lo = true;
      }
    }
  } else {
    if (bs != 0 && bm == 0) {
      navigate();
      lo = false;
    }
    bt = 0;
  }
  if (bm == 0) {
    bs = bm;
  } else {
    bs = max(bs, bm);
  }
}

void core_menu() {
  switch (mstate) {
    case 0:
    case 1:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
    case 31:
    case 32:
    case 33:
    case 34:
    case 35:
    case 36:
    case 37:
    case 38:
    case 39:
    case 40:
    case 41:
    case 42:
    case 43:
    case 44:
      {
      }
      break;
    case 2:
      {
        if (!st.a_run) {
          if (bs == 2) {
            log_println("Run iniciado!");
            st.a_run = true;
            ini = now;
            st.t_run = ini.unixtime();
            last_s_time = 0;
            saveState();

            String line = "";

            if (cfg.time_abs) {
              line += "time_abs";
              line += ",";
            }
            if (cfg.time_abs_raw) {
              line += "time_abs_raw";
              line += ",";
            }
            if (cfg.time_rel_h) {
              line += "time_rel_h";
              line += ",";
            }
            if (cfg.time_rel_s) {
              line += "time_rel_s";
              line += ",";
            }
            for (int i = 0; i < 8; i++) {
              if (cfg.loop_log_raw[i]) {
                sprintf(mtxt, "%s_raw", cfg.loop_title[i]);
                line += mtxt;
                line += ",";
              }
              if (cfg.loop_log_4_20[i]) {
                sprintf(mtxt, "%s_mA", cfg.loop_title[i]);
                line += mtxt;
                line += ",";
              }
              if (cfg.loop_log_conv[i]) {
                sprintf(mtxt, "%s", cfg.loop_title[i]);
                line += mtxt;
                line += ",";
              }
            }
            for (int i = 0; i < 8; i++) {
              if (cfg.volt_log_raw[i]) {
                sprintf(mtxt, "%s_raw", cfg.volt_title[i]);
                line += mtxt;
                line += ",";
              }
              if (cfg.volt_log_conv[i]) {
                sprintf(mtxt, "%s", cfg.volt_title[i]);
                line += mtxt;
                line += ",";
              }
            }
            line += "n_reboot";
            //line += "\n";
            log_println(line);
            sprintf(mtxt, "/data%05d.csv", st.n_run);
            log_print("Arquivo:");
            log_println(mtxt);
            if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
              File file = SD.open(mtxt, FILE_WRITE);
              file.println(line);
              file.flush();
              file.close();
              xSemaphoreGive(sdMutex);  // Devolve o mutex após as checagens
            }
            log_println("Arquivo da run criado!");
          }
        } else {
          trun = now - ini;
          if (bs == 7) {
            st.a_run = false;
            st.t_run = 0;
            last_s_time = 0;
            trun = now - now;
            s_count = 0;
            sampling = false;
          }
        }
      }
      break;
    default:
      {
        log_println(mstate);
        lcd.setCursor(0, 0);  // Coloca o cursor do display na coluna 1 e linha 1
        lcd.print("  ERRO NO CORE  ");
        delay(t_delay);
        lcd.setCursor(0, 1);
        lcd.print(" Back to dev... ");
        fatal_error();
      }
      break;
  }
}

void datalog() {
  if (st.a_run) {
    if (int((trun.totalseconds() - last_s_time) / cfg.s_time) > 0) {
      sampling = true;
      last_s_time = trun.totalseconds();
    }

    if (sampling) {
      if (s_count < cfg.s_num) {
        //log_print(s_count);
        //log_print(",");
        if (cfg.a_1256) {
          for (int i = 0; i < 8; i++) {
            s_loop[i] = (s_count * s_loop[i] + float(readADC(0, i, true))) / (s_count + 1);
          }
        }
        bool a_1115X[4] = { cfg.a_1115A, cfg.a_1115B, cfg.a_1115C, cfg.a_1115D };
        for (int i = 1; i < 5; i++) {
          if (a_1115X[i - 1]) {
            for (int j = 0; j < 2; j++) {
              s_volt[(i - 1) * 2 + j] = (s_count * s_volt[(i - 1) * 2 + j] + float(readADC(i, j, true))) / (s_count + 1);
            }
          }
        }
        s_count++;
      } else {
        s_count = 0;
        gfx_val = adc == 0 ? s_loop[ch] : s_volt[2 * (adc - 1) + ch];
        v_t.unshift(gfx_val);
        xs.push(v_t[0]);
        // --- NOVA LÓGICA DE BUFFER ---
        // 1. Formata a linha de CSV em uma String (no buffer de RAM)
        // Usamos uma String local 'line' para montar a linha

        String line = "";

        if (cfg.time_abs) {
          sprintf(mtxt, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
          line += mtxt;
          line += ",";
        }
        if (cfg.time_abs_raw) {
          sprintf(mtxt, "%d", now.unixtime());
          line += mtxt;
          line += ",";
        }
        if (cfg.time_rel_h) {
          sprintf(mtxt, "%f", trun.totalseconds() / 3600.0);
          line += mtxt;
          line += ",";
        }
        if (cfg.time_rel_s) {
          sprintf(mtxt, "%d", trun.totalseconds());
          line += mtxt;
          line += ",";
        }
        for (int i = 0; i < 8; i++) {
          if (cfg.loop_log_raw[i]) {
            sprintf(mtxt, "%07d", int(s_loop[i]));
            line += mtxt;
            line += ",";
          }
          val_loop_420mV[i] = cfg.loop_cal[i][0] + (s_loop[i] * 2.5f / 8388607.0f) * cfg.loop_cal[i][1];
          if (cfg.loop_log_4_20[i]) {
            sprintf(mtxt, "%+-.7f", val_loop_420mV[i]);
            line += mtxt;
            line += ",";
          }
          val_loop_conv[i] = (cfg.loop_max[i] - cfg.loop_min[i]) / (16.0f) * (val_loop_420mV[i] - 4.0f) + cfg.loop_min[i];
          if (cfg.loop_log_conv[i]) {
            sprintf(mtxt, "%+-.7f", val_loop_conv[i]);
            line += mtxt;
            line += ",";
          }
        }
        for (int i = 0; i < 8; i++) {
          if (cfg.volt_log_raw[i]) {
            sprintf(mtxt, "%07d", int(s_volt[i]));
            line += mtxt;
            line += ",";
          }
          val_volt_conv[i] = cfg.volt_cal[i][2 * (cfg.volt_a[i] ? 1 : 0)] + cfg.volt_cal[i][2 * (cfg.volt_a[i] ? 1 : 0) + 1] * (s_volt[i] * 2.048f / 32767.0f);  // voltage divider recalc
          //log_print("{v["); log_print(i); log_print("]:"); log_print(val_volt_conv[i]); log_println("}");
          if (cfg.volt_log_conv[i]) {
            sprintf(mtxt, "%+-.7f", val_volt_conv[i]);
            line += mtxt;
            line += ",";
          }
        }
        sprintf(mtxt, "%d", n_reboot);
        line += mtxt;
        line += "\n";  // Adiciona o Newline

        // 2. Adiciona a linha ao Buffer de RAM (Nível 1)
        ramBuffer += line;
        ramBufferCount++;

        // 3. Verifica se o buffer de RAM está cheio
        if (ramBufferCount >= MAX_RAM_LINES) {
          dumpRamToFlash();  // Escreve no Nível 2 (Flash)
        }

        sampling = false;
      }
    }
  } else {
    for (int i = 0; i < 8; i++) {
      s_loop[i] = 0;
      s_volt[i] = 0;
      val_loop_420mV[i] = 0;
      val_loop_conv[i] = 0;
      val_volt_conv[i] = 0;
    }
  }

  switch (mstate) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
    case 31:
    case 32:
    case 33:
    case 34:
    case 35:
    case 36:
    case 37:
    case 38:
    case 39:
    case 40:
    case 41:
    case 42:
    case 43:
    case 44:
      {
      }
      break;
    default:
      {
        log_print("(f)(");
        log_print(mstate);
        log_println(")");
        lcd.setCursor(0, 0);  // Coloca o cursor do display na coluna 1 e linha 1
        lcd.print(" ERRO NO DATALOG ");
        delay(t_delay);
        lcd.setCursor(0, 1);
        lcd.print(" Back to dev... ");
        fatal_error();
      }
      break;
  }
}

void draw_menu() {
  switch (mstate) {
    case 0:
    case 1:
    case 3:
    case 4:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 20:
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
    case 29:
    case 30:
    case 31:
    case 32:
    case 33:
    case 34:
    case 35:
    case 36:
    case 37:
    case 38:
    case 39:
    case 40:
    case 41:
    case 42:
      {
      }
      break;
    case 2:
      {
        if (!st.a_run) {
          lcd.setCursor(0, 0);
          sprintf(mtxt, "N%05d", st.n_run);
          lcd.print(mtxt);
        } else {
          lcd.setCursor(0, 0);
          if (sampling) {
            sprintf(mtxt, "S%05d", st.n_run);
          } else {
            sprintf(mtxt, "R%05d", st.n_run);
          }
          lcd.print(mtxt);
          lcd.setCursor(7, 0);
          sprintf(mtxt, "%03d:%02d:%02d", trun.hours() + 24 * trun.days(), trun.minutes(), trun.seconds());
          lcd.print(mtxt);
          lcd.setCursor(0, 1);
          if (adc == 0) {
            sprintf(mtxt, "A%dC%d%+012.7f", adc, ch, val_loop_conv[ch]);
          } else {
            sprintf(mtxt, "A%dC%d%+012.7f", adc, ch, val_volt_conv[(adc - 1) * 2 + ch]);
          }
          lcd.print(mtxt);
        }
      }
      break;
    case 5:
    case 6:
    case 28:
      {
        if (st.a_run) {
          mstate = 7;
          refresh_menu();
        }
      }
      break;
    case 16:
      {
        if (!st.a_run) {
          lcd.setCursor(0, 1);
          sprintf(mtxt, "< n=%d 2^n+1=%3d>", cal_n, cal_size);
          lcd.print(mtxt);
        } else {
          mstate = 7;
          refresh_menu();
        }
      }
      break;
    case 17:
      {
        if (!st.a_run) {
          lcd.setCursor(0, 0);
          sprintf(mtxt, "P(%03d) = %06.3f%c>", cal_i, cal_y[cal_i], (cal_a[cal_i] == true ? '!' : '#'));
          lcd.print(mtxt);
          lcd.setCursor(0, 1);
          sprintf(mtxt, "< %03d [OK] %03d >", cal_size, cal_ok);
          lcd.print(mtxt);
        } else {
          mstate = 7;
          refresh_menu();
        }
      }
      break;
    case 18:
      {
        if (!st.a_run) {
          lcd.setCursor(0, 0);
          sprintf(mtxt, "%+010.7f%+04.0e>", cal_val, cal_err);
          lcd.print(mtxt);
          lcd.setCursor(0, 1);
          sprintf(mtxt, "<RUN> <OK> <A%02d>", cal_sam);
          lcd.print(mtxt);
        } else {
          mstate = 7;
          refresh_menu();
        }
      }
      break;
    case 19:
      {
        lcd.setCursor(0, 0);
        lcd.write(0b11100000);
        sprintf(mtxt, "%+6.4f", *cal_alpha);
        lcd.print(mtxt);
        lcd.write(0b11100010);
        sprintf(mtxt, "%+6.4f", *cal_beta);
        lcd.print(mtxt);
        lcd.setCursor(0, 1);
        lcd.write(0b11100011);
        sprintf(mtxt, "%+010.3e", cal_eps);
        lcd.print(mtxt);
        lcd.print("r");
        sprintf(mtxt, "%04.2f", cal_r);
        lcd.print(mtxt);
      }
      break;
    case 43:
    case 44:
      {
        if (*opt_a) {
          lcd.setCursor(1, 1);
          lcd.write(0b01111110);
          lcd.setCursor(9, 1);
          lcd.print(" ");
        } else {
          lcd.setCursor(9, 1);
          lcd.write(0b01111110);
          lcd.setCursor(1, 1);
          lcd.print(" ");
        }
      }
      break;
    default:
      {
        lcd.setCursor(0, 0);  // Coloca o cursor do display na coluna 1 e linha 1
        lcd.print("  ERRO NO MENU  ");
        lcd.setCursor(0, 1);
        lcd.print(" Back to dev... ");
        fatal_error();
      }
      break;
  }
}


/**
 * @brief Tenta inicializar o chip flash SPI externo (SPIMemory) e carrega o ponteiro de cache da NVS.
 * @return true se o chip foi encontrado e NVS lida, false caso contrário.
 */
bool initExternalFlash() {
  log_println("Tentando inicializar Flash SPI Externa (SPIMemory)...");

  // 1. Lança a tarefa de inicialização (que pode travar) no Core 0
  g_flashBeginDone = false;
  xTaskCreatePinnedToCore(
    flashBeginTask,    // Função
    "flashInit",       // Nome
    4096,              // Stack (tamanho seguro)
    NULL,              // Parâmetros
    1,                 // Prioridade
    &h_flashInitTask,  // Handle
    0                  // Core 0 (PRO_CPU)
  );

  // 2. Aguarda o timeout de 5 segundos
  unsigned long start = millis();
  while (!g_flashBeginDone && (millis() - start < 2000)) {
    vTaskDelay(50);  // Dá tempo para a outra tarefa rodar
  }

  // 3. Analisa o resultado
  if (!g_flashBeginDone) {
    // ----- TIMEOUT! -----
    log_println("ERRO FATAL: Timeout! flash.begin() travou.");

    log_println("Matando tarefa travada...");
    if (h_flashInitTask != NULL) {
      vTaskDelete(h_flashInitTask);
      h_flashInitTask = NULL;
    }

    log_println("Forçando reset do barramento SPI...");
    SPI.end();  // Desliga o periférico SPI que a tarefa zumbi travou

    // --- INÍCIO DA CORREÇÃO CRÍTICA ---
    log_println("Reinicializando o SD Card (que usa o mesmo barramento)...");
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
      // Precisamos chamar SD.begin() de novo, porque SPI.end() matou o barramento
      if (!SD.begin(sdcard_pin)) {
        log_println("ERRO CRÍTICO: Falha ao reinicializar o SD Card apos timeout!");
        // O SD Card é essencial, então se falhar aqui, temos que parar.
        fatal_error();
      } else {
        log_println("SD Card reinicializado com sucesso.");
      }
      xSemaphoreGive(sdMutex);  // Devolve o mutex após as checagens
    }
    // --- FIM DA CORREÇÃO CRÍTICA ---
  }

  // A tarefa terminou a tempo. Ela teve sucesso?
  if (!g_flashBeginSuccess) {
    log_println("Erro: Falha ao inicializar o driver do chip flash.");
    return false;
  }

  // 2. Verifica se é um chip Winbond (0xEF)
  uint32_t jedecID = flash.getJEDECID();
  if (((jedecID >> 16) & 0xFF) != 0xEF) {
    log_print("Erro: Fabricante do chip flash desconhecido: 0x");
    sprintf(mtxt, "0x%X", ((jedecID >> 16) & 0xFF));
    log_println(mtxt);
    return false;
  }

  chip_capacity = flash.getCapacity();
  log_print("Chip W25Qxx encontrado! Capacidade: ");
  log_print(chip_capacity / 1024);
  log_println(" KB");

  // 4. Carrega o ponteiro de onde paramos
  cache_write_ptr = preferences.getULong(NVS_WRITE_PTR_KEY, 0);
  log_print("Ponteiro de cache carregado da NVS: ");
  log_println(cache_write_ptr);

  // Verificação de sanidade (não pode ser maior que o chip)
  if (cache_write_ptr >= chip_capacity) {
    log_println("Ponteiro de cache invalido! Resetando para 0.");
    cache_write_ptr = 0;
    preferences.putULong(NVS_WRITE_PTR_KEY, cache_write_ptr);
    // Apaga o primeiro setor para garantir
    flash.eraseSector(0);
  }

  return true;
}


/**
 * @brief Tarefa que roda no Core 0, responsável por gerenciar o DNS (Captive Portal).
 */
void WebServerTask(void *pvParameters) {
  log_println("Task WebServer iniciada no Core 0.");
  while (1) {
    // O DNSServer precisa ser "processado" em um loop.
    dnsServer.processNextRequest();

    // Pequeno delay para não sobrecarregar o Core 0
    vTaskDelay(10);
  }
}

/**
 * @brief Constrói e envia a página de status do hardware.
 */
void handleStatusPage(AsyncWebServerRequest *request) {
  String html = "<html><head><title>Status do Sistema - ePAC</title>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial,sans-serif;margin:0;padding:0;}";

  // --- ADICIONA O CSS DA NAVBAR ---
  html += ".navbar{background-color:#333;overflow:hidden;}";
  html += ".navbar a{float:left;display:block;color:white;text-align:center;padding:14px 16px;text-decoration:none;}";
  html += ".navbar a:hover{background-color:#ddd;color:black;}";
  // --- FIM DO CSS DA NAVBAR ---

  html += ".content{padding:20px;}";
  html += "table{border-collapse:collapse;}";
  html += "td,th{border:1px solid #ddd;padding:8px;} th{background-color:#f2f2f2;}";
  html += ".ok{color:green;font-weight:bold;} .fail{color:red;font-weight:bold;}</style>";
  html += "</head><body>";

  // --- ADICIONA O HTML DA NAVBAR ---
  html += "<div class='navbar'>";
  html += "<a href='/home'>Home</a>";
  html += "<a href='/status'>Status</a>";
  html += "<a href='/files'>Arquivos</a>";
  html += "<a href='/config'>Configurações</a>";
  html += "<a href='/wifi'>WiFi</a>";
  html += "<a href='/'>Sair</a>";
  html += "</div>";
  // --- FIM DO HTML DA NAVBAR ---

  // Conteúdo da página de status
  html += "<div class='content'>";
  html += "<h1>Status dos Periféricos</h1>";
  html += "<table><tr><th>Periférico</th><th>Status</th></tr>";

  // Função auxiliar (não muda)
  auto addRow = [&](const char *name, bool status) {
    html += "<tr><td>";
    html += name;
    html += "</td>";
    if (status) {
      html += "<td class='ok'>OK</td></tr>";
    } else {
      html += "<td class='fail'>FALHA</td></tr>";
    }
  };

  // Preenche a tabela (não muda) [cite: 885-889]
  addRow("LCD 16x2", sysStatus.lcd);
  addRow("Rede Wi-Fi (AP)", true);
  addRow("SD Card", sysStatus.sd_card);
  addRow("Arquivo 'state.json'", sysStatus.state_file);
  addRow("Arquivo 'config.json'", sysStatus.config_file);
  addRow("Reles (PCF8574)", sysStatus.reles);
  addRow("Relogio (RTC DS1307)", sysStatus.rtc);
  addRow("Cache Flash Externa (W25Q16)", sysStatus.ext_flash);
  addRow("Cache Flash Interna (LittleFS)", sysStatus.int_flash);
  addRow("ADS1256 (Loop 4-20mA)", sysStatus.a_1256);
  addRow("ADS1115-A (Volt)", sysStatus.a_1115A);
  addRow("ADS1115-B (Volt)", sysStatus.a_1115B);
  addRow("ADS1115-C (Volt)", sysStatus.a_1115C);
  addRow("ADS1115-D (Volt)", sysStatus.a_1115D);
  addRow("Display GFX 0 (SPI)", sysStatus.gfx0);
  addRow("Display GFX 1 (I2C)", sysStatus.gfx1);
  addRow("Display GFX 2 (I2C)", sysStatus.gfx2);

  html += "</table></div></body></html>";  // Removemos o link 'Voltar'

  request->send(200, "text/html", html);
}

/**
 * @brief Processa todas as requisições para /files (Listar, Download, Renomear, Excluir).
 * ESTA FUNÇÃO É PROTEGIDA POR MUTEX.
 */
void handleFileManager(AsyncWebServerRequest *request) {

  // Tenta pegar o "bastão" do SD Card. Espera no máximo 2 segundos.
  if (xSemaphoreTake(sdMutex, (TickType_t)2000 / portTICK_PERIOD_MS) != pdTRUE) {
    request->send(503, "text/plain", "Servidor ocupado (SD em uso). Tente novamente.");
    return;
  }

  // --- TEMOS O MUTEX ---

  // (As rotas de DELETE, RENAME, e DOWNLOAD não mudam) [cite: 955-963]
  // --- ROTA 3: Handle DELETE (COM CORREÇÃO DO /) ---
  if (request->hasParam("delete")) {
    String filename = "/" + request->getParam("delete")->value();
    log_print("Arquivo a ser deletado: ");
    log_println(filename.c_str());
    if (SD.remove(filename)) request->send(200, "text/plain", "Deletado");
    else request->send(500, "text/plain", "Erro ao deletar");
  }
  // --- ROTA 4: Handle RENAME (COM CORREÇÃO DO /) ---
  else if (request->hasParam("rename_from") && request->hasParam("rename_to")) {
    String from = "/" + request->getParam("rename_from")->value();
    String to = "/" + request->getParam("rename_to")->value();
    log_print("Arquivo a sera renomeado de: ");
    log_print(from.c_str());
    log_print(" para: ");
    log_println(to.c_str());
    if (SD.rename(from, to)) request->send(200, "text/plain", "Renomeado");
    else request->send(500, "text/plain", "Erro ao renomear");
  }
  // --- ROTA 2: Handle DOWNLOAD (COM CORREÇÃO DO /) ---
  else if (request->hasParam("download")) {
    String filename = "/" + request->getParam("download")->value();
    log_print("Arquivo para download: ");
    log_println(filename.c_str());
    if (SD.exists(filename)) {
      request->send(SD, filename, "application/octet-stream", true);
    } else {
      request->send(404, "text/plain", "Arquivo nao encontrado");
    }
  }

  // --- ROTA 1: Handle LIST FILES (Página Principal) ---
  else {
    String html = "<html><head><title>Arquivos - ePAC</title>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial,sans-serif;margin:0;padding:0;}";
    html += ".navbar{background-color:#333;overflow:hidden;}";
    html += ".navbar a{float:left;display:block;color:white;text-align:center;padding:14px 16px;text-decoration:none;}";
    html += ".navbar a:hover{background-color:#ddd;color:black;}";
    html += ".content{padding:20px;}";
    html += "table{border-collapse:collapse;width:100%;}";
    html += "td,th{border:1px solid #ddd;padding:8px;text-align:left;} th{background-color:#f2f2f2;}";
    html += ".btn{padding:5px 10px;text-decoration:none;border-radius:4px;color:white;font-size:0.9em;cursor:pointer;}";
    html += ".btn-download{background-color:#007bff;} .btn-rename{background-color:#ffc107;color:black;} .btn-delete{background-color:#dc3545;}";
    html += ".file-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;}";
    html += ".upload-form button{background-color:#28a745;color:white;border:none;padding:8px 12px;border-radius:4px;cursor:pointer;}";

    // --- CSS PARA A MENSAGEM DE STATUS ---
    html += "#upload_status{color:blue;font-weight:bold;margin-top:10px;display:none;}";

    html += "</style></head><body>";

    html += "<div class='navbar'>";
    html += "<a href='/home'>Home</a>";
    html += "<a href='/status'>Status</a>";
    html += "<a href='/files'>Arquivos</a>";
    html += "<a href='/config'>Configurações</a>";
    html += "<a href='/wifi'>WiFi</a>";
    html += "<a href='/'>Sair</a>";
    html += "</div>";

    html += "<div class='content'>";
    html += "<div class='file-header'>";
    html += "<h1>Gerenciador de Arquivos (SD Card)</h1>";
    html += "<form action='/upload' method='POST' enctype='multipart/form-data' class='upload-form'>";
    html += "<input type='file' name='upload_file' id='file_upload' required>";
    html += "<button type'submit'>Enviar</button>";
    html += "</form>";
    html += "</div>";

    // --- HTML PARA A MENSAGEM DE STATUS ---
    html += "<div id='upload_status'>Fazendo upload, aguarde...</div>";

    html += "<table><thead><tr><th>Arquivo</th><th>Tamanho (bytes)</th><th>Ações</th></tr></thead><tbody>";

    File root = SD.open("/");
    if (!root) {
      html += "<tr><td colspan='3' style='color:red;'>Falha ao abrir o diretorio raiz do SD Card!</td></tr>";
    } else {
      File file = root.openNextFile();
      int count = 0;
      while (file) {
        count++;
        String fName = String(file.name());
        if (fName.startsWith("/")) { fName = fName.substring(1); }
        html += "<tr><td>" + fName + "</td>";
        html += "<td>" + String(file.size()) + "</td>";
        html += "<td>";
        html += "<a href='/files?download=" + fName + "' class='btn btn-download'>Download</a> ";
        html += "<a href='#' class='btn btn-rename' data-filename='" + fName + "'>Renomear</a> ";
        html += "<a href='#' class='btn btn-delete' data-filename='" + fName + "'>Excluir</a>";
        html += "</td></tr>";
        file.close();
        file = root.openNextFile();
      }
      file.close();
      root.close();
      if (count == 0) {
        html += "<tr><td colspan='3'>Nenhum arquivo encontrado no SD Card.</td></tr>";
      }
    }
    html += "</tbody></table></div>";

    // --- JAVASCRIPT ATUALIZADO ---
    html += "<script>";

    // 1. Conecta ao WebSocket
    html += "const ws = new WebSocket(`ws://${window.location.host}/ws`);";
    html += "ws.onopen = () => console.log('WS Conectado na pag de arquivos');";
    html += "ws.onclose = () => console.log('WS Desconectado');";

    // 2. Listeners dos botões Renomear/Excluir (como antes) [cite: 982-985]
    html += "document.querySelectorAll('.btn-rename').forEach(btn => {";
    html += "  btn.addEventListener('click', (e) => {";
    html += "    e.preventDefault();";
    html += "    const oldName = e.target.getAttribute('data-filename');";
    html += "    const newName = prompt('Digite o novo nome para ' + oldName, oldName);";
    html += "    if (newName && newName !== oldName) {";
    html += "      fetch(`/files?rename_from=${encodeURIComponent(oldName)}&rename_to=${encodeURIComponent(newName)}`).then(response => {";
    html += "        if(response.ok) location.reload(); else alert('Erro ao renomear');";
    html += "      });";
    html += "    }";
    html += "  });";
    html += "});";
    html += "document.querySelectorAll('.btn-delete').forEach(btn => {";
    html += "  btn.addEventListener('click', (e) => {";
    html += "    e.preventDefault();";
    html += "    const filename = e.target.getAttribute('data-filename');";
    html += "    if (confirm('Tem certeza que deseja excluir ' + filename + '?')) {";
    html += "      fetch(`/files?delete=${encodeURIComponent(filename)}`).then(response => {";
    html += "        if(response.ok) location.reload(); else alert('Erro ao excluir');";
    html += "      });";
    html += "    }";
    html += "  });";
    html += "});";

    // 3. LÓGICA DO UPLOAD
    html += "const form = document.querySelector('.upload-form');";
    html += "const uploadStatus = document.getElementById('upload_status');";

    // 3a. Mostrar a mensagem "Aguarde..." ao enviar
    html += "form.addEventListener('submit', () => {";
    html += "  uploadStatus.style.display = 'block';";
    html += "});";

    // 3b. Ouvir a mensagem de "completo" do WebSocket
    html += "ws.onmessage = (event) => {";
    html += "  try { const data = JSON.parse(event.data);";
    // Ouve a nova mensagem 'upload_complete'
    html += "    if (data.type === 'upload_complete') {";
    html += "      uploadStatus.textContent = 'Upload concluído! Recarregando...';";
    html += "      setTimeout(() => { location.reload(); }, 1000);";  // Recarrega
    html += "    }";
    html += "  } catch(e) {}";  // Ignora outras msgs (lcd, serial)
    html += "};";

    html += "</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  }

  // --- DEVOLVE O MUTEX ---
  xSemaphoreGive(sdMutex);
}

/**
 * @brief Handler de Upload de Arquivo (Body Handler).
 * Esta função é chamada para cada "pedaço" do arquivo.
 */
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {

  // 1. Pega o "bastão" do SD Card
  if (xSemaphoreTake(sdMutex, (TickType_t)2000 / portTICK_PERIOD_MS) != pdTRUE) {
    log_println("ERRO UPLOAD: SD em uso, chunk perdido.");
    return;
  }

  // --- INÍCIO DO BLOCO PROTEGIDO ---

  if (index == 0) {
    // Início do upload
    log_print("Upload iniciado: ");
    String filepath = "/" + filename;
    log_println(filepath.c_str());

    if (SD.exists(filepath)) {
      log_println("Arquivo existente, sera sobrescrito.");
      SD.remove(filepath);
    }
    uploadFile = SD.open(filepath, FILE_WRITE);
    if (!uploadFile) {
      log_println("ERRO: Nao foi possivel criar o arquivo no SD.");
      xSemaphoreGive(sdMutex);
      return;
    }
    log_println("Arquivo de upload aberto.");
    log_println("Uploading.");
  }

  // Escreve o bloco de dados
  if (uploadFile) {
    log_print(".");
    uploadFile.write(data, len);
  }

  // Fim do upload
  if (final) {
    if (uploadFile) {
      uploadFile.close();
      log_println("Upload concluído.");

      // --- MUDANÇA ---
      // Envia uma única mensagem de "concluído"
      ws.textAll("{\"type\":\"upload_complete\"}");
      // --- FIM DA MUDANÇA ---

    } else {
      log_println("ERRO UPLOAD: O arquivo nao estava aberto.");
    }
  }

  // DEVOLVE O "bastão" do SD Card APÓS CADA CHUNK
  xSemaphoreGive(sdMutex);
  // --- FIM DO BLOCO PROTEGIDO ---
}

/**
 * @brief Exibe a página de Configurações Gerais, preenchendo com valores atuais.
 */
void handleConfigPage(AsyncWebServerRequest *request) {
  // Carrega o template do LittleFS
  File file = LittleFS.open("/config.html", "r");
  if (!file) {
    log_println("ERRO: Nao foi possivel abrir /config.html");
    return request->send(500, "text/plain", "ERRO: Template nao encontrado");
  }
  String html = file.readString();
  file.close();

  // 1. Preenche Amostragem (O resto da função é igual)
  html.replace("%%s_time%%", String(cfg.s_time));
  html.replace("%%s_num%%", String(cfg.s_num));
  // 2. Preenche Checkboxes de ADCs
  html.replace("%%a_1256_checked%%", cfg.a_1256 ? "checked" : "");
  html.replace("%%a_1115A_checked%%", cfg.a_1115A ? "checked" : "");
  html.replace("%%a_1115B_checked%%", cfg.a_1115B ? "checked" : "");
  html.replace("%%a_1115C_checked%%", cfg.a_1115C ? "checked" : "");
  html.replace("%%a_1115D_checked%%", cfg.a_1115D ? "checked" : "");
  // 3. Preenche Checkboxes de Tempo
  html.replace("%%time_abs_checked%%", cfg.time_abs ? "checked" : "");
  html.replace("%%time_abs_raw_checked%%", cfg.time_abs_raw ? "checked" : "");
  html.replace("%%time_rel_h_checked%%", cfg.time_rel_h ? "checked" : "");
  html.replace("%%time_rel_s_checked%%", cfg.time_rel_s ? "checked" : "");
  // 4. Envia a página preenchida
  request->send(200, "text/html", html);
}

/**
 * @brief Recebe o POST do formulário de Configurações e salva.
 */
void handleConfigSave(AsyncWebServerRequest *request) {
  log_println("Recebendo novas configurações...");

  // IMPORTANTE: Zera todos os booleanos antes de processar.
  // Se um checkbox não for marcado, ele não é enviado no formulário.
  // Então, assumimos 'false' a menos que o parâmetro 'true' seja recebido.
  cfg.a_1256 = false;
  cfg.a_1115A = false;
  cfg.a_1115B = false;
  cfg.a_1115C = false;
  cfg.a_1115D = false;
  cfg.time_abs = false;
  cfg.time_abs_raw = false;
  cfg.time_rel_h = false;
  cfg.time_rel_s = false;

  int params = request->params();
  for (int i = 0; i < params; i++) {
    const AsyncWebParameter *p = request->getParam(i);
    String name = p->name();
    String val = p->value();

    // Amostragem (Inteiros)
    if (name == "s_time") cfg.s_time = val.toInt();
    if (name == "s_num") cfg.s_num = val.toInt();

    // ADCs (Booleans)
    if (name == "a_1256") cfg.a_1256 = true;
    if (name == "a_1115A") cfg.a_1115A = true;
    if (name == "a_1115B") cfg.a_1115B = true;
    if (name == "a_1115C") cfg.a_1115C = true;
    if (name == "a_1115D") cfg.a_1115D = true;

    // Tempo (Booleans)
    if (name == "time_abs") cfg.time_abs = true;
    if (name == "time_abs_raw") cfg.time_abs_raw = true;
    if (name == "time_rel_h") cfg.time_rel_h = true;
    if (name == "time_rel_s") cfg.time_rel_s = true;
  }

  // Tenta salvar no SD Card (a função saveConfiguration() já tem o mutex)
  if (saveConfiguration()) {
    log_println("Configuração salva com sucesso.");
    // Redireciona de volta para a página de config
    request->redirect("/config");
  } else {
    log_println("ERRO: Falha ao salvar configuração.");
    request->send(500, "text/plain", "Erro ao salvar configuracao. Verifique o Serial Monitor.");
  }
}

/**
 * @brief Exibe a página de Calibração de Loop.
 */
void handleConfigLoopPage(AsyncWebServerRequest *request) {
  // Carrega o template do LittleFS
  File file = LittleFS.open("/config_loop.html", "r");
  if (!file) {
    log_println("ERRO: Nao foi possivel abrir /config_loop.html");
    return request->send(500, "text/plain", "ERRO: Template nao encontrado");
  }
  String html = file.readString();
  file.close();

  // O resto da função é igual
  String options = "";
  String cal_n_options = "";
  String cal_sam_options = "";
  String config_json = "[";

  for (int i = 0; i < 8; i++) {
    options += "<option value='";
    options += String(i);
    options += "'>";
    options += String(cfg.loop_title[i]) + " (S" + String(i) + ")";
    options += "</option>";
    config_json += "{\"a\":" + String(cfg.loop_cal[i][0], 8) + ",\"b\":" + String(cfg.loop_cal[i][1], 8) + ",\"is_active\":" + (cfg.loop_a[i] ? "true" : "false") + ",\"is_enabled\":" + (cfg.a_1256 ? "true" : "false") + "}";
    if (i < 7) config_json += ",";
  }
  config_json += "]";
  for (int n = 0; n <= 8; n++) {
    cal_n_options += "<option value='" + String(n) + "'>" + String(pow(2, n) + 1) + " pontos</option>";
  }
  for (int n = 0; n <= 20; n++) {
    String selected = (n == 10) ? "selected" : "";
    cal_sam_options += "<option value='" + String(n) + "' " + selected + ">" + String(pow(2, n)) + " amostras</option>";
  }

  html.replace("%%CHANNEL_OPTIONS%%", options);
  html.replace("%%CAL_N_OPTIONS%%", cal_n_options);
  html.replace("%%CAL_SAM_OPTIONS%%", cal_sam_options);
  html.replace("%%CONFIG_JSON%%", config_json);
  html.replace("%%cur_a%%", String(cfg.loop_cal[0][0], 6));
  html.replace("%%cur_b%%", String(cfg.loop_cal[0][1], 6));

  request->send(200, "text/html", html);
}

/**
 * @brief Salva os novos valores de calibração do loop.
 */
void handleConfigLoopSave(AsyncWebServerRequest *request) {

  // 1. Pede os parâmetros especificamente do POST (usando 'true')
  const AsyncWebParameter *pCh = request->getParam("ch", true);
  const AsyncWebParameter *pA = request->getParam("a", true);
  const AsyncWebParameter *pB = request->getParam("b", true);

  // 2. Verifica se os ponteiros são nulos (se os parâmetros não foram encontrados)
  if (!pCh || !pA || !pB) {
    request->send(400, "text/plain", "Faltando parametros. (Verifique se pCh, pA ou pB sao nulos)");
    return;
  }

  // 3. Obtém os valores dos parâmetros que sabemos que existem
  int ch = pCh->value().toInt();
  double a = pA->value().toFloat();
  double b = pB->value().toFloat();

  log_print("Salvando nova calibracao (Web) para Canal ");
  log_println(ch);
  log_print("A (alpha): ");
  dtostrf(a, 10, 8, mtxt);
  log_println(mtxt);

  log_print("B (beta): ");
  dtostrf(b, 10, 8, mtxt);
  log_println(mtxt);

  // Salva na struct cfg
  cfg.loop_cal[ch][0] = a;
  cfg.loop_cal[ch][1] = b;

  // Salva no SD Card (esta função já tem o mutex)
  if (saveConfiguration()) {
    log_println("Configuracao salva no SD!");
    request->redirect("/config_loop");
  } else {
    log_println("ERRO: Falha ao salvar no SD.");
    request->send(500, "text/plain", "Erro ao salvar no SD Card.");
  }
}

/**
 * @brief (NOVA FUNÇÃO) Exibe a página de Calibração de Voltagem.
 */
void handleConfigVoltPage(AsyncWebServerRequest *request) {
  // Carrega o template do LittleFS
  File file = LittleFS.open("/config_volt.html", "r");
  if (!file) {
    log_println("ERRO: Nao foi possivel abrir /config_volt.html");
    return request->send(500, "text/plain", "ERRO: Template nao encontrado");
  }
  String html = file.readString();
  file.close();

  // O resto da função é igual
  String options = "";
  String cal_n_options = "";
  String cal_sam_options = "";
  String config_json = "[";

  for (int i = 0; i < 8; i++) {
    options += "<option value='";
    options += String(i);
    options += "'>";
    options += String(cfg.volt_title[i]) + " (X" + String(i) + ")";
    options += "</option>";

    config_json += "{";
    config_json += "\"title\":\"" + String(cfg.volt_title[i]) + "\",";
    config_json += "\"a_12v\":" + String(cfg.volt_cal[i][0], 8) + ",";
    config_json += "\"b_12v\":" + String(cfg.volt_cal[i][1], 8) + ",";
    config_json += "\"a_5v\":" + String(cfg.volt_cal[i][2], 8) + ",";
    config_json += "\"b_5v\":" + String(cfg.volt_cal[i][3], 8) + ",";
    config_json += "\"is_5v\":" + String(cfg.volt_a[i] ? "true" : "false");
    int fadc = (i / 2) + 1;
    config_json += ",\"is_enabled\":" + String(isADCEnabled(fadc) ? "true" : "false");
    config_json += "}";
    if (i < 7) config_json += ",";
  }
  config_json += "]";

  for (int n = 0; n <= 8; n++) {
    cal_n_options += "<option value='" + String(n) + "'>" + String(pow(2, n) + 1) + " pontos</option>";
  }
  for (int n = 0; n <= 20; n++) {
    String selected = (n == 10) ? "selected" : "";
    cal_sam_options += "<option value='" + String(n) + "' " + selected + ">" + String(pow(2, n)) + " amostras</option>";
  }

  html.replace("%%CHANNEL_OPTIONS%%", options);
  html.replace("%%CAL_N_OPTIONS%%", cal_n_options);
  html.replace("%%CAL_SAM_OPTIONS%%", cal_sam_options);
  html.replace("%%CONFIG_JSON%%", config_json);

  bool is_5v = cfg.volt_a[0];
  int a_idx = is_5v ? 2 : 0;
  int b_idx = is_5v ? 3 : 1;
  html.replace("%%cur_a%%", String(cfg.volt_cal[0][a_idx], 6));
  html.replace("%%cur_b%%", String(cfg.volt_cal[0][b_idx], 6));

  request->send(200, "text/html", html);
}

/**
 * @brief (NOVA FUNÇÃO) Salva os novos valores de calibração de voltagem.
 */
void handleConfigVoltSave(AsyncWebServerRequest *request) {

  // 1. Pede os parâmetros especificamente do POST
  const AsyncWebParameter *pCh = request->getParam("ch", true);
  const AsyncWebParameter *pA = request->getParam("a", true);
  const AsyncWebParameter *pB = request->getParam("b", true);
  const AsyncWebParameter *pIs5V = request->getParam("is_5v", true);  // Pega o novo parâmetro

  // 2. Verifica se os ponteiros são nulos
  if (!pCh || !pA || !pB || !pIs5V) {
    request->send(400, "text/plain", "Faltando parametros.");
    return;
  }

  // 3. Obtém os valores
  int ch = pCh->value().toInt();
  double a = pA->value().toFloat();
  double b = pB->value().toFloat();
  bool is_5v = (pIs5V->value() == "true");

  log_print("Salvando nova calibracao (Web) para Canal VOLT ");
  log_println(ch);
  log_print("Modo: ");
  log_println(is_5v ? "5V" : "12V");

  // Determina os índices corretos no array cfg.volt_cal[8][4]
  int a_idx = is_5v ? 2 : 0;  // Se 5V, usa índice 2, senão 0
  int b_idx = is_5v ? 3 : 1;  // Se 5V, usa índice 3, senão 1

  log_print("A (alpha): ");
  dtostrf(a, 10, 8, mtxt);
  log_println(mtxt);

  log_print("B (beta): ");
  dtostrf(b, 10, 8, mtxt);
  log_println(mtxt);

  // Salva na struct cfg nos índices corretos
  cfg.volt_cal[ch][a_idx] = a;
  cfg.volt_cal[ch][b_idx] = b;

  // Salva no SD Card
  if (saveConfiguration()) {
    log_println("Configuracao salva no SD!");
    request->redirect("/config_volt");  // Redireciona para a página de voltagem
  } else {
    log_println("ERRO: Falha ao salvar no SD.");
    request->send(500, "text/plain", "Erro ao salvar no SD Card.");
  }
}
/**
 * @brief (NOVA FUNÇÃO) Verifica se um ADC está habilitado na configuração.
 * @param fadc O índice do ADC (0=1256, 1=1115A, 2=1115B, 3=1115C, 4=1115D)
 * @return true se o ADC estiver ativo (cfg.a_... == true), false caso contrário.
 */
bool isADCEnabled(int fadc) {
  switch (fadc) {
    case 0: return cfg.a_1256;   //
    case 1: return cfg.a_1115A;  //
    case 2: return cfg.a_1115B;  //
    case 3: return cfg.a_1115C;  //
    case 4: return cfg.a_1115D;  //
    default: return false;
  }
}

/**
 * @brief (NOVA FUNÇÃO) Exibe a página de Configuração WiFi.
 */
void handleWifiPage(AsyncWebServerRequest *request) {
  File file = LittleFS.open("/wifi.html", "r");
  if (!file) {
    log_println("ERRO: Nao foi possivel abrir /wifi.html");
    return request->send(500, "text/plain", "ERRO: Template nao encontrado");
  }
  String html = file.readString();
  file.close();

  // Busca as credenciais salvas (se existirem)
  html.replace("%%WIFI_SSID%%", wifi_ssid);
  html.replace("%%WIFI_PASS%%", wifi_pass);

  // Checa se há uma mensagem de status (ex: falha ao salvar)
  if (request->hasParam("status")) {
    if (request->getParam("status")->value() == "saved") {
      html.replace("%%STATUS_MSG%%", "<span class='success'>Configurações salvas. O ePAC irá reiniciar para tentar conectar.</span>");
    } else {
      html.replace("%%STATUS_MSG%%", "<span class='error'>Falha ao salvar configurações.</span>");
    }
  } else {
    html.replace("%%STATUS_MSG%%", "");
  }

  request->send(200, "text/html", html);
}

/**
 * @brief (NOVA FUNÇÃO) Salva as credenciais WiFi e reinicia.
 */
void handleWifiSave(AsyncWebServerRequest *request) {
  if (!request->hasParam("ssid", true)) {
    return request->send(400, "text/plain", "Faltando SSID.");
  }

  // Salva os novos valores na NVS (Preferences)
  preferences.begin(NVS_NAMESPACE, false);  // Abre NVS para escrita

  // Salva o SSID
  size_t ssid_ok = preferences.putString("wifi_ssid", request->getParam("ssid", true)->value());

  // Salva a Senha (se existir)
  size_t pass_ok = 0;
  if (request->hasParam("pass", true)) {
    pass_ok = preferences.putString("wifi_pass", request->getParam("pass", true)->value());
  } else {
    pass_ok = preferences.putString("wifi_pass", "");  // Salva senha vazia
  }

  preferences.end();  // Fecha e salva (não retorna nada)

  // --- INÍCIO DA CORREÇÃO (bool ok) ---
  // Verifica se as escritas (putString) foram bem-sucedidas
  if (ssid_ok > 0) {
    // --- FIM DA CORREÇÃO ---

    log_println("Novas credenciais WiFi salvas. Reiniciando...");

    request->redirect("/wifi?status=saved");

    delay(1000);    // Dá tempo para a resposta HTTP ser enviada
    ESP.restart();  // Reinicia para aplicar

  } else {
    log_println("ERRO: Falha ao salvar credenciais WiFi na NVS.");
    request->redirect("/wifi?status=error");
  }
}

void setup(void) {

  // Devolve o CS para HIGH para a biblioteca não bugar depois
  digitalWrite(sdcard_pin, HIGH);
  // ----------------------------------------------------------
  sdMutex = xSemaphoreCreateMutex();
  // --- INÍCIO DA CORREÇÃO DE SPI (Tentativa V3) ---
  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
    // 1. Força CS para HIGH (como antes)
    pinMode(sdcard_pin, OUTPUT);
    pinMode(W25Q16_CS_PIN, OUTPUT);
    pinMode(hspi2_pin, OUTPUT);
    pinMode(cs_1256, OUTPUT);
    digitalWrite(sdcard_pin, HIGH);
    digitalWrite(W25Q16_CS_PIN, HIGH);
    digitalWrite(hspi2_pin, HIGH);
    digitalWrite(cs_1256, HIGH);

    // 2. FORÇA O RESET DO BARRAMENTO SPI
    //    SPI.end() desliga o periférico VSPI.
    SPI.end();

    // 3. *** NOVO "KICK" DE HARDWARE VIA SOFTWARE ***
    //    Inicializa o SPI em baixa velocidade e envia 80+ clocks (10 bytes)
    //    com AMBOS os CS em HIGH. Isso é um requisito do "Modo SPI"
    //    para forçar qualquer cartão SD a sair de um estado "sujo"
    //    e voltar ao modo IDLE.
    SPI.begin();                                                     // Inicializa o barramento
    SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));  // baixa velocidade
    for (int i = 0; i < 10; i++) {
      SPI.transfer(0xFF);
    }
    SPI.endTransaction();
    SPI.end();  // Desliga o barramento de novo, SD.begin() vai religar.
    // --- FIM DA CORREÇÃO ---
    xSemaphoreGive(sdMutex);  // Devolve o mutex após as checagens
  }

  Serial.begin(115200);
  randomSeed(analogRead(0));


  lcd.init();
  Wire.beginTransmission(e_lcd);
  if (Wire.endTransmission() != 0) {
    log_print("    LCD#    ");
    fatal_error();
  } else {
    log_print("    LCD!    ");
    lcd.clear();
    lcd.setBacklight(HIGH);
    lcd.setCursor(0, 0);
    lcd.print("   e-PAC v3.1   ");
    sysStatus.lcd = true;
  }

  // --- CONFIGURAÇÃO DO "LAST GASP" ---
  // Usando pull-up, a lógica do optoacoplador é:
  // 24V ON = LOW
  // 24V OFF = HIGH
  pinMode(POWER_FAIL_PIN, INPUT_PULLUP);
  // Queremos a interrupção quando o pino for para HIGH
  attachInterrupt(digitalPinToInterrupt(POWER_FAIL_PIN), isr_powerFail, RISING);
  // ------------------------------------


  // --- INÍCIO: Lógica de Conexão WiFi (CORRIGIDO) ---
  lcd.setCursor(0, 1);
  lcd.print("   Rede WiFi?   ");
  delay(t_delay);

  // 1. CORREÇÃO: Inicializa o Preferences (NVS) ANTES de ler.
  // A função initExternalFlash() fazia isso, mas agora está desabilitada.
  if (!preferences.begin(NVS_NAMESPACE, false)) {  // false = read/write
    log_println("ERRO FATAL: Nao foi possivel iniciar a NVS.");
    lcd.print(" ERRO NVS ");
    delay(t_delay);
    // Não damos fatal_error() aqui, mas o WiFi não vai funcionar.
  }

  // 2. Carrega as credenciais salvas da NVS
  wifi_ssid = preferences.getString("wifi_ssid", "");
  wifi_pass = preferences.getString("wifi_pass", "");
  // Não fechamos (preferences.end()) para que 'handleWifiSave' possa usá-lo.
  // Se 'initExternalFlash' for reativado, ele já faz o 'begin'.

  bool connected = false;
  if (wifi_ssid.length() > 0) {
    // 3. Tenta conectar (Modo STA) se tiver um SSID salvo
    log_print("Tentando conectar ao WiFi: ");
    log_println(wifi_ssid);
    lcd.setCursor(0, 1);
    lcd.print(wifi_ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
      delay(t_delay);
      log_print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      // 4. Conseguiu conectar!
      connected = true;
      log_println("\nConectado! Endereco IP: ");
      log_println(WiFi.localIP());
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WiFi Conectado!");
      lcd.setCursor(0, 1);
      lcd.print(WiFi.localIP());
      delay(2 * t_delay);
    } else {
      // 5. Falha ao conectar.
      log_println("\nFalha ao conectar. Revertendo para Modo AP.");
      lcd.setCursor(0, 1);
      lcd.print(" Falha na conexao ");
      delay(t_delay);
      WiFi.mode(WIFI_OFF);
    }
  } else {
    log_println("Nenhuma credencial WiFi salva.");
  }

  // 6. Se não conectou, inicia o Modo AP (Access Point)
  if (!connected) {
    lcd.setCursor(0, 1);
    lcd.print("   AP Server?   ");
    log_println("Configurando Access Point (AP)...");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid);

    log_print("AP iniciado. IP: ");
    log_println(WiFi.softAPIP());

    dnsServer.start(53, "*", WiFi.softAPIP());

    lcd.setCursor(0, 1);
    lcd.print("   AP Server!   ");
    delay(t_delay);
  }
  // --- FIM: Lógica de Conexão WiFi ---


  // --- INÍCIO: Configuração do WebServer (sempre inicia) ---

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/login.html", "text/html");
  });
  server.on("/home", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/home.html", "text/html");
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    if (WiFi.getMode() == WIFI_AP) {
      request->send(LittleFS, "/login.html", "text/html");
    } else {
      request->send(404, "text/plain", "Nao encontrado");
    }
  });

  server.on("/status", HTTP_GET, handleStatusPage);
  server.on("/files", HTTP_GET, handleFileManager);

  server.on(
    "/upload", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      request->redirect("/files");
    },
    handleUpload);

  server.on("/config", HTTP_GET, handleConfigPage);
  server.on("/config_save", HTTP_POST, handleConfigSave);
  server.on("/config_loop", HTTP_GET, handleConfigLoopPage);
  server.on("/config_loop_save", HTTP_POST, handleConfigLoopSave);
  server.on("/config_volt", HTTP_GET, handleConfigVoltPage);
  server.on("/config_volt_save", HTTP_POST, handleConfigVoltSave);
  server.on("/wifi", HTTP_GET, handleWifiPage);
  server.on("/wifi_save", HTTP_POST, handleWifiSave);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();  // Inicia o servidor

  //Iniciar SDcard
  lcd.setCursor(0, 1);
  lcd.print("    SD-card?    ");

  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {

    bool sdf = SD.begin(sdcard_pin);
    if (!sdf) {
      lcd.setCursor(0, 1);
      lcd.print("    SD-card#    ");
      log_print("    SD-card#    ");
      fatal_error();
    } else {
      lcd.print("    SD-card!    ");
      log_print("    SD-card!    ");
      sysStatus.sd_card = true;
      lcd.setCursor(0, 1);
      lcd.print("     State?     ");

      if (!SD.exists("/state.json")) {
        lcd.setCursor(0, 1);
        lcd.print("     State#     ");
        log_print("     State#     ");
      } else {
        if (!loadState()) {
          lcd.setCursor(0, 1);
          lcd.print("   Erro-state#    ");
          log_print("   Erro-state#    ");
          fatal_error();
        } else {
          lcd.setCursor(0, 1);
          lcd.print("     State!     ");
          log_print("     State!     ");
          sysStatus.state_file = true;
        }
      }


      //Carregar cfgs
      lcd.print("    configs?    ");
      if (!SD.exists("/config.json")) {
        lcd.setCursor(0, 1);
        lcd.print("    configs#    ");
        log_print("    configs#    ");
        fatal_error();
      }
      if (!loadConfiguration()) {
        lcd.setCursor(0, 1);
        lcd.print("    configs#    ");
        log_print("    configs#    ");
        fatal_error();
      } else {
        lcd.setCursor(0, 1);
        lcd.print("    configs!    ");
        log_print("    configs!    ");
        sysStatus.config_file = true;
        //    if (cfg.s_num > 512) {
        //      lcd.setCursor(0, 1);
        //      lcd.print("!s_num max= 512!");
        //      cfg.s_num = 512;
        //      delay(1000);
        //    }
        int est_s_time = cfg.s_num * 1.2;  //1.2s/amostragem
        if (est_s_time > cfg.s_time) {
          lcd.setCursor(0, 1);
          lcd.print(" !s_time baixo! ");
          cfg.s_time = cfg.s_num * 2;
          delay(t_delay);
        }
      }
    }
    xSemaphoreGive(sdMutex);  // Devolve o mutex após as checagens
  }

  //Iniciar 8-Reles (PCF8574)
  lcd.setCursor(0, 1);
  lcd.print("    8-Reles?    ");

  reles.begin();
  if (!reles.isConnected()) {
    lcd.setCursor(0, 1);
    lcd.print("    8-Reles#    ");
    log_print("    8-Reles#    ");
    fatal_error();
  } else {
    lcd.setCursor(0, 1);
    lcd.print("    8-Reles!    ");
    log_print("    8-Reles!    ");
    sysStatus.reles = true;
  }

  //Iniciar Relogio
  lcd.setCursor(0, 1);
  lcd.print("      RTC?      ");
  bool clockf = rtc.begin();
  if (!clockf) {
    lcd.setCursor(0, 1);
    lcd.print("      RTC#      ");
    log_print("      RTC#      ");
    fatal_error();
  } else {
    lcd.setCursor(0, 1);
    if (!rtc.isrunning()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  //CAPTURA A DATA E HORA EM QUE O SKETCH É COMPILADO
      //rtc.adjust(DateTime(2018, 7, 5, 15, 33, 15)); //(ANO), (MÊS), (DIA), (HORA), (MINUTOS), (SEGUNDOS)
      lcd.print("Data/hora ajust");
    }
    lcd.setCursor(0, 1);
    now = rtc.now();
    sprintf(mtxt, "%02d/%02d/%04d %02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute());
    lcd.print(mtxt);
    delay(t_delay);
    lcd.setCursor(0, 1);
    lcd.print("      RTC!      ");
    log_print("      RTC!      ");
    sysStatus.rtc = true;
  }

  //Iniciar ADSs
  /*
     ADS1115_RANGE_6144  ->  +/- 6144 mV
     ADS1115_RANGE_4096  ->  +/- 4096 mV
     ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
     ADS1115_RANGE_1024  ->  +/- 1024 mV
     ADS1115_RANGE_0512  ->  +/- 512 mV
     ADS1115_RANGE_0256  ->  +/- 256 mV
      ADS1115_8_SPS
      ADS1115_16_SPS
      ADS1115_32_SPS
      ADS1115_64_SPS
      ADS1115_128_SPS (default)
      ADS1115_250_SPS
      ADS1115_475_SPS
      ADS1115_860_SPS
  */
  bool ads = false;
  if (cfg.a_1115A) {
    lcd.setCursor(0, 1);
    lcd.print("    ADS1115?    ");
    ads = ads1115A.init();

    if (!ads) {
      lcd.setCursor(0, 1);
      lcd.print("    ADS1115#A   ");
      log_print("    ADS1115#A   ");
      fatal_error();
    } else {
      ads1115A.setVoltageRange_mV(ADS1115_RANGE_2048);
      ads1115A.setConvRate(ADS1115_64_SPS);
      lcd.setCursor(0, 1);
      lcd.print("    ADS1115!    ");
      log_print("    ADS1115!    ");
      sysStatus.a_1115A = true;
    }
  }

  if (cfg.a_1115B) {
    lcd.setCursor(0, 1);
    lcd.print("    ADS1115?    ");
    ads = false;
    ads = ads1115B.init();
    if (!ads) {
      lcd.setCursor(0, 1);
      lcd.print("    ADS1115#B   ");
      log_print("    ADS1115#B   ");
      fatal_error();
    } else {
      ads1115B.setVoltageRange_mV(ADS1115_RANGE_2048);
      ads1115B.setConvRate(ADS1115_64_SPS);
      lcd.setCursor(0, 1);
      lcd.print("    ADS1115!    ");
      log_print("    ADS1115!    ");
      sysStatus.a_1115B = true;
    }
  }
  if (cfg.a_1115C) {
    lcd.setCursor(0, 1);
    lcd.print("    ADS1115?    ");
    ads = false;
    ads = ads1115C.init();

    if (!ads) {
      lcd.setCursor(0, 1);
      lcd.print("    ADS1115#C   ");
      log_print("    ADS1115#C   ");
      fatal_error();
    } else {
      ads1115A.setVoltageRange_mV(ADS1115_RANGE_2048);
      ads1115A.setConvRate(ADS1115_64_SPS);
      lcd.setCursor(0, 1);
      lcd.print("    ADS1115!    ");
      log_print("    ADS1115!    ");
      sysStatus.a_1115C = true;
    }
  }
  if (cfg.a_1115D) {
    lcd.setCursor(0, 1);
    lcd.print("    ADS1115?    ");
    ads = false;
    ads = ads1115D.init();

    if (!ads) {
      lcd.setCursor(0, 1);
      lcd.print("    ADS1115#D   ");
      log_print("    ADS1115#D   ");
      fatal_error();
    } else {
      ads1115A.setVoltageRange_mV(ADS1115_RANGE_2048);
      ads1115A.setConvRate(ADS1115_64_SPS);
      lcd.setCursor(0, 1);
      lcd.print("    ADS1115!    ");
      log_print("    ADS1115!    ");
      sysStatus.a_1115D = true;
    }
  }
  if (cfg.a_1256) {
    lcd.setCursor(0, 1);
    lcd.print("    ADS1256?    ");
    ads = false;
    ads1256.InitializeADC();
    ads = (ads1256.readRegister(0) != 0 ? true : false);
    //ADS1256_GAIN_1 +/- 5V  1 bit = 596.046 nV
    //ADS1256_GAIN_2 +/- 2.5V  1 bit = 298.023 nV
    //ADS1256_GAIN_4 +/- 1.25V  1 bit = 149.011 nV
    //ADS1256_GAIN_8 +/- 0.625V  1 bit = 74.5058 nV
    //ADS1256_GAIN_16 +/- 0.3125V  1 bit = 37.2529 nV
    //ADS1256_GAIN_32 +/- 0.15625V  1 bit = 18.6264 nV
    //ADS1256_GAIN_64 +/- 0.078125V  1 bit = 9.31322 nV
    ads1256.setPGA(PGA_2);
    //ADS1256_DRATE_30000SPS
    //ADS1256_DRATE_15000SPS
    //ADS1256_DRATE_7500SPS
    //ADS1256_DRATE_3750SPS
    //ADS1256_DRATE_2000SPS
    //ADS1256_DRATE_1000SPS
    //ADS1256_DRATE_500SPS
    //ADS1256_DRATE_100SPS
    //ADS1256_DRATE_60SPS
    //ADS1256_DRATE_50SPS
    //ADS1256_DRATE_30SPS
    //ADS1256_DRATE_25SPS
    //ADS1256_DRATE_15SPS (erro 0.1mV@0.02V)
    //ADS1256_DRATE_10SPS
    //ADS1256_DRATE_5SPS
    //ADS1256_DRATE_2_5SPS
    ads1256.setDRATE(DRATE_15SPS);
    //ads1256.setBuffer(true);
    if (!ads) {
      lcd.setCursor(0, 1);
      lcd.print("    ADS1256#    ");
      log_print("    ADS1256#    ");
      fatal_error();
    } else {
      lcd.setCursor(0, 1);
      lcd.print("    ADS1256!    ");
      log_print("    ADS1256!    ");
      sysStatus.a_1256 = true;
    }
  }

  //Iniciar Display GFX
  if (cfg.GFX_0) {
    lcd.setCursor(0, 1);
    lcd.print("      GFX?      ");
    //u8g2_t1.setBusClock(960000);
    u8g2_t1.setBusClock(600000);
    u8g2_t1.begin();
    if (false) {
      lcd.setCursor(0, 1);
      lcd.print("      GFX#0     ");
      log_print("      GFX#0     ");
      fatal_error();
    } else {
      lcd.setCursor(0, 1);
      lcd.print("      GFX!      ");
      log_print("      GFX!      ");
      sysStatus.gfx0 = true;
    }
  }
  if (cfg.GFX_1) {
    lcd.setCursor(0, 1);
    lcd.print("      GFX?      ");
    u8g2_s1.setI2CAddress(2 * 0x3C);
    u8g2_s1.begin();
    if (false) {
      lcd.setCursor(0, 1);
      lcd.print("      GFX#1     ");
      log_print("      GFX#1     ");
      fatal_error();
    } else {
      lcd.setCursor(0, 1);
      lcd.print("      GFX!      ");
      log_print("      GFX!      ");
      sysStatus.gfx1 = true;
    }
  }
  if (cfg.GFX_2) {
    lcd.setCursor(0, 1);
    lcd.print("      GFX?      ");
    u8g2_s2.setI2CAddress(2 * 0x3D);
    u8g2_s2.begin();
    if (false) {
      lcd.setCursor(0, 1);
      lcd.print("      GFX#2     ");
      log_print("      GFX#2     ");
      fatal_error();
    } else {
      lcd.setCursor(0, 1);
      lcd.print("      GFX!      ");
      log_print("      GFX!      ");
      sysStatus.gfx2 = true;
    }
  }

  // --- INÍCIO DA LÓGICA DE BUFFER (MODIFICADO) ---

  // 1. Inicializa LittleFS (Obrigatório para a Web UI)
  lcd.setCursor(0, 1);
  lcd.print("  LittleFS?     ");
  if (!LittleFS.begin(true)) {  // 'true' = formatar se não conseguir montar
    lcd.setCursor(0, 1);
    lcd.print("   LittleFS#    ");
    log_println("  LittleFS# (Falha ao montar particao).");
    fatal_error();
  }
  lcd.setCursor(0, 1);
  lcd.print("   LittleFS!   ");
  log_println("   LittleFS!   ");
  sysStatus.int_flash = true;  // Indica que o LittleFS está OK

  // 2. Tenta inicializar o Flash Externo (para o Datalog)
  lcd.setCursor(0, 1);
  lcd.print("   Ext. Flash?  ");
  delay(t_delay);

  useExternalFlash = initExternalFlash();
  sysStatus.ext_flash = useExternalFlash;

  if (useExternalFlash) {
    lcd.setCursor(0, 1);
    lcd.print("   Ext. Flash!  ");
    log_println("  Ext. Flash OK! Usando para cache de log.");
    useInternalFlash = false;  // Não usa LittleFS para cache
  } else {
    lcd.setCursor(0, 1);
    lcd.print("   Ext. Flash#  ");
    log_println("  Ext. Flash #. Usando LittleFS como fallback para cache de log.");
    useInternalFlash = true;  // Usa LittleFS para cache

    // *** SUA LÓGICA DE FALLBACK ***
    if (cfg.s_time < 10) {
      log_print("!!! ATENCAO: Flash interna (fallback) ativa. Forcando s_time de ");
      log_print(cfg.s_time);
      log_println(" para 10s para proteger a flash.");
      cfg.s_time = 10;
    }
  }
  delay(t_delay);
  // --- FIM DA LÓGICA DE BUFFER (MODIFICADO) ---

  pinMode(bdir_pin, INPUT);
  pinMode(besq_pin, INPUT);
  pinMode(bfun_pin, INPUT);

  for (int i = 0; i < xs.size(); i++) xs.push(0);
  for (int i = 0; i < v_t.size(); i++) v_t.push(0);
  lcd.setCursor(0, 1);
  //checa se houve desligamento inesperado
  if (st.a_run and st.t_run > 0) {
    ini = DateTime(st.t_run);
    log_println("Ja estava rodando lembrar de corrigir ");
    //sprintf(mtxt, "/data%05d.csv", st.n_run);
    //File file = SD.open(mtxt, FILE_WRITE);
  }
  lcd.print("  Tudo Pronto!  ");
  delay(t_delay);
  lcd.clear();
  //digitalWrite(rst_t, LOW);
  //digitalWrite(rst_t, HIGH);
  navigate();
  t = millis();

  // --- INÍCIO: Configuração do Hardware Watchdog (TWDT - CORRIGIDO v3) ---

  disableLoopWDT();
  log_println("Assumindo controle do Hardware Watchdog (TWDT)...");

  const uint32_t WATCHDOG_TIMEOUT_MS = DUMP_SD_INTERVAL * 3;

  // Cria o struct de configuração
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT_MS,
    .trigger_panic = true
  };

  // Usa RECONFIGURE em vez de INIT para evitar o erro "already initialized"
  esp_err_t reconfig_err = esp_task_wdt_reconfigure(&twdt_config);
  if (reconfig_err != ESP_OK) {
    log_print("ERRO: Falha ao reconfigurar HW Watchdog: ");
    log_println(esp_err_to_name(reconfig_err));
  }

  // Inscreve a tarefa atual (loopTask do Core 1) no watchdog
  esp_err_t add_err = esp_task_wdt_add(NULL);
  if (add_err != ESP_OK && add_err != ESP_ERR_INVALID_STATE) {
    // Ignora erro "ESP_ERR_INVALID_STATE" que significa "já adicionado"
    log_print("ERRO: Falha ao adicionar task ao HW Watchdog: ");
    log_println(esp_err_to_name(add_err));
  }

  log_print("Hardware Watchdog definido para ");
  log_print(WATCHDOG_TIMEOUT_MS / 1000);
  log_println(" segundos.");
  // --- FIM: Configuração do Hardware Watchdog ---

  xTaskCreatePinnedToCore(
    WebServerTask,     // Função da tarefa
    "WebServerTask",   // Nome (para debug)
    10000,             // Tamanho da Stack (web server precisa de espaço)
    NULL,              // Parâmetros da tarefa
    1,                 // Prioridade
    &h_WebServerTask,  // Handle da tarefa
    0                  // Pinar no Core 0 (PRO_CPU)
  );
}


void loop(void) {
  // --- VERIFICAÇÃO DE FALHA DE ENERGIA (DEVE SER A PRIMEIRA COISA) ---
  if (g_powerFailed) {
    handlePowerFailure();
  }
  // ---------------------------------------------------------------

  now = rtc.now();
  t = millis();

  // --- LÓGICA DE PARADA DE LOG ---
  // Se o log parou, core_menu() [cite: 251] vai setar st.a_run = false
  // Precisamos pegar esse momento para fazer um dump final da RAM.
  bool wasRunning = st.a_run;
  core_menu();
  if (wasRunning && !st.a_run) {
    // O usuário acabou de parar o RUN.
    log_println("RUN parado. Fazendo dump final da RAM para a Flash...");
    dumpRamToFlash();
    log_println("Fazendo dump final da Flash para o SD...");
    dumpFlashToSD();  // Esvazia o cache para o SD
                      // SÓ AGORA incrementa o número do run
    st.n_run++;
    log_print("Proximo run sera: ");
    log_println(st.n_run);
    saveState();
  }
  // ------------------------------

  datalog();
  if (!st.a_run) {
    esp_task_wdt_reset();
  }

  byte rbit = r8 % 256;
  reles.write8(rbit);  //temp PCF8574

  // --- INÍCIO: Lógica de Calibração em Tempo Real (ATUALIZADO COM CHECAGEM DE BUG) ---

  // 1. Stream de Valor Ao Vivo (COM THROTTLE)
  if (g_calStreamChannel != -1) {

    // --- INÍCIO DA CORREÇÃO DE FLOOD (THROTTLE) ---
    // Apenas executa a cada CAL_STREAM_INTERVAL milissegundos
    if (t - t_cal_stream >= CAL_STREAM_INTERVAL) {
      t_cal_stream = t;  // Reseta o timer do stream

      // Mapeia canal/tipo para os parâmetros do readADC()
      int fadc = (g_calStreamADCType == 0) ? 0 : (g_calStreamChannel / 2) + 1;
      int fch = (g_calStreamADCType == 0) ? g_calStreamChannel : (g_calStreamChannel % 2);

      if (!isADCEnabled(fadc)) {
        log_println("ERRO CAL: Tentativa de stream em ADC desabilitado.");
        ws.textAll("{\"type\":\"cal_error\", \"msg\":\"ADC Desabilitado na Configuracao!\"}");
        g_calStreamChannel = -1;  // Para o stream
      } else {

        //long raw = random(1000000); // Linha de teste do usuário
        long raw = readADC(fadc, fch, true);  // Linha de produção

        String json = "{\"type\":\"cal_live\", \"raw\":" + String(raw) + "}";
        ws.textAll(json);
      }
    }
    // --- FIM DA CORREÇÃO DE FLOOD ---
  }

  // 2. Leitura de Precisão (Média)
  if (g_calReadChannel != -1) {
    log_println("Iniciando leitura de precisão...");

    // Mapeia canal/tipo para os parâmetros do readADC()
    int fadc = (g_calReadADCType == 0) ? 0 : (g_calReadChannel / 2) + 1;
    int fch = (g_calReadADCType == 0) ? g_calReadChannel : (g_calReadChannel % 2);

    // --- CORREÇÃO DO BUG ---
    if (!isADCEnabled(fadc)) {
      log_println("ERRO CAL: Tentativa de leitura em ADC desabilitado.");
      ws.textAll("{\"type\":\"cal_error\", \"msg\":\"ADC Desabilitado na Configuracao!\"}");
      g_calReadChannel = -1;  // Cancela a leitura
      g_calReadTarget_i = -1;
    } else {
      // --- FIM DA CORREÇÃO ---
      lcd.setBacklight(LOW);
      double cal_S1 = 0;
      double cal_S2 = 0;
      double cal_np = pow(2, g_calReadSamples_n);

      for (int i = 0; i < cal_np; i++) {
        //double raw_double = (double)random(1000000); // Linha de teste do usuário
        double raw_double = (double)readADC(fadc, fch, (i == 0));
        cal_S1 += raw_double;
        cal_S2 += raw_double * raw_double;
        if (i % 4 == 0) {
          delay(10);  // Cede 1ms para o RTOS (FreeRTOS)
        }
      }

      long avg_raw = (long)(cal_S1 / cal_np);
      double cal_err = sqrt(cal_S2 / cal_np - (cal_S1 / cal_np) * (cal_S1 / cal_np));

      log_print("Leitura concluída. Média Raw: ");
      log_print(avg_raw);
      log_print(", Erro: ");
      log_println(cal_err);
      delay(10);  // Cede 1ms para o RTOS (FreeRTOS)
      String json = "{\"type\":\"cal_result\", \"target_i\":" + String(g_calReadTarget_i) + ", \"raw\":" + String(avg_raw) + ", \"err\":" + String(cal_err) + "}";
      ws.textAll(json);
      delay(10);  // Cede 1ms para o RTOS (FreeRTOS)
      // Reseta as flags
      g_calReadChannel = -1;
      g_calReadTarget_i = -1;
      lcd.setBacklight(HIGH);
      delay(10);  // Cede 1ms para o RTOS (FreeRTOS)
    }
  }

  // 3. Cálculo de Regressão (Pedido pelo WebSocket)
  if (g_calCalculateRequest) {
    log_println("Calculando regressão...");
    double Sx = 0, Sxx = 0, Sxy = 0, Sy = 0, Syy = 0;
    int n = g_calCalc_size;

    const double RAW_TO_VOLT = (g_calCalc_ADCType == 0) ? 2.98023e-7 : 6.25e-5;

    double cal_x[257];

    for (int i = 0; i < n; i++) {
      cal_x[i] = (double)g_calCalc_x_raw[i] * RAW_TO_VOLT;
      Sx += cal_x[i];
      Sxx += cal_x[i] * cal_x[i];
      Sxy += cal_x[i] * g_calCalc_y[i];
      Sy += g_calCalc_y[i];
      Syy += g_calCalc_y[i] * g_calCalc_y[i];
    }

    double b = (n * Sxy - Sx * Sy) / (n * Sxx - Sx * Sx);
    double a = Sy / n - b * Sx / n;
    double eps = 0;
    if (n > 2) {
      eps = (1.0 / (n * (n - 2.0))) * (n * Syy - Sy * Sy - b * b * (n * Sxx - Sx * Sx));
    }
    double r_sq = ((n * Sxy - Sx * Sy) * (n * Sxy - Sx * Sy)) / ((n * Sxx - Sx * Sx) * (n * Syy - Sy * Sy));

    log_print("Regressão: A=");
    dtostrf(a, 10, 8, mtxt);
    log_print(mtxt);
    log_print(", B=");
    dtostrf(b, 10, 8, mtxt);
    log_print(mtxt);
    log_print(", R²=");
    dtostrf(r_sq, 8, 6, mtxt);
    log_println(mtxt);

    String json = "{\"type\":\"cal_calculate_result\", \"a\":" + String(a, 8) + ", \"b\":" + String(b, 8) + ", \"eps\":" + String(eps, 5) + ", \"r\":" + String(r_sq, 6) + "}";
    ws.textAll(json);

    g_calCalculateRequest = false;
  }
  // --- FIM: Lógica de Calibração ---


  //desenha menu
  if (t - tmenu >= 100) {
    draw_menu();
    tmenu = t;
    r8++;
    // --- ATUALIZAÇÃO DO WEBSOCKET AQUI ---
    // Envia o estado do LCD para a web, se tiver mudado
    lcd.sendUpdate();
    // ------------------------------------
  }
  // --- LÓGICA PERIÓDICA DE DUMP ---
  if (st.a_run && (t - t_dump_sd >= DUMP_SD_INTERVAL)) {
    // Periodicamente, move o cache Nível 2 (Flash) para o Nível 3 (SD Card)
    // Primeiro, garante que a RAM também foi descarregada
    dumpRamToFlash();
    dumpFlashToSD();
    t_dump_sd = t;
  }
  // ---------------------------------
  //desenha gfx
  if (dgfx < 1) {
    dgfx++;
  } else {
    dgfx = 0;
    // --- INÍCIO DA CORREÇÃO DE CONFLITO I2C ---
    // Se uma calibração (stream ou leitura de precisão) estiver em andamento,
    // NÓS PULAMOS a atualização das telas GFX (s1 e s2).
    // Isso impede que as chamadas I2C de alta frequência do u8g2
    // entrem em conflito com as chamadas I2C de alta frequência do readADC().
    if (g_calStreamChannel == -1 && g_calReadChannel == -1) {
      // picture loop
      if (cfg.GFX_0) {
        u8g2_t1.firstPage();
        do {
          draw_t1();
        } while (u8g2_t1.nextPage());
      }
      if (cfg.GFX_1) {
        u8g2_s1.firstPage();
        do {
          draw_s1();
        } while (u8g2_s1.nextPage());
      }
      if (cfg.GFX_2) {
        u8g2_s2.firstPage();
        do {
          draw_s2();
        } while (u8g2_s2.nextPage());
      }

      draw_state++;
    }
  }
  //debouce e controle dos botoes
  if (t - tbut >= 50) {
    checkb();
    tbut = t;
  }
}
