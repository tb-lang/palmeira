/**
  EST1 - TESTE OTA (ATUALIZAÇÃO VIA GITHUB)
  - MUDANÇA: NUMERO_DE_ENVIOS alterado para 3.
  - MUDANÇA: versaoAtual alterada para "2".
  - MANTIDO: Todos os Logs, Pinos, Pluviômetro e Calibração de Bateria.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ModbusMaster.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include "time.h"
#include <HTTPUpdate.h>

// ============================================================
// CONFIGURAÇÕES
// ============================================================

const char* versaoAtual = "2"; // MUDADO PARA 2 PARA O TESTE
const char* urlVersao = "https://raw.githubusercontent.com/tb-lang/palmeira/main/versao.txt";
const char* urlBinario = "https://raw.githubusercontent.com/tb-lang/palmeira/main/EST1.ino.bin";

const char* ssid = "AiAgro";
const char* password = "20132013";
const char* googleScriptURL = "https://script.google.com/macros/s/AKfycbyM2HiV4MG28u3jnTchZKAHPKNL0tl5GseR6HCqKyNl4bTYtnebhFUA37IrxEfvv4Z_/exec";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 14400; // Dubai UTC+4
const int   daylightOffset_sec = 0; 

#define NUMERO_DE_ENVIOS 3 // MUDADO PARA 3 PARA O TESTE
#define HORA_ENVIO_AGENDADO 9 

#define VEXT_PIN 0
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST -1 
#define DHTPIN 4
#define DHTTYPE DHT22
#define RS485_TX 17
#define RS485_RX 16
#define RS485_DE_RE 32
#define RELE_PIN 26
#define VOLTIMETRO_PIN 34 

// ===== Pluviometro =====
#define PLUVIOMETRO_PIN 25
const unsigned long DEBOUNCE_MS = 250;

RTC_DATA_ATTR uint32_t pluviometroPulsos = 0; 
volatile unsigned long ultimoPulsoMs = 0;
uint32_t pluviometroPulsosLidos = 0;

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
DHT dht(DHTPIN, DHTTYPE);
ModbusMaster node;

float temperaturaAr=0, umidadeAr=0, umidadeSolo=0, tempSolo=0, phSolo=0, condutividade=0;
int nitrogenio=0, fosforo=0, potassio=0;
float voltagemBateria = 0;

void IRAM_ATTR pluviometroISR() {
    unsigned long agora = millis();
    if (agora - ultimoPulsoMs > DEBOUNCE_MS) {
        pluviometroPulsos++;
        ultimoPulsoMs = agora;
        Serial.print("[PLUVIOMETRO] BATIDA #");
        Serial.println(pluviometroPulsos);
    }
}

// ============================================================
// FUNÇÃO DE ATUALIZAÇÃO (OTA)
// ============================================================

void executarOTA() {
    if (WiFi.status() != WL_CONNECTED) return;
    
    WiFiClientSecure client;
    client.setInsecure(); 
    
    HTTPClient http;
    Serial.print("[OTA] Verificando versao em: "); Serial.println(urlVersao);
    
    if (http.begin(client, urlVersao)) {
        int httpCode = http.GET();
        if (httpCode == 200) {
            String novaVersao = http.getString();
            novaVersao.trim();
            Serial.print("[OTA] Versao no servidor: "); Serial.println(novaVersao);
            
            if (novaVersao != versaoAtual) {
                mostrarStatus("ATUALIZANDO...");
                Serial.println("[OTA] Iniciando download do novo firmware...");
                
                t_httpUpdate_return ret = httpUpdate.update(client, urlBinario);
                
                switch (ret) {
                    case HTTP_UPDATE_FAILED:
                        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                        break;
                    case HTTP_UPDATE_NO_UPDATES:
                        Serial.println("HTTP_UPDATE_NO_UPDATES");
                        break;
                    case HTTP_UPDATE_OK:
                        Serial.println("HTTP_UPDATE_OK");
                        break;
                }
            } else {
                Serial.println("[OTA] Placa ja esta atualizada.");
            }
        }
        http.end();
    }
}

// ============================================================
// FUNÇÕES AUXILIARES
// ============================================================

void VextON() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(1000); 
}

void preTransmission() { digitalWrite(RS485_DE_RE, HIGH); }
void postTransmission() { digitalWrite(RS485_DE_RE, LOW); }

void mostrarStatus(String texto) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("EST1 (Dubai)");
    display.println("----------------");
    display.println(texto);
    display.display();
}

bool garantirWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WIFI] Ja conectado. RSSI: ");
        Serial.println(WiFi.RSSI());
        return true;
    }
    
    Serial.println("[WIFI] Tentando conectar...");
    mostrarStatus("Conectando WiFi...");
    WiFi.begin(ssid, password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500); Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WIFI] Conectado!");
        return true;
    } else {
        Serial.println("\n[WIFI] Falha na conexão.");
        return false;
    }
}

void lerSensores() {
    Serial.println("[SISTEMA] Iniciando leitura dos sensores...");
    analogSetAttenuation(ADC_11db);
    int leitura_adc_bruta = analogRead(VOLTIMETRO_PIN);
    voltagemBateria = (leitura_adc_bruta / 4095.0) * 3.3 * 5.197;
    Serial.print("[BATERIA] Voltagem: "); Serial.println(voltagemBateria);

    temperaturaAr = dht.readTemperature();
    umidadeAr = dht.readHumidity();
    if(isnan(temperaturaAr)) temperaturaAr = 0;
    if(isnan(umidadeAr)) umidadeAr = 0;

    Serial.println("[MODBUS] Lendo sensores de solo...");
    if (node.readHoldingRegisters(0x0012, 2) == node.ku8MBSuccess) {
        umidadeSolo = node.getResponseBuffer(0);
        tempSolo = node.getResponseBuffer(1) / 10.0;
    }
    if (node.readHoldingRegisters(0x0015, 1) == node.ku8MBSuccess) condutividade = node.getResponseBuffer(0);
    if (node.readHoldingRegisters(0x0007, 1) == node.ku8MBSuccess) phSolo = node.getResponseBuffer(0);
    if (node.readHoldingRegisters(0x001E, 3) == node.ku8MBSuccess) {
        nitrogenio = node.getResponseBuffer(0);
        fosforo = node.getResponseBuffer(1);
        potassio = node.getResponseBuffer(2);
    }

    noInterrupts();
    pluviometroPulsosLidos = pluviometroPulsos;
    interrupts();
    Serial.print("[SENSORES] Chuva acumulada: "); Serial.println(pluviometroPulsosLidos);
}

void enviarPacote(int indice) {
    StaticJsonDocument<1024> doc; 
    doc["origem"] = "EST1";
    JsonObject d = doc.createNestedObject("dados");
    
    d["temp_ar"] = temperaturaAr;
    d["umidade_ar"] = umidadeAr;
    d["temp_solo"] = tempSolo;
    d["umidade_solo"] = umidadeSolo;
    d["condutividade"] = condutividade;
    d["ph"] = phSolo;
    d["nitrogenio"] = nitrogenio;
    d["fosforo"] = fosforo;
    d["potassio"] = potassio;
    d["voltagem_bateria"] = voltagemBateria;

    if (indice == 0 && pluviometroPulsosLidos > 0) {
        d["viradas"] = pluviometroPulsosLidos;
        Serial.println("[SISTEMA] Incluindo dados do Pluviometro (Envio Único)");
    } else {
        Serial.println("[SISTEMA] Omitindo Pluviometro para evitar duplicidade.");
    }
    
    int rssiValue = WiFi.RSSI();
    int rssiPercent = 2 * (rssiValue + 100);
    if (rssiPercent > 100) rssiPercent = 100;
    if (rssiPercent < 0) rssiPercent = 0;
    d["rssi"] = rssiPercent;
    d["envio_num"] = indice + 1;

    String jsonStr;
    serializeJson(doc, jsonStr);
    Serial.println("[HTTP] JSON Gerado:");
    Serial.println(jsonStr);

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    if (http.begin(client, googleScriptURL)) {
        http.addHeader("Content-Type", "application/json");
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        int httpCode = http.POST(jsonStr);
        Serial.print("[HTTP] Codigo de Retorno: "); Serial.println(httpCode);
        
        if (httpCode == 200 || httpCode == 302) {
            Serial.println("[ENVIO] Sucesso!");
            mostrarStatus("Envio " + String(indice+1) + " OK!");
            
            if(indice == 0) {
                noInterrupts();
                pluviometroPulsos = 0; 
                pluviometroPulsosLidos = 0; 
                interrupts();
                Serial.println("[PLUVIOMETRO] Contador resetado após envio 1.");
            }
        }
        http.end();
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n--- EST1 INICIADO (VERSÃO 2) ---");
    
    VextON();
    pinMode(RELE_PIN, OUTPUT); 
    digitalWrite(RELE_PIN, HIGH);
    pinMode(RS485_DE_RE, OUTPUT); 
    postTransmission();

    Wire.begin(OLED_SDA, OLED_SCL);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextColor(WHITE);
    
    dht.begin();
    Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
    node.begin(1, Serial2);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    pinMode(PLUVIOMETRO_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PLUVIOMETRO_PIN), pluviometroISR, FALLING);

    esp_sleep_enable_ext0_wakeup((gpio_num_t)PLUVIOMETRO_PIN, 0); 
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    struct tm timeinfo;
    bool horaOk = false;

    if (garantirWiFi()) {
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        if(getLocalTime(&timeinfo)) {
            horaOk = true;
            Serial.print("[TEMPO] Hora atual: "); Serial.println(timeinfo.tm_hour);
            executarOTA(); // Verifica se há atualização no GitHub
        }
    }

    if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0 || (horaOk && timeinfo.tm_hour >= HORA_ENVIO_AGENDADO)) {
        Serial.println("[SISTEMA] Iniciando ciclo de 3 envios.");
        for (int i = 0; i < NUMERO_DE_ENVIOS; i++) {
            if (garantirWiFi()) {
                lerSensores();
                enviarPacote(i);
                if(i < NUMERO_DE_ENVIOS - 1) {
                    Serial.println("[SISTEMA] Aguardando 30s para o proximo envio...");
                    delay(30000); 
                }
            }
        }
    } else {
        Serial.println("[SISTEMA] Acordou por pulso de chuva. RTC preservado.");
    }

    long tempo_sono_segundos = 3600; 
    if (horaOk) {
        long segundos_hoje = (timeinfo.tm_hour * 3600) + (timeinfo.tm_min * 60) + timeinfo.tm_sec;
        long segundos_objetivo = HORA_ENVIO_AGENDADO * 3600;
        
        if (segundos_hoje < segundos_objetivo) {
            tempo_sono_segundos = segundos_objetivo - segundos_hoje;
        } else {
            tempo_sono_segundos = (86400 - segundos_hoje) + segundos_objetivo;
        }
    }

    Serial.print("[SISTEMA] Deep Sleep por: "); Serial.print(tempo_sono_segundos); Serial.println(" segundos.");
    
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    digitalWrite(VEXT_PIN, HIGH); 
    
    esp_sleep_enable_timer_wakeup((uint64_t)tempo_sono_segundos * 1000000ULL);
    esp_deep_sleep_start();
}

void loop() {}