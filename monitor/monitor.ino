#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ThingSpeak.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "../esp-idf-ssd1306/components/ssd1306/ssd1306.h"

// Configurações de rede WiFi e ThingSpeak
const char* ssid = ""; // Nome da rede WiFi
const char* password = "senha"; // Senha da rede WiFi
unsigned long myChannelNumber = 2679240; // Número do canal no ThingSpeak
const char* myWriteAPIKey = "G37S3ZXYX7HG5YVL"; // Chave de API para escrita no ThingSpeak

WiFiClient client;
SSD1306_t display;

#define I2C_ADDR_MAX30102 0x57
#define i2c_port 0
#define i2c_frequency 800000
#define i2c_gpio_sda 21
#define i2c_gpio_scl 22

int irpower = 0,
    rpower = 0,
    lirpower = 0,
    lrpower = 0,
    finger_on_sensor = 0,
    frame_id = 0;

float heartrate = 0.0,
      pctspo2 = 100.0,
      meastime;

/**
 * @brief Inicializa o sensor MAX30102 via I2C.
 */
void max30102_init() {
    i2c_init();
    uint8_t data = (0x2 << 5); // Média de amostras
    i2c_write(I2C_ADDR_MAX30102, 0x08, data);
    data = 0x03; // Modo de amostragem
    i2c_write(I2C_ADDR_MAX30102, 0x09, data);
    data = (0x3 << 5) + (0x3 << 2) + 0x3; // Taxa de amostragem
    i2c_write(I2C_ADDR_MAX30102, 0x0a, data);
    data = 0xd0; // Potência do IR
    i2c_write(I2C_ADDR_MAX30102, 0x0c, data);
    data = 0xa0; // Potência do LED vermelho
    i2c_write(I2C_ADDR_MAX30102, 0x0d, data);
}

/**
 * @brief Inicializa o display OLED.
 */
void display_init() {
    spi_master_init(&display, 23, 18, 5, 16, 17);
    ssd1306_init(&display, 128, 64);
    ssd1306_clear_screen(&display, false);
    ssd1306_contrast(&display, 0xff);
}

/**
 * @brief Tarefa para processar dados do sensor MAX30102.
 */
void max30102_task() {
    int cnt, samp, tcnt = 0;
    uint8_t rptr, wptr, data, regdata[256];
    float firxv[5], firyv[5], fredxv[5], fredyv[5];
    float lastmeastime = 0;
    float hrarray[10], spo2array[10];
    int hrarraycnt = 0;

    while (1) {
        if (lirpower != irpower) {
            data = (uint8_t) irpower;
            i2c_write(I2C_ADDR_MAX30102, 0x0c, data);
            lirpower = irpower;
        }
        if (lrpower != rpower) {
            data = (uint8_t) rpower;
            i2c_write(I2C_ADDR_MAX30102, 0x0d, data);
            lrpower = rpower;
        }

        i2c_read(I2C_ADDR_MAX30102, 0x04, &wptr, 1);
        i2c_read(I2C_ADDR_MAX30102, 0x06, &rptr, 1);
        samp = ((32 + wptr) - rptr) % 32;
        i2c_read(I2C_ADDR_MAX30102, 0x07, regdata, 6 * samp);

        for (cnt = 0; cnt < samp; cnt++) {
            meastime = 0.01 * tcnt++;

            // Filtros FIR para batimentos e oxigênio
            firxv[0] = firxv[1];
            firxv[1] = firxv[2];
            firxv[2] = firxv[3];
            firxv[3] = firxv[4];
            firxv[4] = (1 / 3.48311) *
                (256 * 256 * (regdata[6 * cnt + 0] % 4) + 256 * regdata[6 * cnt + 1] + regdata[6 * cnt + 2]);
            firyv[0] = firyv[1];
            firyv[1] = firyv[2];
            firyv[2] = firyv[3];
            firyv[3] = firyv[4];
            firyv[4] = (firxv[0] + firxv[4]) - 2 * firxv[2]
                + (-0.1718123813 * firyv[0]) + (0.3686645260 * firyv[1])
                + (-1.1718123813 * firyv[2]) + (1.9738037992 * firyv[3]);

            fredxv[0] = fredxv[1];
            fredxv[1] = fredxv[2];
            fredxv[2] = fredxv[3];
            fredxv[3] = fredxv[4];
            fredxv[4] = (1 / 3.48311) *
                (256 * 256 * (regdata[6 * cnt + 3] % 4) + 256 * regdata[6 * cnt + 4] + regdata[6 * cnt + 5]);
            fredyv[0] = fredyv[1];
            fredyv[1] = fredyv[2];
            fredyv[2] = fredyv[3];
            fredyv[3] = fredyv[4];
            fredyv[4] = (fredxv[0] + fredxv[4]) - 2 * fredxv[2]
                + (-0.1718123813 * fredyv[0]) + (0.3686645260 * fredyv[1])
                + (-1.1718123813 * fredyv[2]) + (1.9738037992 * fredyv[3]);

            if (-1.0 * firyv[4] >= 100 && -1.0 * firyv[2] > -1 * firyv[0] && -1.0 * firyv[2] > -1 * firyv[4]) {
                if (meastime - lastmeastime < 0.5)
                    continue;
                if (!(finger_on_sensor == 0 && meastime - lastmeastime > 2))
                    finger_on_sensor = 1;

                hrarray[hrarraycnt % 5] = 60 / (meastime - lastmeastime);
                spo2array[hrarraycnt % 5] = 110 - 25 * ((fredyv[4] / fredxv[4]) / (firyv[4] / firxv[4]));
                if (spo2array[hrarraycnt % 5] > 100)
                    spo2array[hrarraycnt % 5] = 99.9;

                lastmeastime = meastime;
                hrarraycnt++;
                heartrate = (hrarray[0] + hrarray[1] + hrarray[2] + hrarray[3] + hrarray[4]) / 5;
                if (heartrate < 40 || heartrate > 150)
                    heartrate = 0;

                pctspo2 = (spo2array[0] + spo2array[1] + spo2array[2] + spo2array[3] + spo2array[4]) / 5;
                if (pctspo2 < 50 || pctspo2 > 101)
                    pctspo2 = 0;
            }
            if (heartrate && lastmeastime + 1.8 < 0.01 * tcnt)
                finger_on_sensor = 0;
        }
    }
}

/**
 * @brief Função que envia dados ao ThingSpeak.
 */
void sendToThingSpeakTask(void *pvParameters) {
    while (1) {
        if (finger_on_sensor) {
            ThingSpeak.setField(1, heartrate);
            ThingSpeak.setField(2, pctspo2);
            int result = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
            if (result == 200) {
                Serial.println("Dados enviados para ThingSpeak com sucesso!");
            } else {
                Serial.println("Erro ao enviar dados para ThingSpeak. Código: " + String(result));
            }
        }
        vTaskDelay(20000 / portTICK_PERIOD_MS); // Delay de 20s para respeitar o limite do ThingSpeak
    }
}

/**
 * @brief Função para atualizar o display.
 */
void draw_data() {
    int rate;
    while (1) {
        rate = (int)heartrate;
        if (finger_on_sensor && rate)
            frame_id = draw_bpm(rate); // Exibe a taxa de batimento no display
        else
            frame_id = draw_finger_req(); // Exibe a mensagem para colocar o dedo no sensor
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Atualiza o display a cada segundo
    }
}

void setup() {
    Serial.begin(9600);
    delay(10);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConectado à rede WiFi");
    ThingSpeak.begin(client);

    max30102_init();
    display_init();

    // Cria as tarefas FreeRTOS para o sensor, display e envio para ThingSpeak
    xTaskCreate(max30102_task, "max30102_task", 4096, NULL, 5, NULL);
    xTaskCreate(draw_data, "draw_data_task", 4096, NULL, 5, NULL);
    xTaskCreate(sendToThingSpeakTask, "sendToThingSpeakTask", 4096, NULL, 5, NULL);
}

void loop() {
    // O loop principal fica vazio, pois todas as tarefas são gerenciadas pelo FreeRTOS
}