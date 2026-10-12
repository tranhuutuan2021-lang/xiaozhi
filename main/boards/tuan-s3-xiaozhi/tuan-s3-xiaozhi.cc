#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/spi_common.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <driver/i2c_master.h>
#include "vl6180x.h"
#include <esp_timer.h>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string>
#include <cstring>
#include <cstdio>
#include <mutex>
#include <cerrno>

#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <unistd.h>
#include <cJSON.h>

#define TAG "TuanS3Xiaozhi"

// ============================================================================
// ========================== BOARD ===========================================
// ============================================================================

class TuanS3XiaozhiBoard : public WifiBoard {
private:
    Button boot_button_;

    Display* display_ = nullptr;

    // ========================================================================
    // C6 STATUS CACHE
    // ========================================================================

    bool c6_online_ = false;

    bool motion_detected_ = false;

    bool light_on_ = false;

    bool zigbee_ready_ = false;

    bool motion_device_connected_ = false;

    bool light_device_connected_ = false;

    bool climate_device_connected_ = false;

    float temperature_ = 0.0f;

    float humidity_ = 0.0f;

    bool temperature_valid_ = false;

    bool humidity_valid_ = false;

    bool climate_data_fresh_ = false;
    uint32_t temperature_age_ms_ = 0xFFFFFFFFUL;
    uint32_t humidity_age_ms_ = 0xFFFFFFFFUL;

    // ========================================================================
    // ROBOTANNO TCP
    // PC RobotAnno Gateway: 192.168.80.145
    // C# Robot Gateway: TCP port 6000
    // ========================================================================

    static constexpr const char* ROBOT_SERVER_IP =
        "192.168.80.145";

    static constexpr uint16_t ROBOT_SERVER_PORT =
        6000;

    bool robot_control_mode_ = false;

    std::mutex robot_tcp_mutex_;

    // ========================================================================
    // SMART HOME DEVKIT TCP
    // XiaoZhi S3 -> Wi-Fi/TCP -> ESP32 DevKit :6001 -> UART -> ESP32-C6
    //
    // QUAN TRỌNG: thay DEVKIT_SERVER_IP bằng IP DevKit in trên Serial Monitor.
    // ========================================================================

    static constexpr const char* DEVKIT_SERVER_IP =
        "192.168.80.89";

    static constexpr uint16_t DEVKIT_SERVER_PORT =
        6001;

    std::mutex devkit_tcp_mutex_;

    TaskHandle_t device_event_task_ = nullptr;

    // ========================================================================
    // GET WATER FLOW
    // Một MCP duy nhất sẽ điều phối:
    // RobotAnno GET_WATER -> đợi READY_FOR_PUMP -> DevKit WATER_PUMP_5S
    // -> đợi bơm DONE -> báo C# WATER_DONE.
    // ========================================================================

    std::mutex water_flow_mutex_;
    bool water_flow_running_ = false;

    // ========================================================================
    // SPI / LCD
    // ========================================================================

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};

        buscfg.mosi_io_num =
            DISPLAY_SPI_MOSI_PIN;

        buscfg.miso_io_num =
            GPIO_NUM_NC;

        buscfg.sclk_io_num =
            DISPLAY_SPI_SCK_PIN;

        buscfg.quadwp_io_num =
            GPIO_NUM_NC;

        buscfg.quadhd_io_num =
            GPIO_NUM_NC;

        buscfg.max_transfer_sz =
            DISPLAY_WIDTH
            *
            DISPLAY_HEIGHT
            *
            sizeof(uint16_t);

        ESP_ERROR_CHECK(
            spi_bus_initialize(
                DISPLAY_SPI_HOST,
                &buscfg,
                SPI_DMA_CH_AUTO
            )
        );
    }

    // ========================================================================
    // DISPLAY
    // ========================================================================

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io =
            nullptr;

        esp_lcd_panel_handle_t panel =
            nullptr;

        esp_lcd_panel_io_spi_config_t io_config =
            {};

        io_config.cs_gpio_num =
            DISPLAY_CS_PIN;

        io_config.dc_gpio_num =
            DISPLAY_DC_PIN;

        io_config.spi_mode =
            DISPLAY_SPI_MODE;

        io_config.pclk_hz =
            DISPLAY_PCLK_HZ;

        io_config.trans_queue_depth =
            10;

        io_config.lcd_cmd_bits =
            8;

        io_config.lcd_param_bits =
            8;

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_io_spi(
                DISPLAY_SPI_HOST,
                &io_config,
                &panel_io
            )
        );

        esp_lcd_panel_dev_config_t panel_config =
            {};

        panel_config.reset_gpio_num =
            DISPLAY_RST_PIN;

        panel_config.rgb_ele_order =
            LCD_RGB_ELEMENT_ORDER_RGB;

        panel_config.bits_per_pixel =
            16;

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_st7789(
                panel_io,
                &panel_config,
                &panel
            )
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_reset(
                panel
            )
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_init(
                panel
            )
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_invert_color(
                panel,
                DISPLAY_INVERT_COLOR
            )
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_swap_xy(
                panel,
                DISPLAY_SWAP_XY
            )
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_mirror(
                panel,
                DISPLAY_MIRROR_X,
                DISPLAY_MIRROR_Y
            )
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_disp_on_off(
                panel,
                true
            )
        );

        display_ =
            new SpiLcdDisplay(
                panel_io,
                panel,
                DISPLAY_WIDTH,
                DISPLAY_HEIGHT,
                DISPLAY_OFFSET_X,
                DISPLAY_OFFSET_Y,
                DISPLAY_MIRROR_X,
                DISPLAY_MIRROR_Y,
                DISPLAY_SWAP_XY
            );

        ESP_LOGI(
            TAG,
            "ST7789 initialized: 240x240"
        );
    }

    // ========================================================================
    // BUTTON
    // ========================================================================

    void InitializeButtons() {
        boot_button_.OnClick(
            [this]() {
                auto& app =
                    Application::GetInstance();

                if (
                    app.GetDeviceState()
                    ==
                    kDeviceStateStarting
                )
                {
                    EnterWifiConfigMode();

                    return;
                }

                app.ToggleChatState();
            }
        );
    }

    // ========================================================================
    // MOBILE BASE: DRV8833 + 2x N20 + VL6180X
    // Chân GPIO lấy từ config.h
    // ========================================================================

    // DRV8833 - lấy chân trực tiếp từ config.h
    // Motor A/trái: IN1=GPIO12, IN2=GPIO13
    // Motor B/phải: IN3=GPIO14, IN4=GPIO21
    static constexpr gpio_num_t ML1 = MOTOR_A_IN1_GPIO;
    static constexpr gpio_num_t ML2 = MOTOR_A_IN2_GPIO;
    static constexpr gpio_num_t MR1 = MOTOR_B_IN3_GPIO;
    static constexpr gpio_num_t MR2 = MOTOR_B_IN4_GPIO;

    static constexpr gpio_num_t TOF_SDA = TOF_I2C_SDA_GPIO;
    static constexpr gpio_num_t TOF_SCL = TOF_I2C_SCL_GPIO;
    static constexpr uint8_t TOF_ADDR = VL6180X_I2C_ADDRESS;

    static constexpr int OBSTACLE_MM = OBSTACLE_STOP_DISTANCE_MM;

    std::mutex car_mutex_;
    int car_speed_ = 45;
    int left_trim_ = 100;
    int right_trim_ = 100;
    int car_left_dir_ = 0;
    int car_right_dir_ = 0;
    bool car_forward_ = false;
    bool tof_ready_ = false;
    bool motor_ready_ = false;
    i2c_master_bus_handle_t tof_bus_handle_ = nullptr;
    vl6180x_handle_t tof_sensor_ = nullptr;

    uint32_t CarPwm(int percent, int trim) {
        percent = std::max(0, std::min(100, percent));
        trim = std::max(0, std::min(150, trim));

        uint32_t duty =
            (static_cast<uint32_t>(percent) *
             static_cast<uint32_t>(MOTOR_PWM_MAX_DUTY) *
             static_cast<uint32_t>(trim)) / 10000U;

        if (duty > static_cast<uint32_t>(MOTOR_PWM_MAX_DUTY)) {
            duty = static_cast<uint32_t>(MOTOR_PWM_MAX_DUTY);
        }

        return duty;
    }

    void SetPwm(ledc_channel_t channel, uint32_t duty) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }

    void MotorRaw(int left_dir, int right_dir, int percent) {
        if (!motor_ready_) {
            return;
        }

        const uint32_t lpwm = CarPwm(percent, left_trim_);
        const uint32_t rpwm = CarPwm(percent, right_trim_);

        // Polarity theo đúng lắp motor thực tế:
        // left_dir > 0  : GPIO12=0,   GPIO13=PWM  (trái tiến)
        // left_dir < 0  : GPIO12=PWM, GPIO13=0    (trái lùi)
        // right_dir > 0 : GPIO14=PWM, GPIO21=0    (phải tiến)
        // right_dir < 0 : GPIO14=0,   GPIO21=PWM  (phải lùi)
        SetPwm(LEDC_CHANNEL_0, left_dir < 0 ? lpwm : 0);
        SetPwm(LEDC_CHANNEL_1, left_dir > 0 ? lpwm : 0);
        SetPwm(LEDC_CHANNEL_2, right_dir > 0 ? rpwm : 0);
        SetPwm(LEDC_CHANNEL_3, right_dir < 0 ? rpwm : 0);
    }

    void MotorRamp(int left_dir, int right_dir, int target_percent) {
        target_percent = std::max(0, std::min(100, target_percent));

        if (target_percent == 0) {
            MotorRaw(0, 0, 0);
            car_left_dir_ = 0;
            car_right_dir_ = 0;
            car_forward_ = false;
            return;
        }

        car_left_dir_ = left_dir;
        car_right_dir_ = right_dir;

        for (int step = 1; step <= 8; ++step) {
            MotorRaw(left_dir, right_dir, (target_percent * step) / 8);
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    }

    // VL6180X: dung component nestorit/vl6180x da test OK tren ESP-IDF.
    // Khoi tao rieng va that som, truoc LCD/audio/network cua XiaoZhi.
    void InitializeTof() {
        tof_ready_ = false;
        tof_sensor_ = nullptr;

        i2c_master_bus_config_t bus_config = {};
        bus_config.i2c_port = TOF_I2C_PORT;
        bus_config.sda_io_num = TOF_SDA;
        bus_config.scl_io_num = TOF_SCL;
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.flags.enable_internal_pullup = true;

        ESP_LOGI(TAG, "EARLY VL6180X: SDA=%d SCL=%d ADDR=0x%02X",
                 static_cast<int>(TOF_SDA),
                 static_cast<int>(TOF_SCL),
                 TOF_ADDR);

        esp_err_t err = i2c_new_master_bus(&bus_config, &tof_bus_handle_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "VL6180X I2C bus create failed: %s", esp_err_to_name(err));
            tof_bus_handle_ = nullptr;
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        err = i2c_master_probe(tof_bus_handle_, TOF_ADDR, 1000);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "EARLY VL6180X probe 0x%02X failed: %s",
                     TOF_ADDR, esp_err_to_name(err));
            return;
        }

        ESP_LOGI(TAG, "EARLY VL6180X FOUND at 0x%02X", TOF_ADDR);

        err = vl6180x_create(tof_bus_handle_, TOF_ADDR, nullptr, &tof_sensor_);
        if (err != ESP_OK || tof_sensor_ == nullptr) {
            ESP_LOGW(TAG, "vl6180x_create failed: %s", esp_err_to_name(err));
            tof_sensor_ = nullptr;
            return;
        }

        uint8_t first_distance = 0;
        err = vl6180x_range_read(tof_sensor_, &first_distance);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "VL6180X first read failed: %s", esp_err_to_name(err));
            return;
        }

        tof_ready_ = true;
        ESP_LOGI(TAG, "VL6180X READY - first distance=%u mm",
                 static_cast<unsigned>(first_distance));
    }

    int DistanceMm() {
        if (!tof_ready_ || tof_sensor_ == nullptr) {
            return -1;
        }

        uint8_t distance = 0;
        esp_err_t err = vl6180x_range_read(tof_sensor_, &distance);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "VL6180X range read failed: %s", esp_err_to_name(err));
            return -1;
        }

        return static_cast<int>(distance);
    }

    void CarStopLocked() {
        MotorRaw(0, 0, 0);
        car_left_dir_ = 0;
        car_right_dir_ = 0;
        car_forward_ = false;
        ESP_LOGI(TAG, "CAR STOP");
    }

    bool CarMoveLocked(int left_dir, int right_dir, bool forward) {
        if (!motor_ready_) {
            ESP_LOGW(TAG, "DRV8833 motor PWM is not ready");
            return false;
        }

        if (forward && tof_ready_) {
            const int mm = DistanceMm();

            if (mm >= 0 && mm <= OBSTACLE_MM) {
                ESP_LOGW(TAG, "Obstacle %d mm - forward blocked", mm);
                CarStopLocked();
                return false;
            }
        }

        MotorRamp(left_dir, right_dir, car_speed_);
        car_forward_ = forward;
        return true;
    }

    static void CarSafetyTaskEntry(void* arg) {
        static_cast<TuanS3XiaozhiBoard*>(arg)->CarSafetyTask();
    }

    void CarSafetyTask() {
        while (true) {
            {
                std::lock_guard<std::mutex> lock(
                    car_mutex_
                );

                // Không có timeout tự dừng.
                // Xe chỉ tự dừng khi đang tiến và VL6180X phát hiện vật cản.
                if (car_forward_ && tof_ready_) {
                    const int mm =
                        DistanceMm();

                    if (
                        mm >= 0
                        &&
                        mm <= OBSTACLE_MM
                    ) {
                        ESP_LOGW(
                            TAG,
                            "Obstacle %d mm -> STOP",
                            mm
                        );

                        CarStopLocked();
                    }
                }
            }

            vTaskDelay(
                pdMS_TO_TICKS(80)
            );
        }
    }

    void InitializeCar() {
        ledc_timer_config_t timer = {};
        timer.speed_mode = LEDC_LOW_SPEED_MODE;
        timer.duty_resolution = MOTOR_PWM_RESOLUTION;
        timer.timer_num = LEDC_TIMER_0;
        timer.freq_hz = MOTOR_PWM_FREQ;
        timer.clk_cfg = LEDC_AUTO_CLK;

        esp_err_t motor_err = ledc_timer_config(&timer);

        const gpio_num_t pins[4] = {
            ML1,
            ML2,
            MR1,
            MR2
        };

        const ledc_channel_t channels[4] = {
            LEDC_CHANNEL_0,
            LEDC_CHANNEL_1,
            LEDC_CHANNEL_2,
            LEDC_CHANNEL_3
        };

        if (motor_err == ESP_OK) {
            for (int i = 0; i < 4; ++i) {
                ledc_channel_config_t channel = {};
                channel.gpio_num = pins[i];
                channel.speed_mode = LEDC_LOW_SPEED_MODE;
                channel.channel = channels[i];
                channel.timer_sel = LEDC_TIMER_0;
                channel.duty = 0;
                channel.hpoint = 0;

                motor_err = ledc_channel_config(&channel);
                if (motor_err != ESP_OK) {
                    ESP_LOGE(
                        TAG,
                        "DRV8833 PWM channel %d init failed: %s",
                        i,
                        esp_err_to_name(motor_err)
                    );
                    break;
                }
            }
        }

        motor_ready_ = (motor_err == ESP_OK);

        if (motor_ready_) {
            ESP_LOGI(
                TAG,
                "DRV8833 READY: L=%d/%d R=%d/%d PWM=%dHz max=%d",
                static_cast<int>(ML1),
                static_cast<int>(ML2),
                static_cast<int>(MR1),
                static_cast<int>(MR2),
                MOTOR_PWM_FREQ,
                MOTOR_PWM_MAX_DUTY
            );
            MotorRaw(0, 0, 0);
        } else {
            ESP_LOGE(
                TAG,
                "DRV8833 PWM init failed: %s. Sensor/Zigbee/RobotAnno continue.",
                esp_err_to_name(motor_err)
            );
        }

        // VL6180X da duoc khoi tao som trong constructor.
        // Motor va MCP van hoat dong neu cam bien loi.

        xTaskCreate(
            CarSafetyTaskEntry,
            "car_safety",
            4096,
            this,
            4,
            nullptr
        );

        ESP_LOGI(
            TAG,
            "Car initialized: Motor=%s speed=%d%% ToF=%s",
            motor_ready_ ? "READY" : "ERROR",
            car_speed_,
            tof_ready_ ? "READY" : "NOT_FOUND"
        );
    }

    // ========================================================================
    // SMART HOME COMMAND -> DEVKIT TCP -> C6 UART
    // ========================================================================

    bool C6SendCommand(
        const std::string& command,
        std::string* command_response = nullptr
    ) {
        if (command_response != nullptr)
        {
            command_response->clear();
        }

        if (command.empty())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(
            devkit_tcp_mutex_
        );

        int sock =
            socket(
                AF_INET,
                SOCK_STREAM,
                IPPROTO_IP
            );

        if (sock < 0)
        {
            ESP_LOGE(
                TAG,
                "DevKit socket create failed"
            );

            return false;
        }

        struct timeval timeout = {};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        setsockopt(
            sock,
            SOL_SOCKET,
            SO_RCVTIMEO,
            &timeout,
            sizeof(timeout)
        );

        setsockopt(
            sock,
            SOL_SOCKET,
            SO_SNDTIMEO,
            &timeout,
            sizeof(timeout)
        );

        sockaddr_in dest = {};
        dest.sin_family = AF_INET;
        dest.sin_port =
            htons(
                DEVKIT_SERVER_PORT
            );

        if (
            inet_pton(
                AF_INET,
                DEVKIT_SERVER_IP,
                &dest.sin_addr
            )
            !=
            1
        )
        {
            ESP_LOGE(
                TAG,
                "DEVKIT_SERVER_IP invalid: %s",
                DEVKIT_SERVER_IP
            );

            close(sock);
            return false;
        }

        ESP_LOGI(
            TAG,
            "SmartHome TCP -> %s:%u",
            DEVKIT_SERVER_IP,
            DEVKIT_SERVER_PORT
        );

        if (
            connect(
                sock,
                reinterpret_cast<sockaddr*>(
                    &dest
                ),
                sizeof(dest)
            )
            !=
            0
        )
        {
            ESP_LOGE(
                TAG,
                "Cannot connect DevKit: errno=%d",
                errno
            );

            close(sock);
            return false;
        }

        std::string packet =
            command + "\n";

        int sent =
            send(
                sock,
                packet.data(),
                packet.size(),
                0
            );

        if (
            sent !=
            static_cast<int>(
                packet.size()
            )
        )
        {
            ESP_LOGE(
                TAG,
                "DevKit TCP send failed: %s",
                command.c_str()
            );

            close(sock);
            return false;
        }

        ESP_LOGI(
            TAG,
            "TCP -> DevKit -> C6: %s",
            command.c_str()
        );

        // Đọc các dòng DevKit/C6 trả về trong cửa sổ timeout.
        // DevKit có thể gửi DEVKIT_ONLINE trước, sau đó là PONG/LIGHT/TEMP/...
        char rx[256];
        std::string line;
        bool direct_response_received = false;

        while (true)
        {
            int len =
                recv(
                    sock,
                    rx,
                    sizeof(rx),
                    0
                );

            if (len <= 0)
            {
                break;
            }

            for (
                int i = 0;
                i < len;
                ++i
            )
            {
                char c = rx[i];

                if (c == '\r')
                {
                    continue;
                }

                if (c == '\n')
                {
                    if (!line.empty())
                    {
                        const bool quiet_empty_event =
                            command == "GET_EVENT"
                            &&
                            line == "EVENT:NONE";

                        if (!quiet_empty_event)
                        {
                            ESP_LOGI(
                                TAG,
                                "TCP <- DevKit: %s",
                                line.c_str()
                            );
                        }

                        if (
                            line !=
                            "DEVKIT_ONLINE"
                        )
                        {
                            // Với lệnh bơm, S3 cần lấy ACK/STATUS trực tiếp.
                            // Các lệnh Smart Home cũ vẫn tiếp tục được parse như trước.
                            if (
                                command_response != nullptr
                                &&
                                command_response->empty()
                            )
                            {
                                *command_response = line;
                                direct_response_received = true;
                            }

                            ProcessC6Message(
                                line
                            );
                        }

                        line.clear();
                    }

                    continue;
                }

                if (line.size() < 256)
                {
                    line.push_back(c);
                }
                else
                {
                    line.clear();
                }
            }

            // ACK/STATUS đã nhận thì trả về ngay, không đợi recv timeout 1 giây.
            if (direct_response_received)
            {
                break;
            }
        }

        close(sock);
        return true;
    }

    // ========================================================================
    // PARSE FLOAT
    // ========================================================================

    float ParseFloat(
        const char* value
    ) {
        if (
            value == nullptr
        )
        {
            return 0.0f;
        }

        return strtof(
            value,
            nullptr
        );
    }

    // ========================================================================
    // PROCESS MESSAGE FROM C6
    // ========================================================================

    void ProcessC6Message(
        const std::string& message
    ) {
        if (
            message.empty()
            ||
            message == "EVENT:NONE"
        )
        {
            return;
        }

        ESP_LOGI(
            TAG,
            "UART <- C6: %s",
            message.c_str()
        );

        // --------------------------------------------------------------------
        // DEVKIT STATUS SNAPSHOT
        // STATUS:C6=1,LIGHT=0,MOTION=0,PUMP=0,
        //        CLIMATE_FRESH=1,TEMP=27.97,TEMP_AGE_MS=1234,
        //        HUMIDITY=69.50,HUMIDITY_AGE_MS=1234
        // --------------------------------------------------------------------

        if (
            message.rfind(
                "STATUS:",
                0
            ) == 0
        )
        {
            auto getField =
                [&message](
                    const char* key,
                    std::string& value
                ) -> bool {
                    const std::string token =
                        std::string(key) + "=";

                    size_t start =
                        message.find(token);

                    if (
                        start == std::string::npos
                    )
                    {
                        return false;
                    }

                    start += token.size();

                    size_t end =
                        message.find(',', start);

                    value =
                        message.substr(
                            start,
                            end == std::string::npos
                                ? std::string::npos
                                : end - start
                        );

                    return true;
                };

            std::string value;

            if (getField("C6", value))
            {
                c6_online_ =
                    value == "1";
            }

            if (getField("LIGHT", value))
            {
                light_on_ =
                    value == "1";
            }

            if (getField("MOTION", value))
            {
                motion_detected_ =
                    value == "1";
            }

            if (getField("TEMP", value))
            {
                temperature_ =
                    ParseFloat(
                        value.c_str()
                    );

                temperature_valid_ =
                    true;
            }

            if (getField("HUMIDITY", value))
            {
                humidity_ =
                    ParseFloat(
                        value.c_str()
                    );

                humidity_valid_ =
                    true;
            }

            if (getField("TEMP_AGE_MS", value))
            {
                temperature_age_ms_ =
                    static_cast<uint32_t>(
                        strtoul(
                            value.c_str(),
                            nullptr,
                            10
                        )
                    );
            }

            if (getField("HUMIDITY_AGE_MS", value))
            {
                humidity_age_ms_ =
                    static_cast<uint32_t>(
                        strtoul(
                            value.c_str(),
                            nullptr,
                            10
                        )
                    );
            }

            if (getField("CLIMATE_FRESH", value))
            {
                climate_data_fresh_ =
                    value == "1";

                // Đây là "recently responding", không phải Zigbee leave/join tuyệt đối.
                climate_device_connected_ =
                    climate_data_fresh_;
            }
            else
            {
                // Tương thích DevKit cũ: có TEMP/HUM thì ít nhất có dữ liệu cache.
                climate_data_fresh_ =
                    temperature_valid_
                    ||
                    humidity_valid_;
            }

            ESP_LOGI(
                TAG,
                "STATUS parsed: C6=%d LIGHT=%d MOTION=%d "
                "TEMP=%.2f(valid=%d age=%lu) "
                "HUM=%.2f(valid=%d age=%lu) FRESH=%d",
                c6_online_,
                light_on_,
                motion_detected_,
                temperature_,
                temperature_valid_,
                static_cast<unsigned long>(
                    temperature_age_ms_
                ),
                humidity_,
                humidity_valid_,
                static_cast<unsigned long>(
                    humidity_age_ms_
                ),
                climate_data_fresh_
            );

            return;
        }

        // --------------------------------------------------------------------
        // C6 ONLINE
        // --------------------------------------------------------------------

        if (
            message == "C6_ONLINE"
        )
        {
            c6_online_ =
                true;

            ESP_LOGI(
                TAG,
                "ESP32-C6 ONLINE"
            );

            return;
        }

        // --------------------------------------------------------------------
        // MOTION
        // --------------------------------------------------------------------

        if (
    message == "MOTION:1"
)
{
    // Chỉ báo khi chuyển từ không có motion -> có motion
    // để tránh report lặp và báo liên tục.
    bool first_motion =
        !motion_detected_;

    motion_detected_ =
        true;

    ESP_LOGW(
        TAG,
        "MOTION DETECTED"
    );

    if (
        first_motion
    )
    {
        auto& app =
            Application::GetInstance();

        app.Schedule(
            []() {
                auto& app =
                    Application::GetInstance();

                app.Alert(
                    "Cảnh báo",
                    "Có người vào",
                    "warning"
                );
            }
        );
    }

    return;
}
        // --------------------------------------------------------------------
        // LIGHT
        // --------------------------------------------------------------------

        if (
            message == "LIGHT:1"
        )
        {
            light_on_ =
                true;

            ESP_LOGI(
                TAG,
                "LIGHT ON"
            );

            return;
        }

        if (
            message == "LIGHT:0"
        )
        {
            light_on_ =
                false;

            ESP_LOGI(
                TAG,
                "LIGHT OFF"
            );

            return;
        }

        // --------------------------------------------------------------------
        // TEMPERATURE
        // --------------------------------------------------------------------

        if (
            message.rfind(
                "TEMP:",
                0
            )
            ==
            0
        )
        {
            temperature_ =
                ParseFloat(
                    message.c_str()
                    +
                    strlen("TEMP:")
                );

            temperature_valid_ =
                true;

            ESP_LOGI(
                TAG,
                "Temperature: %.2f C",
                temperature_
            );

            return;
        }

        // --------------------------------------------------------------------
        // HUMIDITY
        // --------------------------------------------------------------------

        if (
            message.rfind(
                "HUMIDITY:",
                0
            )
            ==
            0
        )
        {
            humidity_ =
                ParseFloat(
                    message.c_str()
                    +
                    strlen("HUMIDITY:")
                );

            humidity_valid_ =
                true;

            ESP_LOGI(
                TAG,
                "Humidity: %.2f %%",
                humidity_
            );

            return;
        }

        // --------------------------------------------------------------------
        // ZIGBEE READY
        // --------------------------------------------------------------------

        if (
            message == "ZIGBEE_READY:1"
        )
        {
            zigbee_ready_ =
                true;

            return;
        }

        if (
            message == "ZIGBEE_READY:0"
        )
        {
            zigbee_ready_ =
                false;

            return;
        }

        // --------------------------------------------------------------------
        // DEVICE STATES
        // --------------------------------------------------------------------

        if (
            message == "MOTION_DEVICE:1"
        )
        {
            motion_device_connected_ =
                true;

            return;
        }

        if (
            message == "MOTION_DEVICE:0"
        )
        {
            motion_device_connected_ =
                false;

            return;
        }

        if (
            message == "LIGHT_DEVICE:1"
        )
        {
            light_device_connected_ =
                true;

            return;
        }

        if (
            message == "LIGHT_DEVICE:0"
        )
        {
            light_device_connected_ =
                false;

            return;
        }

        if (
            message == "CLIMATE_DEVICE:1"
        )
        {
            climate_device_connected_ =
                true;

            return;
        }

        if (
            message == "CLIMATE_DEVICE:0"
        )
        {
            climate_device_connected_ =
                false;

            return;
        }

        // --------------------------------------------------------------------
        // PERMIT JOIN
        // --------------------------------------------------------------------

        if (
            message.rfind(
                "ZIGBEE_JOIN_OPEN:",
                0
            )
            ==
            0
        )
        {
            ESP_LOGI(
                TAG,
                "Zigbee permit join opened"
            );

            return;
        }

        if (
            message == "ZIGBEE_JOIN_CLOSED"
        )
        {
            ESP_LOGI(
                TAG,
                "Zigbee permit join closed"
            );

            return;
        }

        // --------------------------------------------------------------------
        // DEVICE JOINED
        // --------------------------------------------------------------------

        if (
            message == "ZIGBEE_DEVICE_JOINED"
        )
        {
            ESP_LOGI(
                TAG,
                "New Zigbee device joined (type unknown)"
            );

            Application::GetInstance().Schedule(
                [this]() {
                    SpeakAfterWaterFlow(
                        "Đã có thiết bị Zigbee mới tham gia hệ thống."
                    );
                }
            );

            return;
        }

        if (
            message.rfind(
                "ZIGBEE_DEVICE_JOINED:",
                0
            ) == 0
        )
        {
            const std::string prefix =
                "ZIGBEE_DEVICE_JOINED:";

            std::string payload =
                message.substr(prefix.size());

            std::string device_type;
            std::string model;

            const size_t separator =
                payload.find(':');

            if (separator == std::string::npos)
            {
                device_type = payload;
            }
            else
            {
                device_type = payload.substr(0, separator);
                model = payload.substr(separator + 1);
            }

            ESP_LOGI(
                TAG,
                "New Zigbee device: type=%s model=%s",
                device_type.c_str(),
                model.empty() ? "unknown" : model.c_str()
            );

            std::string spoken_text;

            if (device_type == "LIGHT")
            {
                spoken_text =
                    "Đèn Zigbee mới đã tham gia hệ thống.";
            }
            else if (device_type == "MOTION")
            {
                spoken_text =
                    "Cảm biến chuyển động Zigbee mới đã tham gia hệ thống.";
            }
            else if (device_type == "CLIMATE")
            {
                spoken_text =
                    "Cảm biến nhiệt độ và độ ẩm Zigbee mới đã tham gia hệ thống.";
            }
            else
            {
                spoken_text =
                    "Đã có thiết bị Zigbee mới tham gia hệ thống.";
            }

            Application::GetInstance().Schedule(
                [this, spoken_text]() {
                    SpeakAfterWaterFlow(
                        spoken_text.c_str()
                    );
                }
            );

            return;
        }
// --------------------------------------------------------------------
        // ERROR
        // --------------------------------------------------------------------

        if (
            message.rfind(
                "ERROR:",
                0
            )
            ==
            0
        )
        {
            ESP_LOGE(
                TAG,
                "C6 error: %s",
                message.c_str()
            );

            return;
        }
    }

    // ========================================================================
    // ROBOTANNO TCP HELPER
    //
    // Mỗi MCP command:
    //   S3 -> connect TCP -> nhận ROBOT_ANNO_READY
    //      -> gửi 1 dòng command -> nhận 1 dòng response -> close
    //
    // C# gateway dùng StreamReader.ReadLine(), vì vậy command phải kết thúc \n.
    // ========================================================================


    // ========================================================================
    // ASYNC ZIGBEE DEVICE EVENT POLLING
    // ========================================================================

    static void DeviceEventTaskEntry(void* arg)
    {
        static_cast<TuanS3XiaozhiBoard*>(arg)->DeviceEventTask();
    }

    void DeviceEventTask()
    {
        // Đợi Wi-Fi/lwIP khởi động.
        vTaskDelay(
            pdMS_TO_TICKS(5000)
        );

        while (true)
        {
            std::string event;

            C6SendCommand(
                "GET_EVENT",
                &event
            );

            // C6SendCommand tự gọi ProcessC6Message(event).
            vTaskDelay(
                pdMS_TO_TICKS(2000)
            );
        }
    }

    void StartDeviceEventTask()
    {
        if (device_event_task_ != nullptr)
        {
            return;
        }

        const BaseType_t result =
            xTaskCreate(
                DeviceEventTaskEntry,
                "zigbee_event",
                4096,
                this,
                4,
                &device_event_task_
            );

        if (result != pdPASS)
        {
            device_event_task_ = nullptr;
            ESP_LOGE(
                TAG,
                "Cannot create Zigbee event task"
            );
        }
        else
        {
            ESP_LOGI(
                TAG,
                "Zigbee event polling task started"
            );
        }
    }

    bool RobotSendCommand(
        const std::string& command,
        std::string& response
    ) {
        std::lock_guard<std::mutex> lock(
            robot_tcp_mutex_
        );

        response.clear();

        if (
            command.empty()
        )
        {
            return false;
        }

        int sock =
            socket(
                AF_INET,
                SOCK_STREAM,
                IPPROTO_IP
            );

        if (
            sock < 0
        )
        {
            ESP_LOGE(
                TAG,
                "Robot socket create failed, errno=%d",
                errno
            );

            return false;
        }

        struct timeval timeout = {};

        timeout.tv_sec = 3;
        timeout.tv_usec = 0;

        setsockopt(
            sock,
            SOL_SOCKET,
            SO_RCVTIMEO,
            &timeout,
            sizeof(timeout)
        );

        setsockopt(
            sock,
            SOL_SOCKET,
            SO_SNDTIMEO,
            &timeout,
            sizeof(timeout)
        );

        struct sockaddr_in server_addr = {};

        server_addr.sin_family =
            AF_INET;

        server_addr.sin_port =
            htons(
                ROBOT_SERVER_PORT
            );

        int ip_result =
            inet_pton(
                AF_INET,
                ROBOT_SERVER_IP,
                &server_addr.sin_addr
            );

        if (
            ip_result != 1
        )
        {
            ESP_LOGE(
                TAG,
                "Invalid RobotAnno server IP: %s",
                ROBOT_SERVER_IP
            );

            close(
                sock
            );

            return false;
        }

        ESP_LOGI(
            TAG,
            "Robot TCP connecting to %s:%u",
            ROBOT_SERVER_IP,
            static_cast<unsigned>(
                ROBOT_SERVER_PORT
            )
        );

        int connect_result =
            connect(
                sock,
                reinterpret_cast<struct sockaddr*>(
                    &server_addr
                ),
                sizeof(server_addr)
            );

        if (
            connect_result != 0
        )
        {
            ESP_LOGE(
                TAG,
                "Robot TCP connect failed, errno=%d",
                errno
            );

            close(
                sock
            );

            return false;
        }

        // --------------------------------------------------------------------
        // C# gateway gửi greeting ngay khi TCP client kết nối:
        // ROBOT_ANNO_READY\n
        // --------------------------------------------------------------------

        char buffer[512];

        int len =
            recv(
                sock,
                buffer,
                sizeof(buffer) - 1,
                0
            );

        if (
            len <= 0
        )
        {
            ESP_LOGE(
                TAG,
                "Robot TCP greeting receive failed, errno=%d",
                errno
            );

            close(
                sock
            );

            return false;
        }

        buffer[len] =
            '\0';

        ESP_LOGI(
            TAG,
            "Robot TCP greeting: %s",
            buffer
        );

        // --------------------------------------------------------------------
        // SEND ONE LINE
        // --------------------------------------------------------------------

        std::string packet =
            command;

        packet +=
            "\n";

        size_t total_sent =
            0;

        while (
            total_sent
            <
            packet.size()
        )
        {
            int sent =
                send(
                    sock,
                    packet.data()
                    +
                    total_sent,
                    packet.size()
                    -
                    total_sent,
                    0
                );

            if (
                sent <= 0
            )
            {
                ESP_LOGE(
                    TAG,
                    "Robot TCP send failed, errno=%d",
                    errno
                );

                close(
                    sock
                );

                return false;
            }

            total_sent +=
                static_cast<size_t>(
                    sent
                );
        }

        ESP_LOGI(
            TAG,
            "Robot TCP -> %s",
            command.c_str()
        );

        // --------------------------------------------------------------------
        // RECEIVE ONE RESPONSE LINE
        // --------------------------------------------------------------------

        response.clear();

        while (
            response.size() < 511
        )
        {
            len =
                recv(
                    sock,
                    buffer,
                    sizeof(buffer) - 1,
                    0
                );

            if (
                len <= 0
            )
            {
                break;
            }

            buffer[len] =
                '\0';

            response.append(
                buffer,
                len
            );

            if (
                response.find(
                    '\n'
                )
                !=
                std::string::npos
            )
            {
                break;
            }
        }

        close(
            sock
        );

        if (
            response.empty()
        )
        {
            ESP_LOGE(
                TAG,
                "Robot TCP response timeout/empty"
            );

            return false;
        }

        size_t newline =
            response.find(
                '\n'
            );

        if (
            newline
            !=
            std::string::npos
        )
        {
            response.erase(
                newline
            );
        }

        while (
            !response.empty()
            &&
            (
                response.back() == '\r'
                ||
                response.back() == '\n'
            )
        )
        {
            response.pop_back();
        }

        ESP_LOGI(
            TAG,
            "Robot TCP <- %s",
            response.c_str()
        );

        return true;
    }

    bool RobotCommandOk(
        const std::string& command,
        std::string& response
    ) {
        if (
            !RobotSendCommand(
                command,
                response
            )
        )
        {
            return false;
        }

        return
            response.rfind(
                "OK",
                0
            )
            ==
            0
            ||
            response ==
            "PONG"
            ||
            response.rfind(
                "POS ",
                0
            )
            ==
            0
            ||
            response.rfind(
                "STATUS ",
                0
            )
            ==
            0;
    }

    // ========================================================================
    // GET WATER - S3 ORCHESTRATOR
    //
    // Flow:
    //   0. Bơm giữ OFF theo policy của DevKit TCP-ONLY.
    //   1. S3 -> C#: GET_WATER ngay, không chờ DevKit.
    //   2. Poll WATER_STATUS mỗi 1 giây.
    //   3. Chỉ khi C# trả READY_FOR_PUMP:
    //        S3 -> DevKit: WATER_PUMP_5S.
    //   4. Đợi đúng 5 giây rồi gửi WATER_PUMP_OFF tối đa 3 lần.
    //   5. S3 -> C#: WATER_DONE.
    //
    // Tool này KHÔNG yêu cầu self.robot.enter_control trước.
    // ========================================================================

    static void WaterSequenceTaskEntry(void* arg) {
        auto* self =
            static_cast<TuanS3XiaozhiBoard*>(arg);

        self->WaterSequenceTask();

        vTaskDelete(nullptr);
    }

    void FinishWaterFlow() {
        std::lock_guard<std::mutex> lock(
            water_flow_mutex_
        );

        water_flow_running_ = false;
    }

    void SpeakAfterWaterFlow(const char* text) {
        if (
            text == nullptr
            ||
            text[0] == '\0'
        )
        {
            return;
        }

        cJSON* root =
            cJSON_CreateObject();

        if (root == nullptr)
        {
            ESP_LOGE(
                TAG,
                "Cannot create TTS JSON"
            );
            return;
        }

        cJSON_AddStringToObject(
            root,
            "type",
            "tts"
        );

        cJSON_AddStringToObject(
            root,
            "state",
            "start"
        );

        cJSON_AddStringToObject(
            root,
            "text",
            text
        );

        char* json_text =
            cJSON_PrintUnformatted(root);

        if (json_text != nullptr)
        {
            Application::GetInstance().SendMcpMessage(
                std::string(json_text)
            );

            ESP_LOGI(
                TAG,
                "Completion TTS sent: %s",
                text
            );

            cJSON_free(json_text);
        }

        cJSON_Delete(root);
    }

    void WaterSequenceTask() {
        ESP_LOGI(
            TAG,
            "GET_WATER flow started"
        );

        // --------------------------------------------------------------------
        // 0. KHÔNG gọi DevKit trước khi cho robot chạy.
        //
        // DevKit bản TCP-ONLY đã mặc định giữ bơm OFF và chỉ WATER_PUMP_5S
        // mới được phép bật relay. Vì vậy GET_WATER phải được gửi thẳng tới C#
        // trước; tránh việc một kết nối DevKit bị treo làm robot không bao giờ
        // nhận được GET_WATER.
        // --------------------------------------------------------------------
        ESP_LOGI(
            TAG,
            "GET_WATER: start RobotAnno first; pump remains OFF by DevKit policy"
        );

        // --------------------------------------------------------------------
        // 1. Bắt đầu chuỗi G-code trên PC/C#.
        // C# trả OK GET_WATER ngay rồi robot chạy ở thread riêng.
        // --------------------------------------------------------------------
        std::string response;

        ESP_LOGI(
            TAG,
            "GET_WATER: sending GET_WATER to RobotAnno gateway"
        );

        if (
            !RobotSendCommand(
                "GET_WATER",
                response
            )
            ||
            response.rfind(
                "OK GET_WATER",
                0
            ) != 0
        )
        {
            ESP_LOGE(
                TAG,
                "GET_WATER start failed: %s",
                response.c_str()
            );

            C6SendCommand(
                "WATER_PUMP_OFF"
            );

            FinishWaterFlow();
            return;
        }

        // --------------------------------------------------------------------
        // 2. Chờ robot đặt cốc xong và về đúng vị trí cho phép bơm.
        // Mỗi lần poll mở một TCP connection mới.
        // Timeout 120 giây.
        // --------------------------------------------------------------------
        bool ready_for_pump = false;

        for (
            int attempt = 0;
            attempt < 120;
            ++attempt
        )
        {
            vTaskDelay(
                pdMS_TO_TICKS(1000)
            );

            std::string status;

            if (
                !RobotSendCommand(
                    "WATER_STATUS",
                    status
                )
            )
            {
                ESP_LOGW(
                    TAG,
                    "WATER_STATUS poll failed (%d/120)",
                    attempt + 1
                );

                continue;
            }

            ESP_LOGI(
                TAG,
                "C# WATER_STATUS -> %s",
                status.c_str()
            );

            if (
                status.find(
                    "READY_FOR_PUMP"
                ) != std::string::npos
            )
            {
                ready_for_pump = true;
                break;
            }

            // STOP hoặc ERROR => tuyệt đối không bật bơm.
            if (
                status.find(
                    "STOPPED"
                ) != std::string::npos
                ||
                status.find(
                    "ERROR"
                ) != std::string::npos
            )
            {
                ESP_LOGE(
                    TAG,
                    "GET_WATER aborted by C#: %s",
                    status.c_str()
                );

                C6SendCommand(
                    "WATER_PUMP_OFF"
                );

                FinishWaterFlow();
                return;
            }
        }

        if (!ready_for_pump)
        {
            ESP_LOGE(
                TAG,
                "GET_WATER timeout waiting READY_FOR_PUMP; pump will NOT start"
            );

            C6SendCommand(
                "WATER_PUMP_OFF"
            );

            FinishWaterFlow();
            return;
        }

        // --------------------------------------------------------------------
        // 3. Robot đã báo READY_FOR_PUMP -> bật bơm 5 giây.
        // DevKit TCP-ONLY chỉ cho WATER_PUMP_5S bật relay.
        // --------------------------------------------------------------------
        std::string pump_response;

        if (
            !C6SendCommand(
                "WATER_PUMP_5S",
                &pump_response
            )
        )
        {
            ESP_LOGE(
                TAG,
                "Cannot connect DevKit for WATER_PUMP_5S"
            );

            C6SendCommand(
                "WATER_PUMP_OFF"
            );

            FinishWaterFlow();
            return;
        }

        ESP_LOGI(
            TAG,
            "DevKit WATER_PUMP_5S -> %s",
            pump_response.c_str()
        );

        ESP_LOGI(
            TAG,
            "Pump running; explicit WATER_PUMP_OFF in 5000 ms"
        );

        vTaskDelay(
            pdMS_TO_TICKS(5000)
        );

        // --------------------------------------------------------------------
        // 4. Tắt bơm trực tiếp sau đúng 5 giây.
        // DevKit cũng có timer auto-OFF 5 giây làm fail-safe.
        // --------------------------------------------------------------------
        bool pump_off_sent = false;

        for (
            int attempt = 0;
            attempt < 3;
            ++attempt
        )
        {
            std::string off_response;

            if (
                C6SendCommand(
                    "WATER_PUMP_OFF",
                    &off_response
                )
            )
            {
                ESP_LOGI(
                    TAG,
                    "DevKit WATER_PUMP_OFF -> %s",
                    off_response.c_str()
                );

                pump_off_sent = true;
                break;
            }

            ESP_LOGW(
                TAG,
                "WATER_PUMP_OFF retry %d/3",
                attempt + 1
            );

            vTaskDelay(
                pdMS_TO_TICKS(200)
            );
        }

        if (!pump_off_sent)
        {
            ESP_LOGE(
                TAG,
                "WATER_PUMP_OFF command failed after 3 attempts; "
                "DevKit 5-second auto-OFF remains the fail-safe"
            );
        }

        // --------------------------------------------------------------------
        // 5. Báo C# chu kỳ lấy nước đã hoàn tất.
        // --------------------------------------------------------------------
        std::string done_response;

        if (
            !RobotSendCommand(
                "WATER_DONE",
                done_response
            )
        )
        {
            ESP_LOGW(
                TAG,
                "Pump completed but WATER_DONE could not reach C#"
            );
        }
        else
        {
            ESP_LOGI(
                TAG,
                "C# WATER_DONE -> %s",
                done_response.c_str()
            );
        }

        // --------------------------------------------------------------------
        // 6. Chờ C# đưa cốc tới khách và về HOME xong.
        // Giữ water_flow_running_ đến COMPLETE để không nhận đơn chồng.
        // --------------------------------------------------------------------
        bool delivery_complete = false;

        for (
            int attempt = 0;
            attempt < 180;
            ++attempt
        )
        {
            vTaskDelay(
                pdMS_TO_TICKS(1000)
            );

            std::string delivery_status;

            if (
                RobotSendCommand(
                    "WATER_STATUS",
                    delivery_status
                )
            )
            {
                ESP_LOGI(
                    TAG,
                    "C# delivery status -> %s",
                    delivery_status.c_str()
                );

                if (
                    delivery_status.find(
                        "COMPLETE"
                    ) != std::string::npos
                )
                {
                    delivery_complete = true;
                    break;
                }

                if (
                    delivery_status.find(
                        "STOPPED"
                    ) != std::string::npos
                    ||
                    delivery_status.find(
                        "ERROR"
                    ) != std::string::npos
                )
                {
                    break;
                }
            }
        }

        if (!delivery_complete)
        {
            ESP_LOGW(
                TAG,
                "C# did not report COMPLETE before timeout"
            );
        }
        else
        {
            // Chỉ nói sau khi C# xác nhận robot đã giao cốc và về HOME.
            SpeakAfterWaterFlow(
                "Cảm ơn quý khách."
            );
        }

        ESP_LOGI(
            TAG,
            "GET_WATER / SELL_WATER flow completed"
        );

        FinishWaterFlow();
    }

    // ========================================================================
    // MCP TOOLS
    // ========================================================================

    void InitializeTools() {
        auto& mcp_server =
            McpServer::GetInstance();

        // ====================================================================
        // LIGHT
        // ====================================================================

        mcp_server.AddTool(
            "self.zigbee.light",

            "Control the Zigbee light. "
            "Use this tool when the user asks to turn the light on or off.",

            PropertyList({
                Property(
                    "state",
                    kPropertyTypeBoolean
                )
            }),

            [this](
                const PropertyList& properties
            ) -> ReturnValue {
                bool state =
                    properties[
                        "state"
                    ].value<bool>();

                bool ok =
                    C6SendCommand(
                        state
                        ?
                        "LIGHT_CMD:1"
                        :
                        "LIGHT_CMD:0"
                    );

                if (
                    !ok
                )
                {
                    return std::string(
                        "Không kết nối được Smart Home Gateway (DevKit)."
                    );
                }

                if (
                    state
                )
                {
                    return std::string(
                        "Đã gửi lệnh bật đèn Zigbee."
                    );
                }

                return std::string(
                    "Đã gửi lệnh tắt đèn Zigbee."
                );
            }
        );

        // ====================================================================
        // OPEN DEVICE PAIRING
        // ====================================================================

        mcp_server.AddTool(
    "self.zigbee.open_device",

    "Open the Zigbee network for pairing new devices for 180 seconds. "
    "Use this when the user asks to add, pair, connect, "
    "or open connection for a Zigbee device.",

    PropertyList(),

    [this](
        const PropertyList&
    ) -> ReturnValue {
        if (
            !C6SendCommand(
                "OPEN_DEVICE"
            )
        )
        {
            return std::string(
                "Không kết nối được Smart Home Gateway (DevKit)."
            );
        }

        return std::string(
            "Đã mở kết nối thiết bị Zigbee trong 3 phút."
        );
    }
);

        // ====================================================================
        // CLOSE PAIRING
        // ====================================================================

        mcp_server.AddTool(
            "self.zigbee.close_device",

            "Close Zigbee pairing and stop new devices from joining.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                bool ok =
                    C6SendCommand(
                        "CLOSE_DEVICE"
                    );

                if (
                    !ok
                )
                {
                    return std::string(
                        "Không kết nối được Smart Home Gateway (DevKit)."
                    );
                }

                return std::string(
                    "Đã đóng kết nối thiết bị Zigbee."
                );
            }
        );

        // ====================================================================
        // GET CURRENT STATUS
        // ====================================================================

        mcp_server.AddTool(
            "self.zigbee.get_status",

            "Get current Zigbee smart home status including light, motion, "
            "temperature and humidity. climate_data_fresh=true means the "
            "climate sensor has produced a recent reading. If fresh=false but "
            "temperature/humidity are available, those values are cached/stale "
            "and do NOT prove the sensor is currently connected.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                // Lấy snapshot trực tiếp từ DevKit.
                // command_response giúp C6SendCommand trả ngay sau một dòng,
                // không chờ recv timeout 1 giây.
                std::string status_response;

                C6SendCommand(
                    "STATUS",
                    &status_response
                );

                char result[768];

                const char* temp_text =
                    temperature_valid_
                    ?
                    "available"
                    :
                    "unavailable";

                const char* humidity_text =
                    humidity_valid_
                    ?
                    "available"
                    :
                    "unavailable";

                snprintf(
                    result,
                    sizeof(result),

                    "{"
                    "\"c6_online\":%s,"
                    "\"zigbee_ready\":%s,"
                    "\"motion\":%s,"
                    "\"light_on\":%s,"
                    "\"motion_device\":%s,"
                    "\"light_device\":%s,"
                    "\"climate_device\":%s,"
                    "\"climate_data_fresh\":%s,"
                    "\"temperature_status\":\"%s\","
                    "\"temperature\":%.2f,"
                    "\"temperature_age_ms\":%lu,"
                    "\"humidity_status\":\"%s\","
                    "\"humidity\":%.2f,"
                    "\"humidity_age_ms\":%lu"
                    "}",

                    c6_online_
                    ? "true"
                    : "false",

                    zigbee_ready_
                    ? "true"
                    : "false",

                    motion_detected_
                    ? "true"
                    : "false",

                    light_on_
                    ? "true"
                    : "false",

                    motion_device_connected_
                    ? "true"
                    : "false",

                    light_device_connected_
                    ? "true"
                    : "false",

                    climate_device_connected_
                    ? "true"
                    : "false",

                    climate_data_fresh_
                    ? "true"
                    : "false",

                    temp_text,

                    temperature_,

                    static_cast<unsigned long>(
                        temperature_age_ms_
                    ),

                    humidity_text,

                    humidity_,

                    static_cast<unsigned long>(
                        humidity_age_ms_
                    )
                );

                return std::string(
                    result
                );
            }
        );

        // ====================================================================
        // READ TEMPERATURE / HUMIDITY
        // ====================================================================

        mcp_server.AddTool(
            "self.zigbee.read_climate",

            "Request a fresh temperature and humidity reading "
            "from the Zigbee climate sensor.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                bool ok =
                    C6SendCommand(
                        "READ_CLIMATE"
                    );

                if (
                    !ok
                )
                {
                    return std::string(
                        "Không kết nối được Smart Home Gateway (DevKit)."
                    );
                }

                return std::string(
                    "Đã yêu cầu ESP32-C6 đọc nhiệt độ và độ ẩm."
                );
            }
        );

        ESP_LOGI(
            TAG,
            "Zigbee MCP tools registered"
        );

        // ====================================================================
        // ROBOTANNO - ENTER CONTROL MODE
        //
        // Câu ví dụ: "Khởi động robot"
        // Tool sẽ PING gateway trước khi bật robot_control_mode_.
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.enter_control",

            "Khởi động chế độ điều khiển robot RobotAnno SJ602-A. "
            "BẮT BUỘC gọi công cụ này khi người dùng nói 'Khởi động robot', "
            "'Bật robot', 'Bắt đầu điều khiển robot', 'Kết nối robot' "
            "hoặc câu có ý nghĩa tương đương. "
            "Chỉ bật chế độ điều khiển sau khi máy tính trả lời PONG.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::string response;

                bool ok =
                    RobotSendCommand(
                        "PING",
                        response
                    );

                if (
                    !ok
                    ||
                    response != "PONG"
                )
                {
                    robot_control_mode_ =
                        false;

                    return std::string(
                        "Không kết nối được với RobotAnno gateway."
                    );
                }

                robot_control_mode_ =
                    true;

                ESP_LOGI(
                    TAG,
                    "RobotAnno control mode ON"
                );

                return std::string(
                    "Robot đã khởi động và sẵn sàng điều khiển."
                );
            }
        );

        // ====================================================================
        // ROBOTANNO - EXIT CONTROL MODE
        //
        // Câu ví dụ: "Thoát điều khiển robot"
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.exit_control",

            "Thoát chế độ điều khiển RobotAnno SJ602-A. "
            "Anno may be transcribed by Vietnamese speech recognition as Anu, A-nô, "
            "Anh Nô, Ano, Anh No, A No, or similar. Treat those names as RobotAnno. "
            "Use this when the user says 'Thoát Anno', 'Thoát Anu', 'Thoát Anh Nô', "
            "'dừng điều khiển Anno', or asks to leave RobotAnno control mode. "
            "This does not move the robot.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                robot_control_mode_ =
                    false;

                ESP_LOGI(
                    TAG,
                    "RobotAnno control mode OFF"
                );

                return std::string(
                    "Đã thoát chế độ điều khiển RobotAnno."
                );
            }
        );

        // ====================================================================
        // ROBOTANNO - SET SPEED
        //
        // vp = 1..100
        // S3 gửi: VP 5
        // C# gửi robot: G07 VP=5
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.set_speed",

            "Đặt tốc độ di chuyển của robot theo phần trăm. "
            "Anno may be transcribed as Anu, A-nô, Anh Nô, Ano, Anh No, A No, or similar; "
            "treat those pronunciations as RobotAnno. "
            "Chỉ dùng khi robot đã được khởi động. "
            "Giá trị tốc độ là số nguyên từ 1 đến 100.",

            PropertyList({
                Property(
                    "vp",
                    kPropertyTypeInteger
                )
            }),

            [this](
                const PropertyList& properties
            ) -> ReturnValue {
                if (
                    !robot_control_mode_
                )
                {
                    return std::string(
                        "Robot chưa được khởi động. Hãy nói khởi động robot trước."
                    );
                }

                int vp =
                    properties[
                        "vp"
                    ].value<int>();

                if (
                    vp < 1
                    ||
                    vp > 100
                )
                {
                    return std::string(
                        "Tốc độ RobotAnno phải từ 1 đến 100 phần trăm."
                    );
                }

                std::string response;

                std::string command =
                    "VP "
                    +
                    std::to_string(
                        vp
                    );

                if (
                    !RobotCommandOk(
                        command,
                        response
                    )
                )
                {
                    if (
                        response.empty()
                    )
                    {
                        return std::string(
                            "Không kết nối được RobotAnno gateway."
                        );
                    }

                    return std::string(
                        "RobotAnno báo lỗi: "
                    )
                    +
                    response;
                }

                return std::string(
                    "RobotAnno: "
                )
                +
                response;
            }
        );

        // ====================================================================
        // ROBOTANNO - RELATIVE MOVE 1..6 JOINTS AT ONCE
        //
        // Luôn truyền đủ j1..j6.
        // Trục không yêu cầu di chuyển => delta = 0.
        //
        // Ví dụ:
        // "trục 1 đi 4 độ, trục 2 đi 5 độ"
        //
        // MCP:
        // j1=4 j2=5 j3=0 j4=0 j5=0 j6=0
        //
        // TCP:
        // MOVE 1 4 2 5
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.move_relative",

            "Move one or more RobotAnno SJ602-A joints relatively. "
            "Anno may be transcribed as Anu, A-nô, Anh Nô, Ano, Anh No, A No, or similar; "
            "treat those pronunciations as RobotAnno. "
            "Chỉ dùng khi robot đã được khởi động. "
            "Arguments j1 through j6 are relative angle changes in whole degrees. "
            "ALWAYS provide all six arguments. "
            "For every joint the user did not ask to move, pass 0. "
            "Example: user says 'trục 1 đi 4 độ và trục 2 đi 5 độ': "
            "j1=4, j2=5, j3=0, j4=0, j5=0, j6=0. "
            "Negative values move in the negative direction.",

            PropertyList({
                Property(
                    "j1",
                    kPropertyTypeInteger
                ),
                Property(
                    "j2",
                    kPropertyTypeInteger
                ),
                Property(
                    "j3",
                    kPropertyTypeInteger
                ),
                Property(
                    "j4",
                    kPropertyTypeInteger
                ),
                Property(
                    "j5",
                    kPropertyTypeInteger
                ),
                Property(
                    "j6",
                    kPropertyTypeInteger
                )
            }),

            [this](
                const PropertyList& properties
            ) -> ReturnValue {
                if (
                    !robot_control_mode_
                )
                {
                    return std::string(
                        "Robot chưa được khởi động. Hãy nói khởi động robot trước."
                    );
                }

                int delta[6] = {
                    properties["j1"].value<int>(),
                    properties["j2"].value<int>(),
                    properties["j3"].value<int>(),
                    properties["j4"].value<int>(),
                    properties["j5"].value<int>(),
                    properties["j6"].value<int>()
                };

                bool has_movement =
                    false;

                std::string command =
                    "MOVE";

                for (
                    int i = 0;
                    i < 6;
                    ++i
                )
                {
                    // Giới hạn delta mỗi lần gọi.
                    // Đây là giới hạn lệnh tương đối, KHÔNG phải soft-limit cơ khí.
                    if (
                        delta[i] < -180
                        ||
                        delta[i] > 180
                    )
                    {
                        return std::string(
                            "Góc thay đổi mỗi trục phải nằm trong khoảng -180 đến 180 độ."
                        );
                    }

                    if (
                        delta[i] == 0
                    )
                    {
                        continue;
                    }

                    has_movement =
                        true;

                    command +=
                        " "
                        +
                        std::to_string(
                            i + 1
                        )
                        +
                        " "
                        +
                        std::to_string(
                            delta[i]
                        );
                }

                if (
                    !has_movement
                )
                {
                    return std::string(
                        "Không có trục nào cần di chuyển."
                    );
                }

                std::string response;

                if (
                    !RobotCommandOk(
                        command,
                        response
                    )
                )
                {
                    if (
                        response.empty()
                    )
                    {
                        return std::string(
                            "Không kết nối được RobotAnno gateway."
                        );
                    }

                    return std::string(
                        "RobotAnno báo lỗi: "
                    )
                    +
                    response;
                }

                return std::string(
                    "RobotAnno: "
                )
                +
                response;
            }
        );

        // ====================================================================
        // ROBOTANNO - ABSOLUTE MOVE
        //
        // Ví dụ:
        // ABS 10 20 -30 0 15 0
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.move_absolute",

            "Move RobotAnno SJ602-A to an absolute six-joint target. "
            "Anno may be transcribed as Anu, A-nô, Anh Nô, Ano, Anh No, A No, or similar; "
            "treat those pronunciations as RobotAnno. "
            "Chỉ dùng khi robot đã được khởi động. "
            "Always provide all six joint angles j1 through j6 in whole degrees. "
            "Use this only when the user clearly requests absolute joint positions.",

            PropertyList({
                Property(
                    "j1",
                    kPropertyTypeInteger
                ),
                Property(
                    "j2",
                    kPropertyTypeInteger
                ),
                Property(
                    "j3",
                    kPropertyTypeInteger
                ),
                Property(
                    "j4",
                    kPropertyTypeInteger
                ),
                Property(
                    "j5",
                    kPropertyTypeInteger
                ),
                Property(
                    "j6",
                    kPropertyTypeInteger
                )
            }),

            [this](
                const PropertyList& properties
            ) -> ReturnValue {
                if (
                    !robot_control_mode_
                )
                {
                    return std::string(
                        "Robot chưa được khởi động. Hãy nói khởi động robot trước."
                    );
                }

                int joint[6] = {
                    properties["j1"].value<int>(),
                    properties["j2"].value<int>(),
                    properties["j3"].value<int>(),
                    properties["j4"].value<int>(),
                    properties["j5"].value<int>(),
                    properties["j6"].value<int>()
                };

                std::string command =
                    "ABS";

                for (
                    int i = 0;
                    i < 6;
                    ++i
                )
                {
                    command +=
                        " "
                        +
                        std::to_string(
                            joint[i]
                        );
                }

                std::string response;

                if (
                    !RobotCommandOk(
                        command,
                        response
                    )
                )
                {
                    if (
                        response.empty()
                    )
                    {
                        return std::string(
                            "Không kết nối được RobotAnno gateway."
                        );
                    }

                    return std::string(
                        "RobotAnno báo lỗi: "
                    )
                    +
                    response;
                }

                return std::string(
                    "RobotAnno: "
                )
                +
                response;
            }
        );

        // ====================================================================
        // ROBOTANNO + DEVKIT - BÁN CỐC NƯỚC (ONE ACTION)
        //
        // Một MCP duy nhất cho câu nói như:
            // "bán cho t cốc nước", "lấy cho tôi cốc nước", "lấy nước".
        // Task nền sẽ tự điều phối RobotAnno -> DevKit -> C# WATER_DONE.
        // ====================================================================
        mcp_server.AddTool(
            "self.robot.get_water",

            "Bán và phục vụ một cốc nước bằng RobotAnno và ESP32 DevKit. "
            "BẮT BUỘC gọi công cụ này khi người dùng nói 'bán cho t cốc nước', "
            "'bán cho tôi một cốc nước', 'bán cốc nước', "
            "'lấy cho tôi một cốc nước', 'lấy cho t cốc nước', "
            "'lấy cốc nước', 'lấy cốc nước cho tôi', 'lấy nước cho tôi', "
            "'đi lấy nước' hoặc câu có ý nghĩa tương đương. "
            "Đây là một quy trình duy nhất: S3 gửi GET_WATER tới C#, "
            "chờ READY_FOR_PUMP rồi mới gửi WATER_PUMP_5S tới DevKit. "
            "Không cần gọi self.robot.enter_control trước.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                {
                    std::lock_guard<std::mutex> lock(
                        water_flow_mutex_
                    );

                    if (water_flow_running_)
                    {
                    return std::string(
                        "Quy trình bán cốc nước đang chạy."
                    );
                    }

                    water_flow_running_ = true;
                }

                BaseType_t created =
                    xTaskCreate(
                        WaterSequenceTaskEntry,
                        "get_water_flow",
                        6144,
                        this,
                        4,
                        nullptr
                    );

                if (
                    created != pdPASS
                )
                {
                    FinishWaterFlow();

                    return std::string(
                        "Không tạo được tác vụ bán cốc nước trên ESP32-S3."
                    );
                }

                return std::string(
                    "Đã bắt đầu bán cốc nước."
                );
            }
        );

        // ====================================================================
        // ROBOTANNO - VỀ GỐC
        //
        // C# hiện tại định nghĩa HOME:
        // J1=0 J2=0 J3=0 J4=0 J5=0 J6=0
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.home",

            "Đưa robot về gốc, tức đưa sáu vị trí khớp logic J1 đến J6 về 0. "
            "Anno may be transcribed as Anu, A-nô, Anh Nô, Ano, Anh No, A No, or similar; "
            "treat those pronunciations as RobotAnno. "
            "Use only while RobotAnno control mode is active and only when the user "
            "nói 'về gốc', 'robot về gốc' hoặc 'đưa robot về vị trí gốc'.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                if (
                    !robot_control_mode_
                )
                {
                    return std::string(
                        "Robot chưa được khởi động. Hãy nói khởi động robot trước."
                    );
                }

                std::string response;

                if (
                    !RobotCommandOk(
                        "HOME",
                        response
                    )
                )
                {
                    if (
                        response.empty()
                    )
                    {
                        return std::string(
                            "Không kết nối được RobotAnno gateway."
                        );
                    }

                    return std::string(
                        "RobotAnno báo lỗi: "
                    )
                    +
                    response;
                }

                return std::string(
                    "RobotAnno: "
                )
                +
                response;
            }
        );

        // ====================================================================
        // ROBOTANNO - STOP
        //
        // STOP được phép gọi kể cả khi robot_control_mode_ = false.
        // C# sẽ gửi raw byte 0x30 tới robot.
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.stop",

            "Dừng robot ngay lập tức. "
            "Anno may be transcribed as Anu, A-nô, Anh Nô, Ano, Anh No, A No, or similar; "
            "treat those pronunciations as RobotAnno. "
            "Use whenever the user says stop, emergency stop, dừng robot, "
            "or asks RobotAnno to stop. "
            "This tool is allowed even when RobotAnno control mode is not active.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::string response;

                if (
                    !RobotCommandOk(
                        "STOP",
                        response
                    )
                )
                {
                    if (
                        response.empty()
                    )
                    {
                        return std::string(
                            "Không kết nối được RobotAnno gateway để gửi STOP."
                        );
                    }

                    return std::string(
                        "RobotAnno báo lỗi khi STOP: "
                    )
                    +
                    response;
                }

                return std::string(
                    "RobotAnno: "
                )
                +
                response;
            }
        );

        // ====================================================================
        // ROBOTANNO - GET POSITION
        //
        // LƯU Ý: C# gateway hiện trả về vị trí LOGIC mà chương trình đang lưu,
        // chưa phải feedback encoder thực từ robot.
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.get_position",

            "Đọc vị trí logic hiện tại của sáu khớp robot. "
            "Anno may be transcribed as Anu, A-nô, Anh Nô, Ano, Anh No, A No, or similar; "
            "treat those pronunciations as RobotAnno. "
            "The returned values are the positions tracked by the PC gateway, "
            "không phải phản hồi encoder thực tế từ robot.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::string response;

                if (
                    !RobotCommandOk(
                        "POS",
                        response
                    )
                )
                {
                    if (
                        response.empty()
                    )
                    {
                        return std::string(
                            "Không kết nối được RobotAnno gateway."
                        );
                    }

                    return std::string(
                        "RobotAnno báo lỗi: "
                    )
                    +
                    response;
                }

                return response;
            }
        );

        // ====================================================================
        // ROBOTANNO - STATUS
        // ====================================================================

        mcp_server.AddTool(
            "self.robot.get_status",

            "Đọc trạng thái robot gồm kết nối, vị trí logic các khớp, "
            "trạng thái bận và tốc độ hiện tại. "
            "Anno may be transcribed as Anu, A-nô, Anh Nô, Ano, Anh No, A No, or similar; "
            "treat those pronunciations as RobotAnno.",

            PropertyList(),

            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::string response;

                if (
                    !RobotCommandOk(
                        "STATUS",
                        response
                    )
                )
                {
                    if (
                        response.empty()
                    )
                    {
                        return std::string(
                            "Không kết nối được RobotAnno gateway."
                        );
                    }

                    return std::string(
                        "RobotAnno báo lỗi: "
                    )
                    +
                    response;
                }

                return response;
            }
        );

        // MOBILE BASE MCP
        // ====================================================================

        mcp_server.AddTool(
            "self.car.set_speed",
            "Đặt tốc độ xe từ 0 đến 100 phần trăm.",
            PropertyList({
                Property(
                    "speed",
                    kPropertyTypeInteger
                )
            }),
            [this](
                const PropertyList& properties
            ) -> ReturnValue {
                const int speed =
                    properties[
                        "speed"
                    ].value<int>();

                if (speed < 0 || speed > 100) {
                    return std::string(
                        "Tốc độ xe phải từ 0 đến 100 phần trăm."
                    );
                }

                std::lock_guard<std::mutex> lock(
                    car_mutex_
                );

                car_speed_ = speed;

                if (speed == 0) {
                    CarStopLocked();
                }
                else if (
                    car_left_dir_ != 0
                    ||
                    car_right_dir_ != 0
                ) {
                    MotorRaw(
                        car_left_dir_,
                        car_right_dir_,
                        car_speed_
                    );
                }

                return
                    std::string("Đã đặt tốc độ xe ")
                    +
                    std::to_string(speed)
                    +
                    "%.";
            }
        );

        mcp_server.AddTool(
            "self.car.forward",
            "Cho xe đi thẳng. Dùng khi người dùng nói đi thẳng, tiến lên hoặc chạy tới.",
            PropertyList(),
            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::lock_guard<std::mutex> lock(
                    car_mutex_
                );

                if (!motor_ready_) {
                    return std::string(
                        "Bộ điều khiển động cơ DRV8833 chưa sẵn sàng."
                    );
                }

                if (car_speed_ <= 0) {
                    return std::string(
                        "Tốc độ xe đang bằng 0."
                    );
                }

                if (!CarMoveLocked(+1, +1, true)) {
                    return std::string(
                        "Phía trước có vật cản, xe không tiến."
                    );
                }

                return std::string(
                    "Xe đang đi thẳng."
                );
            }
        );

        mcp_server.AddTool(
            "self.car.backward",
            "Cho xe lùi. Dùng khi người dùng nói lùi, đi lùi hoặc lùi lại.",
            PropertyList(),
            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::lock_guard<std::mutex> lock(
                    car_mutex_
                );

                if (!motor_ready_) {
                    return std::string(
                        "Bộ điều khiển động cơ DRV8833 chưa sẵn sàng."
                    );
                }

                if (car_speed_ <= 0) {
                    return std::string(
                        "Tốc độ xe đang bằng 0."
                    );
                }

                CarMoveLocked(-1, -1, false);

                return std::string(
                    "Xe đang lùi."
                );
            }
        );

        mcp_server.AddTool(
            "self.car.left",
            "Cho xe quay trái tại chỗ. Dùng khi người dùng nói rẽ trái hoặc quay trái.",
            PropertyList(),
            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::lock_guard<std::mutex> lock(
                    car_mutex_
                );

                if (!motor_ready_) {
                    return std::string(
                        "Bộ điều khiển động cơ DRV8833 chưa sẵn sàng."
                    );
                }

                if (car_speed_ <= 0) {
                    return std::string(
                        "Tốc độ xe đang bằng 0."
                    );
                }

                CarMoveLocked(-1, +1, false);

                return std::string(
                    "Xe đang quay trái."
                );
            }
        );

        mcp_server.AddTool(
            "self.car.right",
            "Cho xe quay phải tại chỗ. Dùng khi người dùng nói rẽ phải hoặc quay phải.",
            PropertyList(),
            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::lock_guard<std::mutex> lock(
                    car_mutex_
                );

                if (!motor_ready_) {
                    return std::string(
                        "Bộ điều khiển động cơ DRV8833 chưa sẵn sàng."
                    );
                }

                if (car_speed_ <= 0) {
                    return std::string(
                        "Tốc độ xe đang bằng 0."
                    );
                }

                CarMoveLocked(+1, -1, false);

                return std::string(
                    "Xe đang quay phải."
                );
            }
        );

        mcp_server.AddTool(
            "self.car.stop",
            "Dừng xe ngay. Dùng khi người dùng nói dừng xe, xe dừng lại hoặc đứng lại.",
            PropertyList(),
            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::lock_guard<std::mutex> lock(
                    car_mutex_
                );

                if (!motor_ready_) {
                    return std::string(
                        "Bộ điều khiển động cơ DRV8833 chưa sẵn sàng."
                    );
                }

                CarStopLocked();

                return std::string(
                    "Xe đã dừng."
                );
            }
        );

        mcp_server.AddTool(
            "self.car.get_distance",
            "Đọc khoảng cách phía trước bằng VL6180X.",
            PropertyList(),
            [this](
                const PropertyList&
            ) -> ReturnValue {
                std::lock_guard<std::mutex> lock(
                    car_mutex_
                );

                if (!tof_ready_) {
                    return std::string(
                        "Không tìm thấy cảm biến VL6180X."
                    );
                }

                const int mm =
                    DistanceMm();

                if (mm < 0) {
                    return std::string(
                        "Không đọc được khoảng cách từ VL6180X."
                    );
                }

                return
                    std::string("Khoảng cách phía trước là ")
                    +
                    std::to_string(mm)
                    +
                    " mm.";
            }
        );

        ESP_LOGI(
            TAG,
            "Car MCP tools registered"
        );

        ESP_LOGI(
            TAG,
            "RobotAnno MCP tools registered (GET_WATER enabled; CHAO/MUA removed)"
        );
    }

public:
    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    TuanS3XiaozhiBoard()
        :
        boot_button_(
            BOOT_BUTTON_GPIO
        )
    {
        // Khoi tao VL6180X truoc cac thanh phan XiaoZhi khac de tranh xung dot I2C/GPIO.
        InitializeTof();

        InitializeSpi();

        InitializeDisplay();

        InitializeButtons();

        // DRV8833 luôn được khởi tạo; VL6180X lỗi không chặn motor.
        InitializeCar();

        // Smart Home dùng Wi-Fi/TCP tới DevKit, không còn UART trực tiếp C6.
        InitializeTools();

        StartDeviceEventTask();

        ESP_LOGI(
            TAG,
            "Tuan S3 XiaoZhi board initialized"
        );
    }

    // ========================================================================
    // AUDIO
    // ========================================================================

    virtual AudioCodec* GetAudioCodec()
        override
    {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,

            AUDIO_I2S_SPK_GPIO_BCLK,
            AUDIO_I2S_SPK_GPIO_LRCK,
            AUDIO_I2S_SPK_GPIO_DOUT,

            AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_WS,
            AUDIO_I2S_MIC_GPIO_DIN
        );

        return &audio_codec;
    }

    // ========================================================================
    // DISPLAY
    // ========================================================================

    virtual Display* GetDisplay()
        override
    {
        return display_;
    }
};

DECLARE_BOARD(
    TuanS3XiaozhiBoard
);
