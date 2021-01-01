#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include <stdarg.h>

// this should be replaced by a standard
// Arduino library
#include "MPU9250.h"

#include "cobs.h"
#include "CRC16.h"

#define FW_MAJOR 0
#define FW_MINOR 1

#define RM_ENCODER_A 2
#define RM_ENCODER_B 4
#define LM_ENCODER_A 3 //18 before
#define LM_ENCODER_B 22

#define UINT16_TO_BE(x, p)            \
    serializable_obj.u16 = x;         \
    *(p) = serializable_obj.bytes[1]; \
    *(p + 1) = serializable_obj.bytes[0];

#define FLOAT_TO_BE(x, p)                 \
    serializable_obj.f = x;               \
    *(p) = serializable_obj.bytes[3];     \
    *(p + 1) = serializable_obj.bytes[2]; \
    *(p + 2) = serializable_obj.bytes[1]; \
    *(p + 3) = serializable_obj.bytes[0];

#define LOG_FATAL 0u   /* system is unusable */
#define LOG_ERR 1u     /* error conditions */
#define LOG_WARNING 2u /* warning conditions */
#define LOG_INFO 3u    /* informational */
#define LOG_DEBUG 4u   /* debug-level messages */

#define FRAME_TYPE_DATA 0u
#define FRAME_TYPE_LOG 1u
#define FRAME_TYPE_CMD 2u
/**
 * Basic sensor data frame has 45 + 2 CRC bytes of raw data:
 * 1byte frame type + 9x4 bytes IMU data + 4 bytes (voltage: Float) + 2 bytes lelf wheel tick + 2 bytes right wheel tick
 * 
 * We allocate a buffer of 256 bytes for buffer
*/
#define MAX_BUFFER_LEN 256
#define MAX_RAW_DATA_LEN (MAX_BUFFER_LEN - 4u)

#define OFFSET_GYRO_X 1
#define OFFSET_GYRO_Y (OFFSET_GYRO_X + 4u)
#define OFFSET_GYRO_Z (OFFSET_GYRO_Y + 4u)

#define OFFSET_ACCEL_X (OFFSET_GYRO_Z + 4u)
#define OFFSET_ACCEL_Y (OFFSET_ACCEL_X + 4u)
#define OFFSET_ACCEL_Z (OFFSET_ACCEL_Y + 4u)

#define OFFSET_MAG_X (OFFSET_ACCEL_Z + 4u)
#define OFFSET_MAG_Y (OFFSET_MAG_X + 4u)
#define OFFSET_MAG_Z (OFFSET_MAG_Y + 4u)

#define OFFSET_BAT (OFFSET_MAG_Z + 4)

#define OFFSET_LEFT_TICK (OFFSET_BAT + 4)
#define OFFSET_RIGHT_TICK (OFFSET_LEFT_TICK + 2)

#define SENSOR_DATA_SIZE (OFFSET_RIGHT_TICK + 2)

#define ADS_MULTIPLIER 3.0F

#define LOG_PREFIX "[JETTY]:"
#define LOG_PREFIX_LEN 8u

#define FRAME_CMD_LEN 7u
#define FRAME_LOG_CMD_LEN 4u

typedef union
{
    float f;
    uint16_t u16;
    uint8_t bytes[4];
} serializable_t;

static uint8_t tx_frame_buffer[MAX_BUFFER_LEN];
static uint8_t rx_frame_buffer[MAX_BUFFER_LEN];
static uint8_t *raw_tx_frame_ptr;
static uint8_t *encoded_tx_frame_ptr;
static uint8_t *encoded_rx_frame_ptr;
static uint8_t *raw_rx_frame_ptr;

static size_t i, rx_counter;
static uint8_t log_level;
static int status;
static serializable_t serializable_obj;

static int16_t left_motor_tick = 0;
static int16_t right_motor_tick = 0;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 mpu;

Adafruit_ADS1015 ads;

// motor
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);

static void log(uint8_t level, char *fmt, ...);
static void send_frame(size_t size);
static void left_encoder_event();
static void right_encoder_event();
static void send_sensors_data();
static void process_cmd();
static void motor_ctrl(Adafruit_DCMotor *M, int16_t data);

static void motor_ctrl(Adafruit_DCMotor *M, int16_t data)
{
    uint8_t speed;
    uint8_t cmd;

    speed = 255;
    if (data < 0)
    {
        if (data > -255)
            speed = -data;
        cmd = BACKWARD;
    }
    else
    {
        if (data < 255)
            speed = data;
        cmd = FORWARD;
    }
    M->setSpeed(speed);
    M->run(cmd);
}

static void send_frame(size_t size)
{
    // crc store in network by order
    UINT16_TO_BE(crc16_ccitt(raw_tx_frame_ptr, size), raw_tx_frame_ptr + size);
    i = cobs_encode(raw_tx_frame_ptr, size + 2, encoded_tx_frame_ptr);
    encoded_tx_frame_ptr[i] = 0;
    i++;
    Serial.write(encoded_tx_frame_ptr, i);
}

static void process_cmd()
{
    int j = 0;
    // read data until 0
    while (Serial.available() > 0)
    {
        if (rx_counter >= MAX_BUFFER_LEN)
        {
            log(LOG_ERR, "RX buffer overflow. Reset it");
            rx_counter = 0;
            break;
        }
        else
        {
            encoded_rx_frame_ptr[rx_counter] = Serial.read();
            rx_counter++;
            if (encoded_rx_frame_ptr[rx_counter - 1] == 0)
            {
                // reach end of frame
                // process it
                i = cobs_decode(encoded_rx_frame_ptr, rx_counter);

                // check CRC value
                status = (raw_rx_frame_ptr[i - 2] << 8) | (raw_rx_frame_ptr[i - 1]);
                if (status == crc16_ccitt(raw_rx_frame_ptr, i - 2))
                {
                    // process the command
                    switch (raw_rx_frame_ptr[0])
                    {
                    case FRAME_TYPE_CMD:
                        if (i == FRAME_CMD_LEN)
                        {
                            motor_ctrl(M3, (raw_rx_frame_ptr[1] << 8) | raw_rx_frame_ptr[2]);
                            motor_ctrl(M2, (raw_rx_frame_ptr[3] << 8) | raw_rx_frame_ptr[4]);
                        }
                        else
                        {
                            log(LOG_WARNING, "CMD Frame size mistmatched, expected: %d get: %d", FRAME_CMD_LEN, i);
                        }
                        break;
                    case FRAME_TYPE_LOG:
                        if (i == FRAME_LOG_CMD_LEN)
                        {
                            log_level = raw_rx_frame_ptr[1];
                            log(LOG_WARNING, "Log level set to: %d", log_level);
                        }
                        else
                        {
                            log(LOG_WARNING, "LOG CMD Frame size mistmatched, expected: %d get: %d", FRAME_LOG_CMD_LEN, i);
                        }
                        break;
                    default:
                        log(LOG_WARNING, "Unknow command type: %d", raw_rx_frame_ptr[0]);
                        break;
                    }
                }
                else
                {
                    log(LOG_WARNING, "Frame CRC error, frame will be ignored: 0x%.4X", status);
                }
                // reset the buffer counter
                rx_counter = 0;
                break;
            }
        }
    }
}

static void log(uint8_t level, char *fmt, ...)
{
    va_list args;
    if (level > log_level)
        return;
    *raw_tx_frame_ptr = FRAME_TYPE_LOG;
    *(raw_tx_frame_ptr + 1) = level;
    snprintf((char *)raw_tx_frame_ptr + 2, MAX_RAW_DATA_LEN - 2u, LOG_PREFIX);
    //char *buff = (char *)(raw_tx_frame_ptr + 2);
    va_start(args, fmt);
    status = vsnprintf((char *)(raw_tx_frame_ptr + LOG_PREFIX_LEN + 2u), MAX_RAW_DATA_LEN - (LOG_PREFIX_LEN + 2u), fmt, args);
    va_end(args);
    if (status > 0)
    {
        //Serial.write(raw_tx_frame_ptr, status + 2);
        send_frame((size_t)(status + LOG_PREFIX_LEN + 2u));
    }
}

static void send_sensors_data()
{
    mpu.update();

    *raw_tx_frame_ptr = FRAME_TYPE_DATA;
    FLOAT_TO_BE(mpu.getGyro(0), raw_tx_frame_ptr + OFFSET_GYRO_X);
    FLOAT_TO_BE(mpu.getGyro(1), raw_tx_frame_ptr + OFFSET_GYRO_Y);
    FLOAT_TO_BE(mpu.getGyro(2), raw_tx_frame_ptr + OFFSET_GYRO_Z);

    FLOAT_TO_BE(mpu.getAcc(0), raw_tx_frame_ptr + OFFSET_ACCEL_X);
    FLOAT_TO_BE(mpu.getAcc(1), raw_tx_frame_ptr + OFFSET_ACCEL_Y);
    FLOAT_TO_BE(mpu.getAcc(2), raw_tx_frame_ptr + OFFSET_ACCEL_Z);

    FLOAT_TO_BE(mpu.getMag(0), raw_tx_frame_ptr + OFFSET_MAG_X);
    FLOAT_TO_BE(mpu.getMag(1), raw_tx_frame_ptr + OFFSET_MAG_Y);
    FLOAT_TO_BE(mpu.getMag(2), raw_tx_frame_ptr + OFFSET_MAG_Z);

    serializable_obj.u16 = ads.readADC_Differential_2_3();
    FLOAT_TO_BE(serializable_obj.u16 * ADS_MULTIPLIER, raw_tx_frame_ptr + OFFSET_BAT);

    UINT16_TO_BE(left_motor_tick, raw_tx_frame_ptr + OFFSET_LEFT_TICK);
    UINT16_TO_BE(right_motor_tick, raw_tx_frame_ptr + OFFSET_RIGHT_TICK);

    send_frame(SENSOR_DATA_SIZE);
}

static void left_encoder_event()
{
    if (digitalRead(LM_ENCODER_A) == HIGH)
    {
        if (digitalRead(LM_ENCODER_B) == LOW)
        {
            left_motor_tick++;
        }
        else
        {
            left_motor_tick--;
        }
    }
    else
    {
        if (digitalRead(LM_ENCODER_B) == LOW)
        {
            left_motor_tick--;
        }
        else
        {
            left_motor_tick++;
        }
    }
}

// encoder event for the right
static void right_encoder_event()
{
    if (digitalRead(RM_ENCODER_A) == HIGH)
    {
        if (digitalRead(RM_ENCODER_B) == LOW)
        {
            right_motor_tick++;
        }
        else
        {
            right_motor_tick--;
        }
    }
    else
    {
        if (digitalRead(RM_ENCODER_B) == LOW)
        {
            right_motor_tick--;
        }
        else
        {
            right_motor_tick++;
        }
    }
}

void setup()
{
    raw_tx_frame_ptr = tx_frame_buffer + 1;
    encoded_tx_frame_ptr = tx_frame_buffer;

    raw_rx_frame_ptr = rx_frame_buffer + 1;
    encoded_rx_frame_ptr = rx_frame_buffer;

    rx_counter = 0;

    i = 0;
    for (size_t i = 0; i < MAX_BUFFER_LEN; i++)
    {
        tx_frame_buffer[i] = 0;
        rx_frame_buffer[i] = 0;
    }
    log_level = LOG_INFO;
    Wire.begin();
    Wire.setClock(400000); // 100kHz I2C clock. Comment this line if having compilation difficulties
    Serial.begin(115200);
    // wait for the serial link becomes available
    //while (!Serial.available())
    //    ;
    //delay(5000);
    log(LOG_INFO, "Init system. Firmware version %d.%d", FW_MAJOR, FW_MINOR);
    status = mpu.initialize();
    if (status == MPU_OK)
    {
        log(LOG_INFO, "IMU is initialized. Calibrating accel + gyro. Dont move the robot");
        mpu.calibrateAccelGyro();
    }
    else
    {
        log(LOG_ERR, "Cannot initialize IMU device, status: %d", status);
    }

    log(LOG_INFO, "starting ads1115");
    // init the ads1115
    //                                                                ADS1015  ADS1115
    //                                                                -------  -------
    // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    ads.begin();

    log(LOG_INFO, "starting motor controller");
    // Init the motor
    AFMS.begin();
    left_motor_tick = 0;
    right_motor_tick = 0;
    pinMode(LM_ENCODER_A, INPUT);
    pinMode(LM_ENCODER_B, INPUT);
    pinMode(RM_ENCODER_A, INPUT);
    pinMode(RM_ENCODER_B, INPUT);

    // initialize hardware interrupts
    attachInterrupt(digitalPinToInterrupt(LM_ENCODER_A), left_encoder_event, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RM_ENCODER_A), right_encoder_event, CHANGE);
    log(LOG_INFO, "System initialized");
}

void loop()
{
    i = 0;
    process_cmd();
    send_sensors_data();
    // around 50hz
    delay(5);
}
