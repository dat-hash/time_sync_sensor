#include "gnss_ubx.h"
#include "gnss_uart.h"
#include "main.h"

#define TARGET_BAUD_RATE 115200u
#define MEASUREMENT_PERIOD_MS 100u // 10hz

// try 38400 is default and 115200 is wanted
static const uint32_t candidate_baud_rates[2] = { 38400u ,115200u };   /* TEMPORARY: test 115200 only */
#define CANDIDATE_BAUD_RATE_COUNT (sizeof(candidate_baud_rates) / sizeof(candidate_baud_rates[0]))

#define ACK_TIMEOUT_MS 300u
#define ATTEMPTS_PER_BAUD_RATE 3u

// config keys
#define KEY_UART1_BAUD_RATE 0x40520001u
#define KEY_MEASUREMENT_RATE_MS 0x30210001u
#define KEY_NMEA_RMC_ON_UART1 0x209100ACu
#define KEY_NMEA_GGA_ON_UART1 0x209100BBu
#define KEY_NMEA_GSA_ON_UART1 0x209100C0u
#define KEY_NMEA_GSV_ON_UART1 0x209100C5u
#define KEY_NMEA_GLL_ON_UART1 0x209100CAu
#define KEY_NMEA_VTG_ON_UART1 0x209100B1u

#define LAYER_RAM                       0x01u

#define UBX_SYNC_BYTE_1 0xB5u
#define UBX_SYNC_BYTE_2 0x62u
#define UBX_CLASS_CONFIGURATION 0x06u
#define UBX_ID_CONFIGURATION_SET_VALUES 0x8Au

static uint32_t baud_rate_found = 0u;

// build frame (little endian)
static uint16_t append_key_value(uint8_t *payload, uint16_t length, uint32_t key, uint32_t value, uint8_t value_size_bytes)
{
    // put in key to payload
    for (uint8_t byte_index = 0u; byte_index < 4u; byte_index++) {
        payload[length] = (uint8_t)(key >> (8u * byte_index));
        length = length + 1u;
    }
    // put in value to playload
    for (uint8_t byte_index = 0u; byte_index < value_size_bytes; byte_index++) {
        payload[length] = (uint8_t)(value >> (8u * byte_index));
        length = length + 1u;
    }
    return length;
}

// sync, class, id, length, payload, checksum.
static uint16_t build_frame(uint8_t *frame, uint16_t payload_length)
{
    // the frame already have payload in the right place
    frame[0] = UBX_SYNC_BYTE_1;
    frame[1] = UBX_SYNC_BYTE_2;
    frame[2] = UBX_CLASS_CONFIGURATION;
    frame[3] = UBX_ID_CONFIGURATION_SET_VALUES;
    frame[4] = (uint8_t)(payload_length);
    frame[5] = (uint8_t)(payload_length >> 8);

    uint8_t checksum_a = 0u;
    uint8_t checksum_b = 0u;
    uint16_t checksum_end = (uint16_t)(6u + payload_length);

    // checksum is after 2sync bytes till end (fletcher style)
    for (uint16_t index = 2u; index < checksum_end; index++) {
        checksum_a = (uint8_t)(checksum_a + frame[index]);
        checksum_b = (uint8_t)(checksum_b + checksum_a);
    }

    frame[checksum_end] = checksum_a;
    frame[checksum_end + 1u] = checksum_b;
    return (uint16_t)(checksum_end + 2u); // return frame lenght
}

// build config
static uint16_t build_main_configuration_frame(uint8_t *frame)
{
    uint8_t *payload = &frame[6];
    payload[0] = 0x00u;
    payload[1] = LAYER_RAM;
    payload[2] = 0x00u;
    payload[3] = 0x00u;

    uint16_t length = 4u;

    length = append_key_value(payload, length, KEY_MEASUREMENT_RATE_MS, MEASUREMENT_PERIOD_MS, 2u);
    length = append_key_value(payload, length, KEY_NMEA_RMC_ON_UART1, 1u, 1u); // on
    length = append_key_value(payload, length, KEY_NMEA_GGA_ON_UART1, 1u, 1u); // on
    length = append_key_value(payload, length, KEY_NMEA_GSA_ON_UART1, 0u, 1u); // off
    length = append_key_value(payload, length, KEY_NMEA_GSV_ON_UART1, 0u, 1u); // off
    length = append_key_value(payload, length, KEY_NMEA_GLL_ON_UART1, 0u, 1u); // off
    length = append_key_value(payload, length, KEY_NMEA_VTG_ON_UART1, 0u, 1u); // off

    return build_frame(frame, length);
}

// build baud
static uint16_t build_baud_rate_frame(uint8_t *frame, uint32_t new_baud_rate)
{
    uint8_t *payload = &frame[6];
    payload[0] = 0x00u;
    payload[1] = LAYER_RAM;
    payload[2] = 0x00u;
    payload[3] = 0x00u;

    uint16_t length = 4u;

    length = append_key_value(payload, length, KEY_UART1_BAUD_RATE, new_baud_rate, 4u);

    return build_frame(frame, length);
}

// ACK NAK PAYLOAD
static const uint8_t acknowledged_frame[10] = { 0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8A, 0x98, 0xC1 };
static const uint8_t rejected_frame[10] = { 0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x06, 0x8A, 0x97, 0xBC };

#define ANSWER_FRAME_LENGTH 10u

typedef enum
{
    ANSWER_NONE,
    ANSWER_ACKNOWLEDGED,
    ANSWER_REJECTED
} receiver_answer_t;

static bool frames_are_equal(const uint8_t *first, const uint8_t *second)
{
    for (uint32_t index = 0u; index < ANSWER_FRAME_LENGTH; index++) {
        if (first[index] != second[index]) {
            return false;
        }
    }
    return true;
}

static receiver_answer_t wait_for_answer(uint32_t timeout_ms)
{
    uint8_t  received[64];
    uint8_t  collected_frame[ANSWER_FRAME_LENGTH];
    uint32_t collected_count = 0u;
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms)
    {
        uint32_t count = gnss_uart_read_new_bytes(received, sizeof(received));

        for (uint32_t index = 0u; index < count; index++)
        {
            uint8_t byte = received[index];

            if (collected_count == 0u) {
                if (byte == UBX_SYNC_BYTE_1) {
                    collected_frame[0] = byte;
                    collected_count = 1u;
                }
            }
            else if (collected_count == 1u && byte != UBX_SYNC_BYTE_2) {
                // no full sync
                if (byte == UBX_SYNC_BYTE_1){
                    collected_count = 1u;
                }
                else {
                    collected_count = 0u;
                }
            }
            else {
                // start compare
                collected_frame[collected_count] = byte;
                collected_count = collected_count + 1u;

                if (collected_count == ANSWER_FRAME_LENGTH) {
                    // 10 bytes collected: is it ACK or NAK?
                    if (frames_are_equal(collected_frame, acknowledged_frame)) {
                        return ANSWER_ACKNOWLEDGED;
                    }
                    else if (frames_are_equal(collected_frame, rejected_frame)) {
                        return ANSWER_REJECTED;
                    }
                    else {
                        collected_count = 0u; // not ack nak ubx
                    }
                }
            }
        }
    }
    return ANSWER_NONE; // timeout
}

static receiver_answer_t send_and_wait(const uint8_t *frame, uint16_t frame_length)
{
    receiver_answer_t answer = ANSWER_NONE;

    for (uint32_t attempt = 0u; attempt < ATTEMPTS_PER_BAUD_RATE; attempt++) {
        gnss_uart_send(frame, frame_length);
        answer = wait_for_answer(ACK_TIMEOUT_MS);

        if (answer != ANSWER_NONE) {
            break; // got ans
        }
    }
    return answer;
}

// in main
bool gnss_ubx_configure_receiver(void)
{
    uint8_t frame[64];
    uint16_t main_frame_length = build_main_configuration_frame(frame);
    uint8_t main_config_frame[64];

    for (uint16_t index = 0u; index < main_frame_length; index++) {
        main_config_frame[index] = frame[index];
    }

    baud_rate_found = 0u;

    for (uint32_t baud_index = 0u; baud_index < CANDIDATE_BAUD_RATE_COUNT; baud_index++)
    {
        uint32_t baud_rate = candidate_baud_rates[baud_index];

        if (!gnss_uart_set_baud_rate(baud_rate)) {
            continue;
        }

        // is the receiver at this baud, send settings and see
        receiver_answer_t answer = send_and_wait(main_config_frame, main_frame_length);

        if (answer == ANSWER_NONE) {
            continue; // silent move on to other baud
        }
        else if (answer == ANSWER_REJECTED) {
            baud_rate_found = baud_rate;
            return false; // something wrong with the key
        }
        else {
            baud_rate_found = baud_rate;
        }

        if (baud_rate == TARGET_BAUD_RATE) {
            return true; // already at where we want
        }
        else { // need to push to 115200
            uint16_t baud_frame_length = build_baud_rate_frame(frame, TARGET_BAUD_RATE);
            gnss_uart_send(frame, baud_frame_length);
            HAL_Delay(50u);

            if (!gnss_uart_set_baud_rate(TARGET_BAUD_RATE)) {
                return false;
            }
            else
            {
                // confirm
                if (send_and_wait(main_config_frame, main_frame_length) == ANSWER_ACKNOWLEDGED) {
                    baud_rate_found = TARGET_BAUD_RATE;
                    return true;
                }
                else {
                    return false; // something gone wrong
                }
            }
        }
    }
    return false; // no ans anywhere weird
}

// debug
uint32_t gnss_ubx_get_baud_rate_found(void)
{
    return baud_rate_found;
}
