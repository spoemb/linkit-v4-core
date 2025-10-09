#ifndef SMD_SAT_REGISTER_HPP
#define SMD_SAT_REGISTER_HPP

#ifndef DEFAULT_TCXO_WARMUP_TIME_SECONDS
#define DEFAULT_TCXO_WARMUP_TIME_SECONDS 2
#endif

#define SMDSAT_IS_DEBUG_EN

#ifdef SMDSAT_IS_DEBUG_EN
#define SMDSAT_DELAY_POWER_ON_MS (1000)
#define SMDSAT_DELAY_LOAD_KMAC_MS (1000)
#define SMDSAT_DELAY_TICK_INTERRUPT_MS (10)
#define SMDSAT_DELAY_CMD_MS (400)
#define SMDSAT_DELAY_CMD_TX (1000)
#define SMDSAT_DELAY_RST_MS (100)
#else
#define SMDSAT_DELAY_POWER_ON_MS (200)
#define SMDSAT_DELAY_LOAD_KMAC_MS (400)
#define SMDSAT_DELAY_TICK_INTERRUPT_MS (10)
#define SMDSAT_DELAY_CMD_MS (200)
#define SMDSAT_DELAY_CMD_TX (1000)
#define SMDSAT_DELAY_RST_MS (5)

#endif
#define TX_FREQUENCY_ARGOS_2_3_BAND_START		401.62

#define SMDSAT_CMD_WRITERCONF_LEN 33 // 32 conf size + 1 cmd
#define SMDSAT_CMD_WRITELPM_LEN 2    // 1 LPM mode + 1 cmd
#define SMDSAT_CMD_WRITEKMAC_LEN 2    // 1 write only ID for the moment + 1 cmd
#define SMDSAT_CMD_WRITETX_LEN 3    // 1 write only ID for the moment + 2 datasize (u16)
#define SMDSAT_CMD_WRITECONF_LEN 17    // 1 write only ID for the moment + 2 datasize (u16)
#define SMDSAT_CMD_READCONF_LEN 13    // 1 write only ID for the moment + 2 datasize (u16)
#define SMDSAT_CMD_READ_FIRMWARE_LEN 128
#define SMDSAT_CMD_READ_SERIAL_LEN 16
#define SMDSAT_CMD_WRITE_LPM_LEN 2
#define SMDSAT_CMD_WRITE_ID_LEN 5
#define SMDSAT_CMD_WRITE_TCXO_LEN 5
#define SMDSAT_CMD_READ_ADDR_LEN 4
#define SMDSAT_CMD_READ_ID_LEN 4
#define SMDSAT_CMD_WRITE_ADDR_LEN (SMDSAT_CMD_READ_ADDR_LEN+1)
#define SMDSAT_CMD_READ_SECKEY_LEN 16
#define SMDSAT_CMD_WRITE_SECKEY_LEN (SMDSAT_CMD_READ_SECKEY_LEN+1)
typedef enum {
    ARGOS_MOD_LDA2,
    ARGOS_MOD_LDA2L,
    ARGOS_MOD_LDK,
    ARGOS_MOD_VLDA4
} ArgosModulation;

typedef enum {
    MAC_OK            = 0x01,
    MAC_TX_DONE       = 0x02,
    MAC_TX_SIZE_ERROR = 0x03,
    MAC_TXACK_DONE    = 0x04,
    MAC_TX_TIMEOUT    = 0x05,
    MAC_TXACK_TIMEOUT = 0x06,
    MAC_RX_ERROR      = 0x07,
    MAC_RX_TIMEOUT    = 0x08,
    MAC_ERROR         = 0x09
} SMDSAT_MACSTATUS;

typedef enum {
    SMDSAT_SPICMD_INIT,                // Register SPI command manager
    SMDSAT_SPICMD_IDLE,                // Idle waiting TX request
    SMDSAT_SPICMD_PROCESS_CMD,         // Process incoming command
    SMDSAT_SPICMD_WAITING_RX,          // Waiting for RX data with specified length
    SMDSAT_SPICMD_WAITING_TX,          // Waiting for TX data to be sent
    SMDSAT_SPICMD_WAITING_MAC_EVT,     // Waiting for MAC event (e.g., TX done, RX done)
    SMDSAT_SPICMD_ERROR,                // Error state
    SMDSAT_SPICMD_STATUS_MAX_LEN
} SMDSAT_SPISTATUS;


typedef enum {
	SMDSAT_CMD_NONE           = 0x00,  // Cmd unknoiwn
    SMDSAT_CMD_READ           = 0x01,  // Ping command
    SMDSAT_CMD_PING           = 0x02,  // Ping command
    SMDSAT_CMD_MAC_STATUS     = 0x03,  // Read MAC status
    SMDSAT_CMD_SPI_STATUS     = 0x04,  // Read SPI status
    SMDSAT_CMD_READ_VERSION   = 0x05,  // Read version
    SMDSAT_CMD_READ_FIRMWARE  = 0x06,  // Read firmware
    SMDSAT_CMD_READ_ADDR      = 0x07,  // Read address
    SMDSAT_CMD_READ_ID        = 0x08,  // Read ID
    SMDSAT_CMD_READ_SN        = 0x09,  // Read serial number
    SMDSAT_CMD_READ_RCONF     = 0x0A,  // Read configuration
    SMDSAT_CMD_WRITE_RCONF_REQ= 0x0B,  // Write configuration
    SMDSAT_CMD_WRITE_RCONF    = 0x0C,  // Write configuration
    SMDSAT_CMD_SAVE_RCONF     = 0x0D,  // Save configuration
    SMDSAT_CMD_READ_KMAC      = 0x0E,  // Reload configuration
    SMDSAT_CMD_WRITE_KMAC_REQ = 0x0F,  // Reload configuration
    SMDSAT_CMD_WRITE_KMAC     = 0x10,  // Reload configuration
    SMDSAT_CMD_READ_LPM       = 0x11,  // Read low power mode
    SMDSAT_CMD_WRITE_LPM_REQ  = 0x12,  // Set low power mode
    SMDSAT_CMD_WRITE_LPM      = 0x13,  // Set low power mode
    SMDSAT_CMD_WRITE_TX_REQ   = 0x14,  // Read low power mode
    SMDSAT_CMD_WRITE_TX_SIZE  = 0x15,  // Read low power mode
    SMDSAT_CMD_WRITE_TX       = 0x16,  // Set low power mode
	SMDSAT_CMD_READ_CW        = 0x17,  // Set low power mode
	SMDSAT_CMD_WRITE_CW_REQ   = 0x18,  // Read low power mode
	SMDSAT_CMD_WRITE_CW       = 0x19,  // Set low power mode
	SMDSAT_CMD_READ_PREPASSEN        = 0x1A,  // Set low power mode
	SMDSAT_CMD_WRITE_PREPASSEN_REQ   = 0x1B,  // Read low power mode
	SMDSAT_CMD_WRITE_PREPASSEN       = 0x1C,  // Set low power mode
	SMDSAT_CMD_READ_UDATE        = 0x1D,  // Set low power mode
	SMDSAT_CMD_WRITE_UDATE_REQ   = 0x1E,  // Read low power mode
	SMDSAT_CMD_WRITE_UDATE       = 0x1F,  // Set low power mode
    SMDSAT_CMD_WRITE_ID_REQ          = 0x20,  // Write ID Request
    SMDSAT_CMD_WRITE_ID              = 0x21,  // Write ID value
    SMDSAT_CMD_WRITE_ADDR_REQ        = 0x22,  // Write addres REquest
    SMDSAT_CMD_WRITE_ADDR            = 0x23,  // Write Address Value
	SMDSAT_CMD_READ_SECKEY           = 0x24,  // Read Secret key
	SMDSAT_CMD_WRITE_SECKEY_REQ      = 0x25,  // Write Secret key request
	SMDSAT_CMD_WRITE_SECKEY		  = 0x26,  // Write Secret key value
	SMDSAT_CMD_READ_SPIMAC_STATE	  = 0x27,  // Write Secret key value
	SMDSAT_CMD_READ_TCXO  	         = 0x28,  // Write Secret key value
    SMDSAT_CMD_WRITE_TCXO_REQ         = 0x29,  // Write ID Request
    SMDSAT_CMD_WRITE_TCXO              = 0x2A,  // Write ID value
	SMDSAT_SPICMD_MAX_COUNT          = 0x2B
} SMDSAT_SPICMD;

#define ARGOS_TX_LDA2_PAYLOAD_BYTE_SIZE 24
#define ARGOS_TX_VLDA4_PAYLOAD_BYTE_SIZE 3
#define ARGOS_TX_LDK_PAYLOAD_BYTE_SIZE 19

#endif
