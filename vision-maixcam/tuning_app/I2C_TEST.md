# MaixCAM I2C test transport

- Version: `3.9.89-i2c-gpio-test`, based on the deployed 3.9.89 vision code.
- MaixCAM is a GPIO open-drain master. STM32 must be slave at **7-bit 0x42** (HAL shifted address 0x84).
- A1=SCL, A0=SDA, common GND. Use 3.3V pull-ups on both lines; typically 4.7k ohm, accounting for existing pull-ups. This is not crossed UART wiring.
- The GPIO backend caps SCL at 10 kHz. Linux scheduling can slow it further.

## Mailbox contract

Every master write and master read transfers exactly **32 bytes**:

| Offset | Contents |
| --- | --- |
| 0 | Valid stream byte count, 0..31 |
| 1..N | N bytes from the existing AA55/LEN/payload/CRC8/0D protocol |
| N+1..31 | Zero padding, ignored |

Master writes contain vision replies. STM32 appends only the valid bytes to its existing frame parser. Master reads fetch STM32 commands. STM32 serves queued stream bytes and must return 32 zero bytes when empty. Frame boundaries need not match transaction boundaries. There is **no register-address byte** and no SMBus count byte beyond this mailbox length.

Prepare the STM32 slave receive and transmit buffers before transactions. Do not wait for image recognition inside I2C callbacks. When STM32 sends R in its read mailbox, MaixCAM should parse R and enqueue a framed READY reply. Consume the startup READY already queued before evaluating the R response.

The worker schedules reads 50 ms after an empty mailbox, 20 ms after receiving data, and 100 ms after a read error. Intervals begin after the bus operation completes. Queued writes run between reads. A full RX queue pauses reads until the application makes space. Malformed mailbox packets are counted and the worker resumes reading after backoff. Existing TX/READY logs show enqueue, while `[I2C] TX transaction delivered` confirms that the GPIO transaction completed. Retries after a write error may repeat a packet if the peer received it before the master detected the error.

## Initial test

1. Configure STM32 slave 0x42 with this fixed-size mailbox and connect SCL/SDA/GND.
2. Check that Errno 121 stops and startup READY is delivered.
3. Queue an original framed R command; check RX R and a new READY response.
4. Only then exercise vision commands.

Backup before this change: `/root/yyb_mover-before-i2c-20261029-201352.tar.gz`.
