"""Deploy I2C test using the checked and backed-up deployment workflow."""
from pathlib import Path

source = Path(__file__).with_name('deploy_soft_uart.py').read_text(encoding='utf-8')
changes = {
    "'/root/yyb_mover-before-softuart-'": "'/root/yyb_mover-before-i2c-'",
    "r'(?m)^version:\\s*3\\.9\\.89\\s*$'": "r'(?m)^version:\\s*3\\.9\\.89-softuart-test\\s*$'",
    "needle = 'self.ser = uart.UART(dev, baud)'": "needle = 'self.ser = SoftUART(dev, baud)'",
    "protocol.replace(needle, 'self.ser = SoftUART(dev, baud)')": "protocol.replace(needle, 'self.ser = I2CTransport(dev, baud)')",
    "protocol.replace('from maix import uart, pinmap',\n                                    'from maix import uart, pinmap\\nfrom soft_uart import SoftUART')": "protocol.replace('from soft_uart import SoftUART', 'from i2c_transport import I2CTransport')",
    "UART_DEV='soft:A1_TX/A0_RX'": "UART_DEV='i2c-gpio-master:0x42'",
    "UART_BAUDRATE=115200)": "UART_BAUDRATE=115200, FRAME_TIMEOUT_MS=600)",
    "'version: 3.9.89-softuart-test'": "'version: 3.9.89-i2c-gpio-test'",
    "yaml += '  - soft_uart.py\\n'": "yaml = re.sub(r'(?m)^  - soft_uart\\.py[ \\t]*\\n', '', yaml) + '  - i2c_transport.py\\n  - gpio_i2c_master.py\\n'",
    "'soft_uart.py': Path(__file__).with_name('soft_uart.py').read_bytes()": "'i2c_transport.py': Path(__file__).with_name('i2c_transport.py').read_bytes(),\n                    'gpio_i2c_master.py': Path(__file__).with_name('gpio_i2c_master.py').read_bytes()",
    "pkill -9 -f '[l]auncher'": "pkill -9 -f '/yyb_mover/[s]oft_uart.py' || true; pkill -9 -f '[l]auncher'",
}
for old, new in changes.items():
    if old not in source:
        raise RuntimeError('Deployment template changed: ' + old)
    source = source.replace(old, new)
exec(compile(source, str(Path(__file__)), 'exec'))
