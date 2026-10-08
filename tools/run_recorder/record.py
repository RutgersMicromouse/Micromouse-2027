"""
Records everything the robot says, over Bluetooth, into a text file on this computer.

The robot copies all of its output (the same text as the serial monitor and the phone app's
"Robot output" panel) to Bluetooth. This program listens to that and writes it to
logs/run_<date>_<time>.log in the project folder, one line at a time with the time in front.
The computer can stay on its normal Wi-Fi and the phone app keeps working as usual.

    pip install bleak
    python tools/run_recorder/record.py              record until Ctrl+C
    python tools/run_recorder/record.py --stream     also switch on the 5-per-second sensor line
    python tools/run_recorder/record.py status ir    send these commands once connected

While it is running, any line added to logs/send.txt is sent to the robot as a console command
(for example:  echo status >> logs/send.txt).

Leave it running while you drive the robot. If the robot is switched off or goes out of range
it waits and reconnects by itself. Only one program can be connected to the robot's Bluetooth
at a time, so close the web dashboard's Bluetooth connection first.
"""
import argparse
import asyncio
import datetime
import os

from bleak import BleakClient, BleakScanner

ROBOT_NAME = 'Antigrav-Mouse'                       # BLE_DEVICE_NAME in src/config.h
UART_RX = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'    # We write commands here
UART_TX = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'    # The robot's output arrives here

LOG_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'logs')
SEND_FILE = os.path.join(LOG_FOLDER, 'send.txt')    # Lines added here are sent to the robot


def clock():
    return datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]


class Recorder:
    def __init__(self):
        os.makedirs(LOG_FOLDER, exist_ok=True)
        name = datetime.datetime.now().strftime('run_%Y-%m-%d_%H-%M-%S.log')
        self.path = os.path.normpath(os.path.join(LOG_FOLDER, name))
        self.file = open(self.path, 'a', encoding='utf-8', buffering=1)  # Written line by line
        self.partial = ''

    def note(self, text):
        """A line from this program rather than from the robot."""
        line = '%s  # %s' % (clock(), text)
        print(line)
        self.file.write(line + '\n')

    def received(self, _sender, data):
        """Bluetooth delivers the text in pieces; write it out whole line by whole line."""
        self.partial += bytes(data).decode('utf-8', errors='replace')
        while '\n' in self.partial:
            line, self.partial = self.partial.split('\n', 1)
            line = line.rstrip('\r')
            if line:
                stamped = '%s  %s' % (clock(), line)
                print(stamped)
                self.file.write(stamped + '\n')


async def record(commands):
    recorder = Recorder()
    recorder.note('Recording to ' + recorder.path)
    first_connection = True

    while True:
        recorder.note('Looking for "%s" on Bluetooth...' % ROBOT_NAME)
        robot = await BleakScanner.find_device_by_name(ROBOT_NAME, timeout=10.0)
        if robot is None:
            await asyncio.sleep(2.0)
            continue

        gone = asyncio.Event()
        try:
            async with BleakClient(robot, disconnected_callback=lambda _client: gone.set()) as link:
                recorder.note('Connected.')
                await link.start_notify(UART_TX, recorder.received)
                for command in (commands if first_connection else []):
                    recorder.note('Sending: ' + command)
                    await link.write_gatt_char(UART_RX, (command + '\n').encode(), response=False)
                    await asyncio.sleep(0.5)
                first_connection = False

                # Stay connected, passing on any command lines that appear in logs/send.txt
                open(SEND_FILE, 'a').close()
                sent = os.path.getsize(SEND_FILE)
                while not gone.is_set():
                    await asyncio.sleep(0.3)
                    if os.path.getsize(SEND_FILE) < sent:
                        sent = 0  # The file was emptied
                    with open(SEND_FILE, 'rb') as waiting:
                        waiting.seek(sent)
                        new_bytes = waiting.read()
                    if not new_bytes.endswith(b'\n'):
                        continue  # Nothing new, or a line is still being written
                    sent += len(new_bytes)
                    for command in new_bytes.decode('utf-8', errors='replace').splitlines():
                        command = command.strip()
                        if command:
                            recorder.note('Sending: ' + command)
                            await link.write_gatt_char(UART_RX, (command + '\n').encode(), response=False)
                            await asyncio.sleep(0.5)
        except Exception as problem:  # Out of range, switched off, or already connected elsewhere
            recorder.note('Connection problem: %s' % problem)
        recorder.note('Disconnected.')
        await asyncio.sleep(2.0)


def main():
    parser = argparse.ArgumentParser(description='Record the robot\'s output over Bluetooth.')
    parser.add_argument('commands', nargs='*', help='console commands to send once connected, e.g. status ir')
    parser.add_argument('--stream', action='store_true', help='switch on the 5-per-second [TEL] sensor line')
    args = parser.parse_args()
    commands = (['stream on'] if args.stream else []) + args.commands
    try:
        asyncio.run(record(commands))
    except KeyboardInterrupt:
        print('Stopped.')


if __name__ == '__main__':
    main()
