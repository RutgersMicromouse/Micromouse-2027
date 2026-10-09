"""
Puts a question on the robot's phone app, with Yes and No buttons.

    python tools/run_recorder/ask.py "Lower the curve speed from 140 to 120 mm/s, because ..."

The recorder (record.py) must be running and connected: this only adds lines to logs/send.txt,
which the recorder passes to the robot over Bluetooth. The robot takes short command lines, one
at a time, so the text goes over in pieces with a pause between them. The answer comes back in
the recording as a line starting with [ANSWER].
"""
import os
import sys
import time

SEND_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'logs', 'send.txt')
PIECE = 110     # Characters per line: well inside what the robot's Bluetooth link takes at once
PAUSE_S = 0.9   # The recorder looks at the file every 0.3 s and the robot holds one command at a time


def pieces(text):
    """The text cut at spaces into pieces no longer than PIECE."""
    line = ''
    for word in text.split():
        if line and len(line) + 1 + len(word) > PIECE:
            yield line
            line = word
        else:
            line = (line + ' ' + word).strip()
    if line:
        yield line


def send(line):
    with open(SEND_FILE, 'a', encoding='utf-8') as out:
        out.write(line + '\n')
    time.sleep(PAUSE_S)


def main():
    text = ' '.join(sys.argv[1:]).strip()
    if not text:
        sys.exit(__doc__)
    if text == '--clear':
        send('ask clear')
        return
    send('ask new')
    for piece in pieces(text[:690]):
        send('ask+ ' + piece)
    send('ask show')
    print('Question sent (%d characters). Watch the recording for [ASK] and [ANSWER].' % len(text[:690]))


if __name__ == '__main__':
    main()
