"""
Balancing robot serial data logger.

Reads CSV lines from the ESP32 over Serial and logs them to a timestamped
CSV file. Optionally live-plots pitch angle and PID terms.

Expects each line from the robot as:
    timestamp_ms,roll,pitch,setpoint,P,I,D,motor_output

Install dependencies:
    pip install pyserial matplotlib pandas

Usage:
    python robot_logger.py --port COM5              # Windows
    python robot_logger.py --port /dev/ttyUSB0       # Linux
    python robot_logger.py --port /dev/cu.usbserial* # macOS
    python robot_logger.py --port COM5 --plot        # with live plot
    python robot_logger.py --list                    # list available ports
"""

import argparse
import csv
import sys
import time
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports

# Single source of truth for column names. Used both as the CSV header row
# AND to check that every incoming line has exactly 8 fields before we
# trust it (see parse_line below).
COLUMNS = ["timestamp_ms", "roll", "pitch",
           "setpoint", "P", "I", "D", "motor_output"]


def list_ports():
    """Print every serial device the OS currently sees, so you can figure
    out which one is your ESP32 without guessing (run with --list)."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available ports:")
    for p in ports:
        print(f"  {p.device} — {p.description}")


def parse_line(line):
    """
    Turn one raw text line from the robot into a list of 8 floats.

    Returns None (instead of raising an error) if the line is malformed --
    e.g. it got cut off mid-transmission, or a Serial.print() from
    handleSerialTuning() snuck in. This is the safety net that lets the
    rest of the script just skip garbage instead of crashing on it.
    """
    parts = line.strip().split(",")  # split "120,0.5,-1.2,..." into pieces
    if len(parts) != len(COLUMNS):   # not exactly 8 pieces? not a real line.
        return None
    try:
        # try converting every piece to a number
        return [float(x) for x in parts]
    except ValueError:
        return None  # one of the pieces wasn't a valid number -> reject the whole line


def run_logger(port, baud, outfile, use_plot, max_points=500):
    # Open the serial "hose" to the robot. baud MUST match Serial.begin()
    # on the Arduino side (57600 here) or you'll just get garbled bytes.
    # timeout=1 means any read gives up after 1 second of silence instead
    # of freezing the script forever if the robot stops sending.
    ser = serial.Serial(port, baud, timeout=1)

    # Plugging in over USB resets most ESP32 boards, so the first couple
    # seconds of "data" are actually boot noise. Wait it out...
    time.sleep(2)
    # ...then throw away whatever garbage piled up in the buffer during
    # that reset, so the first line we actually read is clean real data.
    ser.reset_input_buffer()

    print(f"Logging {port} @ {baud} baud -> {outfile}")
    print("Press Ctrl+C to stop.\n")

    # open() is a FUNCTION (lowercase) that does setup work and then
    # constructs + returns a file object for you -- you're not calling a
    # class directly, but f is still a real object underneath
    # (type(f) is TextIOWrapper). newline="" avoids extra blank lines on
    # Windows -- a small quirk of the csv module, always include it.
    f = open(outfile, "w", newline="")

    # writer is a SEPARATE object that wraps f. It doesn't write to disk
    # itself -- it knows how to correctly format a Python list into a
    # comma-separated row (handling edge cases like values that contain
    # commas), then hands the finished text off to f to actually write.
    # f = the connection to the file. writer = the formatter in front of it.
    writer = csv.writer(f)
    writer.writerow(COLUMNS)  # write the header row once, up front

    if use_plot:
        run_with_plot(ser, writer, f, max_points)
    else:
        run_headless(ser, writer, f)


def run_headless(ser, writer, f):
    """Plain logging: no plot window, just write rows to the CSV as they
    arrive, with a short progress line printed every 50 rows."""
    count = 0
    try:
        while True:
            # readline() blocks until it sees a newline byte or times out.
            # It returns raw BYTES (not text) -- serial doesn't know or
            # care whether you're sending text, so we have to explicitly
            # decode bytes -> string ourselves before we can .split(",") it.
            # errors="ignore" means a corrupted byte gets dropped instead
            # of crashing the decode.
            raw = ser.readline().decode("utf-8", errors="ignore")

            if not raw:
                continue  # nothing arrived before the timeout -- try again

            row = parse_line(raw)
            if row is None:
                # continue jumps straight back to "while True:" above --
                # it does NOT restart parsing this same line, it moves
                # forward and tries reading the NEXT line instead. Every
                # line below this is skipped for this one bad reading.
                continue

            writer.writerow(row)  # write this reading as one CSV row
            count += 1

            # % is the modulo (remainder) operator -- this is True every
            # 50th row. We force a flush() here because Python normally
            # buffers file writes in memory for speed; without flush(),
            # a crash could lose data that "looks written" but isn't
            # actually on disk yet.
            if count % 50 == 0:
                f.flush()
                print(f"  {count} rows logged | pitch={row[2]:.2f}  "
                      f"P={row[4]:.1f} I={row[5]:.1f} D={row[6]:.1f}")
    except KeyboardInterrupt:
        # Catches Ctrl+C specifically so stopping the script gives a clean
        # message instead of a scary traceback.
        print(f"\nStopped. {count} rows written.")
    finally:
        # finally: runs no matter how the loop ends (Ctrl+C, error, or
        # otherwise) -- guarantees the file and port always close cleanly,
        # so the CSV never gets left in a half-written/corrupted state.
        f.close()
        ser.close()


def run_with_plot(ser, writer, f, max_points):
    """Same logging as run_headless, but also pops up a live-updating
    chart of pitch angle and PID terms while it logs."""

    # Imported here (not at the top of the file) so headless mode never
    # has to load matplotlib at all -- keeps that path lighter/faster.
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    # Five "sliding window" buffers. deque(maxlen=N) behaves like a list,
    # except once it holds N items, adding a new one silently drops the
    # OLDEST one -- so the plot always shows just the most recent
    # max_points readings (~50 seconds at 10Hz) instead of growing and
    # slowing down forever.
    #
    # IMPORTANT: all five get appended to together, once per reading, in
    # update() below. That's what keeps them aligned -- t_buf[i] and
    # pitch_buf[i] always describe the exact same moment, because they're
    # never appended separately.
    t_buf = deque(maxlen=max_points)
    pitch_buf = deque(maxlen=max_points)
    p_buf = deque(maxlen=max_points)
    i_buf = deque(maxlen=max_points)
    d_buf = deque(maxlen=max_points)

    # fig = the whole window/canvas (one Figure object).
    # ax1, ax2 = the two individual charts living inside it (two Axes
    # objects), unpacked from a 2x1 grid -- same "unpack a list of known
    # length" pattern as ts_ms, roll, ... = row further down.
    # sharex=True links their x-axes together so scrolling/zooming one
    # moves the other with it -- both are plotting the same time axis.
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    # Create empty line objects up front ([], [] = no data yet). We hold
    # onto these references so update() can just swap in new data later
    # (fast) instead of redrawing the whole chart from scratch every time.
    # Note the comma: ax1.plot() always returns a LIST of lines (it
    # supports plotting several at once), so "line_pitch," unpacks that
    # single-item list into just the one variable we want.
    (line_pitch,) = ax1.plot([], [], label="Pitch angle")
    (line_setpoint,) = ax1.plot([], [], "--", color="gray", label="Setpoint")
    ax1.set_ylabel("Angle (deg)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    (line_p,) = ax2.plot([], [], label="P")
    (line_i,) = ax2.plot([], [], label="I")
    (line_d,) = ax2.plot([], [], label="D")
    ax2.set_ylabel("PID terms")
    ax2.set_xlabel("Time (s)")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    # A dict instead of a plain variable for `count`: update() is a nested
    # function, and Python won't let a nested function reassign a plain
    # outer variable without extra syntax -- mutating a dict's contents
    # sidesteps that restriction.
    state = {"count": 0, "setpoint_buf": deque(maxlen=max_points)}

    # t0 will be set to the FIRST timestamp we ever see. The robot's
    # millis() counts milliseconds since IT booted, not since we started
    # logging -- so without this, the x-axis would start at some ugly
    # arbitrary number (e.g. 84213) instead of 0.
    t0 = None

    def update(_frame):
        """Called automatically by FuncAnimation every interval=100ms.
        This is the whole "live" part of the live plot."""
        nonlocal t0  # lets this inner function modify the outer t0 variable

        # in_waiting tells us how many bytes are sitting in the buffer,
        # unread, right now. We drain ALL of them each time update() runs
        # (not just one line) -- if a redraw ever takes a bit longer than
        # 100ms, this catches the plot back up instead of letting it
        # permanently fall behind the robot's actual data.
        while ser.in_waiting:
            raw = ser.readline().decode("utf-8", errors="ignore")
            row = parse_line(raw)
            if row is None:
                continue
            writer.writerow(row)  # still log to CSV even in plot mode
            state["count"] += 1
            if state["count"] % 50 == 0:
                f.flush()

            # Unpack the 8 numbers into named variables in one line --
            # equivalent to row[0], row[1], row[2] ... but far more
            # readable, and Python errors out if the count doesn't match.
            ts_ms, roll, pitch, setpoint, P, I, D, out = row

            if t0 is None:
                t0 = ts_ms  # lock the anchor point on the very first reading

            # (ts_ms - t0) = "milliseconds since logging started" instead
            # of "milliseconds since the robot booted". /1000.0 converts
            # that to seconds, which is a friendlier axis unit.
            t_buf.append((ts_ms - t0) / 1000.0)
            pitch_buf.append(pitch)
            state["setpoint_buf"].append(setpoint)
            p_buf.append(P)
            i_buf.append(I)
            d_buf.append(D)

        if t_buf:
            # set_data() swaps in new x/y values on the EXISTING line
            # object -- much cheaper than erasing and redrawing everything.
            line_pitch.set_data(t_buf, pitch_buf)
            line_setpoint.set_data(t_buf, state["setpoint_buf"])
            line_p.set_data(t_buf, p_buf)
            line_i.set_data(t_buf, i_buf)
            line_d.set_data(t_buf, d_buf)
            for ax in (ax1, ax2):
                ax.relim()            # recompute what the data's min/max actually are
                ax.autoscale_view()   # apply that new range to the visible axes
        return line_pitch, line_setpoint, line_p, line_i, line_d

    # This is the "timer" -- it calls update() every interval=100
    # milliseconds for as long as the plot window stays open. Nothing
    # above this line actually runs yet; it's all just setup.
    ani = FuncAnimation(fig, update, interval=100,
                        blit=False, cache_frame_data=False)
    try:
        plt.tight_layout()
        plt.show()  # opens the window and starts the animation loop; blocks until closed
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nStopped. {state['count']} rows written.")
        f.close()
        ser.close()


def main():
    parser = argparse.ArgumentParser(
        description="Balancing robot serial data logger")
    parser.add_argument(
        "--port", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=57600,
                        help="Baud rate (default 57600, matches Serial.begin)")
    parser.add_argument("--out", default=None, help="Output CSV filename")
    parser.add_argument("--plot", action="store_true",
                        help="Show live plot while logging")
    parser.add_argument("--list", action="store_true",
                        help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        print("Error: --port is required (use --list to see available ports)")
        sys.exit(1)

    outfile = args.out or f"robot_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    run_logger(args.port, args.baud, outfile, args.plot)


# Only run this program if I launched it directly. Do not run it if I am just borrowing code from it.
if __name__ == "__main__":
    main()
