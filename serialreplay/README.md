# Serial recorder and replayer

This directory includes a __work in progress__ serial recorder and replayer for testing and developing OScar in the lab, without the need of the connection with an actual serial GNSS receiver.

The scripts available here let you:
- Record the output of a serial GNSS receiver while driving (i.e., NMEA sentences and other binary data such as UBX messages for U-blox receivers), with microsecond timestamps, and save it in a log file:
    ```
    sudo ./serialrecord.sh <LOG FILE NAME> <SERIAL DEVICE TO RECORD FROM> <BAUDRATE>
    ```
    Example:
    ```
    sudo ./serialrecord.sh outlog.seriallog /dev/ttyACM0 115200
    ```
- Replay the recorded data to a virtual serial port, as if it was during the recording, with the real device connected:
    ```
    sudo ./serialrecord.sh <LOG FILE NAME TO READ FROM> <VIRTUAL SERIAL DEVICE TO CREATE> <BAUDRATE>
    ```
    Example:
    ``` 
    sudo ./serialreplay.sh outlog.seriallog /tmp/oscar 115200
    ```

**Warning**: `serialreplay` attempts to reproduce the serial trace previously recorded with timings as much accurate as possible. However, small variations may occur due to how the Python core estimates the time that should pass between two consecutive serial data blocks (e.g., two consecutive NMEA sentences).
**Warning**: this is still very **work in progress**! Stay tuned and expect an updated and more feature-rich release of the serial replay tool soon!