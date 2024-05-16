import serial
import time
import os, pty, serial, getopt, sys, threading

exp_timer=True

def timer_expired():
    #print("Expired")
    global exp_timer
    exp_timer=True

def main(argv):
    global exp_timer

    # Default values
    filename="outlog.seriallog"
    device="/tmp/oscar"
    baudrate=115200

    # The mode must always be specified
    mode=""

    try:
        opts, args = getopt.getopt(argv,"f:m:hd:B:",["file=","mode=","help","device=","baudrate="])
    except getopt.GetoptError:
        print('pyserial_test.py -f <file name> -m <mode: either "l" or "r"')
        sys.exit(2)

    for opt,arg in opts:
        if opt == '-h':
            print ('test.py -i <inputfile> -o <outputfile>')
            sys.exit(0)   
        elif opt in ("-i", "--file"):
            filename=arg
        elif opt in ("-m", "--mode"):
            mode=arg
        elif opt in ("-d", "--device"):
            device=arg
        elif opt in ("-B", "--baudrate"):
            baudrate=arg
        else:
            print("Error. Invalid option specified.")
            sys.exit(1)

    if mode != "l" and mode != "r":
        print("Error. Invalid mode. Supported modes are l=log or r=reproduce.")
        sys.exit(1)

    if mode == "l":
        f = open(filename, "ab")

        ser = serial.Serial(
            port=device,\
            baudrate=int(baudrate),\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=0)

        print("Logging from: " + ser.portstr + " - baudrate: " + str(baudrate))
        print("Logging to file: " + filename)

        while True:
            val = ser.readline()
            while not '\\n'in str(val):
                temp = ser.readline() 
                if not not temp.decode():
                    val = (val.decode()+temp.decode()).encode()
            f.write(str(time.time()*1e6).encode())
            f.write(str("\n").encode())
            f.write(val)

        f.close()
    elif mode == "r":
        previous_ts=""
        is_ts=True
        wait_time_us=0

        with open(filename,"r") as file:
            for line in file:
                #print(line.rstrip())
                while exp_timer==False:
                    pass

                before=time.time()

                if(is_ts==False):
                    print(line,end ="",flush=True)
                    #pass

                if(is_ts==True and previous_ts!=""):
                    wait_time_us=float(line)-previous_ts
                    #print("Wait time: %f us" % wait_time_us)
                    previous_ts=float(line)

                    after=time.time()

                    execution_time=(after-before)*1e6

                    exp_timer=False
                    timer = threading.Timer((wait_time_us-execution_time)/1e6,timer_expired)
                    timer.start()
                if previous_ts=="":
                    previous_ts=float(line)
                    #print("First timestamp: %f" % previous_ts)

                if is_ts==True:
                    is_ts=False
                else:
                    is_ts=True


if __name__ == "__main__":
   main(sys.argv[1:])