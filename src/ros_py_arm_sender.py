import serial
from time import sleep
from optparse import OptionParser

# Options
parser = OptionParser()
parser.add_option("-d", "--device", dest = "device",
    default = "/dev/ttyACM0",
    help = "Path to device to connect to Arduino")
parser.add_option("-p", "--port", dest = "port",
    default = "9600",
    help = "Port to connect to Arduino")

(options, args) = parser.parse_args()
device = options.device
port = int(options.port)

# Set up serial connection
ser = serial.Serial(device, port)
if ser.is_open:
    ser.close()
ser.open()

def getSerialArmCommand(angleA, angleB):
    return "A" + str(angleA) + "B" + str(angleB) + "*"

# Serial commands
sHome = getSerialArmCommand(87, 85)

# Communication loop
halt = False
while(halt != True):
    # Ensure safe angles
    if angle_A < 0 or angle_A > 180 or \
       angle_B < 0 or angle_B > 180:
        print ("Angles outside of safe range")
        halt = True
        break
    # Generate command using angles
    sCommand = getSerialArmCommand(angles_A[aidx], angles_B[aidx])
    # Log command
    print ("Sendng serial command %s on %s:%d",
            (sCommand, device, port))
    # Send command to Arduino
    try:
        ser.write(sCommand)
    except KeyboardInterrupt:
        print("Interrupted")
        break
    sleep(.5)

ser.write(sHome)
ser.close()




