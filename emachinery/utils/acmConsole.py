import time, serial, sys

# configure the serial connections (the parameters differs on the device you are connecting to)
console = serial.Serial(
    port='COM5',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)
console.isOpen() # console.open()
console.write(b'hello')

user_input=1
while 1 :
    # get keyboard user_input
    # user_input = sys.stdin.read()
    user_input = input(">> ")
    print("USER", user_input)
    if user_input == 'exit':
        console.close()
    else:
        # send the character to the device
        # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
        console.write(user_input.encode()) # default encoding to utf-8
        out = []

        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while console.inWaiting() > 0:
            out.append( console.read(1) )

        if out != []:
            print("DSP:" , out)

