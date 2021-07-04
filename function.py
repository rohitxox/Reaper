import os
from sys import float_repr_style, prefix
import time
from func_servo import ServoKit
from numpy import *



#dont touch or modify  here----------------------------------------------------------------------------
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
#elbow
kit = ServoKit(channels=16)

from dynamixel_sdk import * 
MY_DXL = 'X_SERIES'     
# Control table address
if MY_DXL == 'X_SERIES':
    add_torque            = 64
    add_goal_pos          = 116
    add_present_pos       = 132    
    baud                  = 1000000
    velocity_mode         = 104
    shutdown              = 63
    load_address          = 126
    load_byte             = 2
    led_address           = 65
    led_len               = 1
    temp_address          = 146
protocol  = 2.0
com                  = '/dev/ttyUSB0'
torque_E  = 1                                            # Value for enabling the torque
torque_D  = 0                                            # Value for disabling the torque
threshold = 15                                           # Dynamixel moving status threshold
move_for = 250                                           #max 250 min 44
move_back = -250
velocity_stop = 0
index = 0
dxl_led_value = [0x00, 0x01]
portHandler = PortHandler(com)
packetHandler = PacketHandler(protocol)
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)
# Initialize GroupBulkWrite instance
# Initialize GroupBulkRead instace for Present Position

# Open port
if portHandler.openPort():
    print("Connecting to Robot")
else:
    print("Failed to connect Robot")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(baud):
    print("Connected....!")
else:
    print("Failed to Sync")
    print("Press any key to terminate...")
    getch()
    quit()
#till here----------------------------------------------------------------------------------------

#Motor ID
leg1_ID = [1, 11, 12, 13, 14]
leg2_ID = [2, 21, 22, 23, 24]
leg3_ID = [3, 31, 32, 33, 34]
leg4_ID = [4, 41, 42, 43, 44]
#motor control start from here.
def motor(ID, val):
    encoder = round(val/ 0.08789)
    pos = encoder
    dxl = packetHandler.write4ByteTxRx(portHandler, ID, add_goal_pos, pos)
    return dxl
  
  
def torque_en(ID):
    packetHandler.write1ByteTxRx(portHandler, ID , add_torque, torque_E)


def torque_dis(ID):
    packetHandler.write1ByteTxRx(portHandler, ID, add_torque, torque_D)


def wheel_velocity(ID, control):
    packetHandler.write4ByteTxRx(portHandler, ID, velocity_mode, control)

#current = load   
def current(ID):        # Add parameter for load conition
    dxl_current_present,dxl_current_error,dxl_current_result = packetHandler.read2ByteTxRx(portHandler, ID, load_address)
    if dxl_current_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_current_result)) #500
    elif dxl_current_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_current_error))
    data = dxl_current_present / 10
    print("[ID:%03d]  Present Load: %03d percent" % (ID, data))#%1.0f


def temp(ID):
    dxl_temp_error,dxl_temp_result,dxl_temp_present = packetHandler.read1ByteTxRx(portHandler, ID, temp_address)
    if dxl_temp_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_temp_result))
    elif dxl_temp_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_temp_error))
    print("[ID:%03d]  PressentTemp:%1.0f" % (ID, dxl_temp_present))
    

def led(ID, mode):
    groupBulkRead.addParam(ID, led_address, led_len)
    groupBulkWrite.addParam(ID, led_address, led_len, [dxl_led_value[mode]]) #[dxl_led_value[index]]
    return groupBulkWrite.txPacket()


#///////////////////////////////////////////////////////////////////////////////!
#inverse kinematic  👉------------------->
def test_IK(c):
    #given data
    a = 9.4        #knee        +0.3 offset
    b = 6.9        #shoulder    +0.1 offset
    cos_A = (b**2 + c**2 - a**2) / (2 * b * c)
    shoulderAngle = math.acos(cos_A)
    A = round(math.degrees(shoulderAngle),1)

    #let find angle B
    cos_B = (c**2 + a**2 -b**2) / (2 * c * a)
    kneeAngle = math.acos(cos_B)
    B = round(math.degrees(kneeAngle),1)

    #finally let find angle C
    C = round(180 - A - B,1)

    #offset
    shoulderOff = (A + 180)              #shoulder 150
    kneeOff     = (C + 55.5)              #knee 131
    # return shoulderOff, kneeOff
    #standing parameter
    motor(11,shoulderOff)
    motor(12,kneeOff)
    motor(13,180)
    
    motor(21,shoulderOff)
    motor(22,kneeOff)
    motor(23,180)
    #
    motor(31,shoulderOff)
    motor(32,kneeOff)
    motor(33,180)
    #
    motor(41,shoulderOff)
    motor(42,kneeOff)
    motor(43,180)
    
def IK_standing_translation(x):
    #data
    h = 10
    # x = 5
    #let find angle theta
    angle = math.tan(x / h)
    Angle = round(math.degrees(angle))

    #let find hypotenuse
    z = math.cos(angle)
    out = round(h/z)

    a = 9.4        #knee        +0.3 offset
    b = 6.9        #shoulder    +0.1 offset
    c = out
        
    cos_A = (b**2 + c**2 - a**2) / (2 * b * c)
    shoulderAngle = math.acos(cos_A)
    A = round(math.degrees(shoulderAngle),1)

    #let find angle B
    cos_B = (c**2 + a**2 -b**2) / (2 * c * a)
    kneeAngle = math.acos(cos_B)
    B = round(math.degrees(kneeAngle),1)

    #finally let find angle C
    C = round(180 - A - B,1)

    #offset
    shoulderOff = (A + 180) +  (angle + 90)         #shoulder 150
    kneeOff     = (C + 55.5)              #knee 131

    motor(11,shoulderOff)
    motor(12,kneeOff)
    motor(13,180)
    
    motor(21,shoulderOff)
    motor(22,kneeOff)
    motor(23,180)
    #
    motor(31,shoulderOff)
    motor(32,kneeOff)
    motor(33,180)
    #
    motor(41,shoulderOff)
    motor(42,kneeOff)
    motor(43,180)

#///////////////////////////////////////////////////////////////////////////////!
class Motion:
    def enable_motor(self):    
        torque_en(11)
        torque_en(12)
        torque_en(13)
        torque_en(14)
        #
        torque_en(21)
        torque_en(22)
        torque_en(23)
        torque_en(24)
        #
        torque_en(31)
        torque_en(32)
        torque_en(33)
        torque_en(34)
        #
        torque_en(41)
        torque_en(42)
        torque_en(43)
        torque_en(44)
        
        
    def disable_motor(self):
        torque_dis(11)
        torque_dis(12)
        torque_dis(13)
        torque_dis(14)
        #
        torque_dis(21)
        torque_dis(22)
        torque_dis(23)
        torque_dis(24)
        #
        torque_dis(31)
        torque_dis(32)
        torque_dis(33)
        torque_dis(34)
        #
        torque_dis(41)
        torque_dis(42)
        torque_dis(43)
        torque_dis(44)
    

    def IK_stand(self,c):
        
        #given data
        a = 9.4        #knee        +0.3 offset
        b = 6.9        #shoulder    +0.1 offset
        cos_A = (b**2 + c**2 - a**2) / (2 * b * c)
        shoulderAngle = math.acos(cos_A)
        A = round(math.degrees(shoulderAngle),1)

        #let find angle B
        cos_B = (c**2 + a**2 -b**2) / (2 * c * a)
        kneeAngle = math.acos(cos_B)
        B = round(math.degrees(kneeAngle),1)

        #finally let find angle C
        C = round(180 - A - B,1)

        #offset
        shoulderOff = (A + 180)              #shoulder 150
        kneeOff     = (C + 55.5)              #knee 131

        #standing parameter
        motor(11,shoulderOff)
        motor(12,kneeOff)
        motor(13,180)
        
        motor(21,shoulderOff)
        motor(22,kneeOff)
        motor(23,180)
        #
        motor(31,shoulderOff)
        motor(32,kneeOff)
        motor(33,180)
        #
        motor(41,shoulderOff)
        motor(42,kneeOff)
        motor(43,180)

        

    def test(self):
        for i in range(5):
            c =8
            while c < 15:
                test_IK(c)
                c += 1
            z = 15
            while z >= 8:
                z = c
                test_IK(c)
                c -= 1


    # def test1(self):
    #     x =3
    #     while x < 9:
    #         IK_standing_translation(x)
    #         x += 1
    #     # z = 14
    #     # while z >= 6:
    #     #     z = x
    #     #     IK_standing_translation(x)
    #     #     x -= 1


    def down(self):
        motor(11,237)
        motor(12,118)
        #
        motor(21,237)
        motor(22,118)
        #
        motor(31,237)
        motor(32,118)
        #
        motor(41,237)
        motor(42,118)
        

    def leg_seq(self):
        #start
        #leg1
        motor(11, 215)
        motor(12, 182)
        #leg3
        motor(31, 173)
        motor(32, 216)
        #leg2
        motor(21, 173)
        motor(22, 216)
        #leg4
        motor(41, 215)#mirror of first leg1
        motor(42, 182)
        time.sleep(0.6)
        #alter
        #leg1
        motor(11, 184)
        motor(12, 211)
        #leg3
        motor(31, 232)
        motor(32, 182)
        #leg2
        motor(21, 232)
        motor(22, 185)
        #leg4#mirror of alter first leg1
        motor(41, 184)
        motor(42, 211)
        time.sleep(0.6)   


    def elbow(self):
        kit.servo[1].angle = 15 #15
        kit.servo[2].angle = 12
        kit.servo[3].angle = 11
        kit.servo[4].angle = 15


    def elbow_90(self):
        kit.servo[1].angle = 90
        #kit.servo[2].angle = 90
        #kit.servo[3].angle = 90
        #kit.servo[4].angle = 90
    

    def elbow_home(self):
        kit.servo[1].angle = 90


    def vel_move_forward(self):
        wheel_velocity(14, move_back)
        wheel_velocity(24, move_for)
        wheel_velocity(34, move_back)
        wheel_velocity(44, move_for)
        

    def vel_move_backward(self):
        wheel_velocity(14, move_for)
        wheel_velocity(24, move_back)
        wheel_velocity(34, move_for)
        wheel_velocity(44, move_back)
    
    
    def velocity_stop_mode(self):
        wheel_velocity(14, velocity_stop)
        wheel_velocity(24, velocity_stop)
        wheel_velocity(34, velocity_stop)
        wheel_velocity(44, velocity_stop)
        
        
    def turn_around(self):
        wheel_velocity(14, move_for)
        wheel_velocity(24, move_for)
        wheel_velocity(34, move_for)
        wheel_velocity(44, move_for)
    
    
    def side_rot(self):
        motor(13, 359)
        #motor(23, 0100)
        motor(33, 359)
        motor(43, 179)
    
    
    def side_way_right(self):
        #leg1&4
        motor(12, 150)#up
        motor(42, 152)
        time.sleep(0.2)
        kit.servo[1].angle = 25
        kit.servo[4].angle = 3  
        
        motor(12, 180)#touch down
        motor(42, 180)
        time.sleep(0.1)
        #leg2&3
        motor(22, 150)#up
        motor(32, 132)
        time.sleep(0.2)
        kit.servo[2].angle = 3
        kit.servo[3].angle = 25  
        
        motor(22, 180)#touch down
        motor(32, 180)
    
    
    def side_way_left(self):
        #leg1&4
        motor(12, 152)#up
        motor(42, 132)
        time.sleep(0.2)
        kit.servo[1].angle = 3
        kit.servo[4].angle = 25  
        
        motor(12, 180)#touch down
        motor(42, 180)
        time.sleep(0.2)
        #leg2&3
        motor(22, 152)#up
        motor(32, 132)
        time.sleep(0.2)
        kit.servo[2].angle = 25
        kit.servo[3].angle = 3  
        
        motor(22, 180)#touch down
        motor(32, 180)
        
            
    def get_current(self,ID):
        current(ID)
    
    
    def get_temp(self,ID):
        temp(ID)
        

    def activate_led(self, ID):
            led(ID, 1)
      
      
    def deactivate_led(self, ID):
            led(ID, 0)
    
    
    def distance(self,time):
        speed = 0.0247368 #m/s
        #time = 19 #sec
        D = speed * time
        return print(round(D, 4))


#main func----------------------------------------------------------------------------------------
command = ''
started = False
stopped = False
ledOn = False
start = Motion()
num = 10
turn = 5
def walk_stats():
    try:
        while True:
            start.leg_seq()
    except KeyboardInterrupt:  
        c = 12    
        start.IK_stand(c)
        return 'Robot is now halted!'
#------------?


while True:
    command = input('->').lower()
    
    if command == 's':
        count = int(input('height parameter?').lower())
        c = count  
        if count >= 10:      
            start.enable_motor()
            start.elbow()
            start.IK_stand(c)
            count -=1
            print(f'robot height is now {c}')
         

    elif command == 'j':    
        count = int(input('distance parameter?').lower())
        x = count
        print(f'robot distance is now {x}')
        start.enable_motor()
        start.elbow()
        IK_standing_translation(x)
    
    elif command == 'm':
        while turn <5:
            start.side_way_right()
            turn -= 1

    elif command == 'd':
        if stopped:
            stopped = True
            print("it is down")              
        else:
            print("Robot now laying down...!")
            start.down()
            time.sleep(2)
            start.disable_motor()

    elif command == 'w':
        walk_stats()
    
    elif command == 't':
        pass
        # start.enable_motor()
        # start.test()
        #start.test1()
        #start.elbow_90()
        # start.enable_motor()
        # start.standing_translation()

    elif command == 'n':
        start.enable_motor()
        start.test()

    elif command == 'g':
        start.elbow_home()
        
    elif command == '1':
        ID = int(input("motor led_on..? :").lower())
        start.activate_led(ID)
        groupBulkWrite.clearParam()
    elif command == '2':
        ID = int(input("motor led_off..! :").lower())
        start.deactivate_led(ID)
        groupBulkWrite.clearParam()
    elif command == 'l':
        ID = int(input("which motor do you want to find load act..? :").lower())
        start.get_current(ID)
        
    elif command == 'f':
        start.vel_move_forward()
    elif command == 'e':
        start.velocity_stop_mode()
    elif command == 'b':
        start.vel_move_backward()
    elif command == 'q':
        break
    elif command == 'h':
        print("""
These are the command's to control the Robot:
To stand up   -'s'
To sit down   -'d'
elbow 90deg   -'p'
walk forward  -'w'
move forward  -'f'
move backward -'b'
stop motion   -'e'
get load      -'l'
turn on led   -'1'
turn off led  -'2'
quit          -'q'
test          -'t'

""")
    else:
        print("i did't understand that")



groupBulkRead.clearParam()
# Close port
portHandler.closePort()




