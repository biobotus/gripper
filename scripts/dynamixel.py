#!/usr/bin/python
import serial
import pigpio
import time

def format_message(arg_list):
    """Returns a string of chars for each arg in the list."""
    out = ""
    for arg in arg_list:
        out += chr(arg)
    return out

class ComAX12a:

#---------------------------------------------------------------------------
#-------------------Dynamixel constants and adresses------------------------
#---------------------------------------------------------------------------

    #--------------------------------------------------------------------
    # EEPROM address-----------------------------------------------------
    #--------------------------------------------------------------------
    MODEL_NUMBER_L = 0
    MODEL_NUMBER_H = 1
    VERSION = 2
    ID_ADDR = 3
    BAUD_RATE = 4
    RETURN_DELAY_TIME = 5
    CW_ANGLE_LIMIT_L = 6
    CW_ANGLE_LIMIT_H = 7
    CCW_ANGLE_LIMIT_L = 8
    CCW_ANGLE_LIMIT_H = 9
    SYSTEM_DATA2 = 10
    LIMIT_TEMPERATURE = 11
    DOWN_LIMIT_VOLTAGE = 12
    UP_LIMIT_VOLTAGE = 13
    MAX_TORQUE_L = 14
    MAX_TORQUE_H = 15
    RETURN_LEVEL = 16
    ALARM_LED = 17
    ALARM_SHUTDOWN = 18
    OPERATING_MODE = 19
    DOWN_CALIBRATION_L = 20
    DOWN_CALIBRATION_H = 21
    UP_CALIBRATION_L = 22
    UP_CALIBRATION_H = 23

    #--------------------------------------------------------------------
    # RAM address--------------------------------------------------------
    #--------------------------------------------------------------------
    TORQUE_STATUS = 24
    LED_STATUS = 25
    CW_COMPLIANCE_MARGIN = 26
    CCW_COMPLIANCE_MARGIN = 27
    CW_COMPLIANCE_SLOPE = 28
    CCW_COMPLIANCE_SLOPE = 29
    GOAL_POSITION_L = 30
    GOAL_POSITION_H = 31
    GOAL_SPEED_L = 32
    GOAL_SPEED_H = 33
    TORQUE_LIMIT_L = 34
    TORQUE_LIMIT_H = 35
    PRESENT_POSITION_L = 36
    PRESENT_POSITION_H = 37
    PRESENT_SPEED_L = 38
    PRESENT_SPEED_H = 39
    PRESENT_LOAD_L = 40
    PRESENT_LOAD_H = 41
    PRESENT_VOLTAGE = 42
    PRESENT_TEMPERATURE = 43
    REGISTERED_INSTRUCTION = 44
    PAUSE_TIME = 45
    MOVING = 46
    LOCK = 47
    PUNCH_L = 48
    PUNCH_H = 49

    #--------------------------------------------------------------------
    # Status return types------------------------------------------------
    #--------------------------------------------------------------------
    RETURN_NONE = 0
    RETURN_READ = 1
    RETURN_ALL = 2

    #--------------------------------------------------------------------
    # Instruction identifiant--------------------------------------------
    #--------------------------------------------------------------------
    PING = 1
    READ_DATA = 2
    WRITE_DATA = 3
    REG_WRITE = 4
    ACTION = 5
    RESET = 6
    SYNC_WRITE = 131

    #--------------------------------------------------------------------
    # Message length-----------------------------------------------------
    #--------------------------------------------------------------------
    RESET_LENGTH = 2
    ACTION_LENGTH = 2
    ID_LENGTH = 4
    LR_LENGTH = 4
    SRL_LENGTH = 4
    RDT_LENGTH = 4
    LEDALARM_LENGTH = 4
    SHUTDOWNALARM_LENGTH = 4
    TL_LENGTH = 4
    VL_LENGTH = 6
    AL_LENGTH = 7
    CM_LENGTH = 6
    CS_LENGTH = 5
    COMPLIANCE_LENGTH = 7
    CCW_CW_LENGTH = 8
    BD_LENGTH = 4
    TEM_LENGTH = 4
    MOVING_LENGTH = 4
    RWS_LENGTH = 4
    VOLT_LENGTH = 4
    LOAD_LENGTH = 4
    LED_LENGTH = 4
    TORQUE_LENGTH = 5
    POS_LENGTH = 4
    GOAL_LENGTH = 5
    MT_LENGTH = 5
    PUNCH_LENGTH = 5
    SPEED_LENGTH = 5
    GOAL_SP_LENGTH = 7

    #--------------------------------------------------------------------
    # Special constants--------------------------------------------------
    #--------------------------------------------------------------------
    BYTE_READ = 1
    INT_READ = 2
    ACTION_CHECKSUM = 250
    BROADCAST_ID = 254
    START = 255
    CCW_AL_L = 255
    CCW_AL_H = 3
    LOCK_VALUE = 1

    LEFT = 0
    RIGHT = 1

    RX_TIME_OUT = 10
    TX_DELAY_TIME = 0.00002

    #--------------------------------------------------------------------
    # RPI constants------------------------------------------------------
    #--------------------------------------------------------------------
    RPI_DIRECTION_TX = pigpio.HIGH
    RPI_DIRECTION_RX = pigpio.LOW
    RPI_DIRECTION_SWITCH_DELAY = 0.0001
    QUERY_DELAY = .01

    #-----------------------------------------------------------------------
    #-------------------------------Methods-------------------------------
    #-----------------------------------------------------------------------

    #---------------------------General methods---------------------------
    def __init__(self, baudrate, dir_pin):
        self.baudrate = baudrate
        self.dir_pin = dir_pin
        self.byte_delay = 1/(baudrate*10)

        # Handle for PIGPIO
        self.gpio = pigpio.pi()
        self.gpio.set_mode(dir_pin, pigpio.OUTPUT)

        # Handle fo Serial communication
        self.port = serial.Serial("/dev/ttyAMA0", baudrate=self.baudrate,\
                                  timeout=self.QUERY_DELAY)

    def com_direction(self, write_read):
        self.gpio.write(self.dir_pin, write_read)

    def checksum(self, message):
        # Return checksum of message
        return ~(message)&0xFF

    def read_data(self, ID, size):
        # size: number of bytes expected
        valid_reply = False;
        self.com_direction(self.RPI_DIRECTION_RX)
        in_data_str = self.port.read(size)
        message = 0
        #print([ord(i) for i in in_data_str])

        # Validate Checksum
        if len(in_data_str) != size:
            return valid_reply, None

        for i in range(size)[2:-1]:
            message += ord(in_data_str[i])

        check = self.checksum(message)

        if check == ord(in_data_str[-1]) and ord(in_data_str[2]) == ID:
            valid_reply = True
        else:
            print("Received bad checksum")

        # Look for error
        if  ord(in_data_str[4]) != 0:
            error = ord(in_data_str[4])
            #print([ord(i) for i in in_data_str])
            if error & 0x40:
                print("ERROR: Instruction")
                valid_reply = False
            if error & 0x20:
                print("ERROR: Overload")
                valid_reply = False
            if error & 0x10:
                print("ERROR: Checksum")
                valid_reply = False
            if error & 0x08:
                if self.is_register_lock(ID):
                    print("ERROR: Register of target ID is locked")
                else:
                    print("ERROR: Range")
                valid_reply = False
            if error & 0x04:
                print("ERROR: Overheating")
                valid_reply = False
            if error & 0x02:
                print("ERROR: Angle limit")
                valid_reply = False
            if error & 0x01:
                print("ERROR: Input voltage")
                valid_reply = False

        return valid_reply, in_data_str

    def write_data(self, message):
        self.port.flushInput()
        time.sleep(self.RPI_DIRECTION_SWITCH_DELAY)
        self.com_direction(self.RPI_DIRECTION_TX)
        time.sleep(self.RPI_DIRECTION_SWITCH_DELAY)
        self.port.write(message)

        time.sleep(self.RPI_DIRECTION_SWITCH_DELAY)
        # Wait the appropriate time
        time.sleep(len(message)*self.byte_delay)

    def ping(self, ID):
        # Send Ping
        check = self.checksum(ID + self.READ_DATA + self.PING)
        out_data = format_message([self.START, self.START, ID, self.READ_DATA, \
                                   self.PING, check])

        # Transmit data
        self.write_data(out_data)

        # Read received data
        valid_reply, _ = self.read_data(ID, 6)
        return valid_reply

    def servo_census(self, min_id=0, max_id=6):
        return [i for i in range(min_id, max_id+1) if self.ping(i)]

    def factory_reset(self, ID, confirm = False):
        # Reset register to factory settings
        if(confirm):
            check = self.checksum(ID + self.RESET_LENGTH + self.RESET)

            out_data = format_message([self.START, self.START, ID, \
                                       self.RESET_LENGTH, self.RESET, check])

            # Transmit data
            self.write_data(out_data)

            # Read received data
            valid_reply, _ = self.read_data(ID,6)
            return valid_reply
        else:
            return False

#-----------------------Register setting functions----------------------
    def set_status_return_level(self, ID, level):
        check = self.checksum(ID + self.SRL_LENGTH + self.WRITE_DATA + \
                              self.RETURN_LEVEL + level)

        out_data = format_message([self.START, self.START, ID, \
                                   self.SRL_LENGTH, self.WRITE_DATA, \
                                   self.RETURN_LEVEL, level, check])

        # Transmit data
        self.write_data(out_data)

        valid_reply, _ = self.read_data(ID, 6)
        return valid_reply

    def set_return_delay_time(self, ID, delay):
        delay = (delay)&0xff
        check = self.checksum(ID + self.RDT_LENGTH + self.WRITE_DATA + \
                              self.RETURN_DELAY_TIME + delay)

        out_data = format_message([self.START, self.START, ID, \
                                   self.RDT_LENGTH, self.WRITE_DATA, \
                                   self.RETURN_DELAY_TIME, delay, check])

        # Transmit data
        self.write_data(out_data)

        valid_reply, data = self.read_data(ID, 6)
        return valid_reply

    def set_angle_limit(self, ID, cw_limit, ccw_limit):
        # Degree to hex angle
        cw_limit = int(round(cw_limit*(1024./300)+ 1024./2))
        ccw_limit = int(round(ccw_limit*(1024./300)+ 1024./2))

        if cw_limit < 0:
            cw_limit = 0
        if ccw_limit > 1023:
            ccw_limit = 1023

        cw = [cw_limit&0xff, cw_limit>>8]
        ccw = [ccw_limit&0xff, ccw_limit>>8]
        check = self.checksum(ID + self.AL_LENGTH + self.WRITE_DATA + \
                       self.CW_ANGLE_LIMIT_L + cw[0] + cw[1] + ccw[0] + ccw[1])

        out_data = format_message([self.START, self.START, ID, \
                                   self.AL_LENGTH, self.WRITE_DATA, \
                                   self.CW_ANGLE_LIMIT_L, cw[0], cw[1], \
                                   ccw[0], ccw[1], check])

        # Transmit data
        self.write_data(out_data)

        valid_reply, data = self.read_data(ID, 6)
        return valid_reply

    def set_max_torque(self, ID, torque_percent):
        # Validate torque value (between 0 and 1)
        # Writes in EEPROM
        if torque_percent > 1 or torque_percent < 0:
            return False

        # Translate torque (%) into RPM into command format
        torque =  int(round(torque_percent*(1023)))&0x3ff
        t = [torque&0xff, torque>>8]

        # Construct message to output
        check = self.checksum(ID + self.MT_LENGTH + self.WRITE_DATA + \
                              self.MAX_TORQUE_L + t[0] + t[1])

        out_data = format_message([self.START, self.START, ID, \
                                   self.MT_LENGTH, self.WRITE_DATA, \
                                   self.MAX_TORQUE_L, t[0], t[1], check])

        # Transmit data
        self.write_data(out_data)

        # No return message if broadcast message
        if ID == self.BROADCAST_ID:
            return True

        # Validate return packet
        valid_reply, data = self.read_data(ID, 6)
        return valid_reply

    def set_torque_limit(self, ID, torque_percent):
        # Validate torque value (between 0 and 1)
        # Writes in RAM
        if torque_percent > 1 or torque_percent < 0:
            return False

        # Translate torque (%) into RPM into command format
        torque =  int(round(torque_percent*(1023)))&0x3ff
        t = [torque&0xff, torque>>8]

        # Construct message to output
        check = self.checksum(ID + self.TORQUE_LENGTH  + self.WRITE_DATA + \
                              self.TORQUE_LIMIT_L  + t[0] + t[1])

        out_data = format_message([self.START, self.START, ID, \
                                   self.TORQUE_LENGTH, self.WRITE_DATA, \
                                   self.TORQUE_LIMIT_L, t[0], t[1], check])

        # Transmit data
        self.write_data(out_data)

        # No return message if broadcast message
        if ID == self.BROADCAST_ID:
            return True

        # Validate return packet
        valid_reply, data = self.read_data(ID, 6)
        return valid_reply

    def set_id(self, ID, new_id):
        # Set new ID
        check = self.checksum(ID + self.ID_LENGTH +self.WRITE_DATA + \
                              self.ID_ADDR + new_id)

        out_data = format_message([self.START, self.START, ID, \
                                   self.ID_LENGTH, self.WRITE_DATA, \
                                   self.ID_ADDR, new_id, check])

        # Transmit data
        self.write_data(out_data)

        # Read received data
        valid_reply, _ = self.read_data(ID, 6)
        return valid_reply

    def set_punch_limit(self, ID, punch):
        p = [punch&0xff, punch>>8]

        # Construct message to output
        check = self.checksum(ID + self.PUNCH_LENGTH  + self.WRITE_DATA + \
                              self.PUNCH_L + p[0] + p[1])

        out_data = format_message([self.START, self.START, ID, \
                                   self.PUNCH_LENGTH, self.WRITE_DATA, \
                                   self.PUNCH_L, p[0], p[1], check])

        # Transmit data
        self.write_data(out_data)

        # No return message if broadcast message
        if ID == self.BROADCAST_ID:
            return True

        # Validate return packet
        valid_reply, data = self.read_data(ID, 6)
        return valid_reply

    def set_baudrate(self, ID, baudrate):
        # Set new baudrate to target ID
        br = int((2000000./baudrate)-1)&0xff
        check = self.checksum(ID + self.BD_LENGTH +self.WRITE_DATA +
                              self.BAUD_RATE + br)

        out_data = format_message([self.START, self.START, ID, \
                                   self.BD_LENGTH, self.WRITE_DATA, \
                                   self.BAUD_RATE, br, check])

        # Transmit data
        self.write_data(out_data)

        # Read received data
        valid_reply, _ = self.read_data(ID, 6)
        return valid_reply

    def lock_register(self, ID):
         # Lock register of target ID

        check = self.checksum(ID + self.LR_LENGTH +self.WRITE_DATA + \
                              self.LOCK + self.LOCK_VALUE)

        out_data = format_message([self.START, self.START, ID, \
                                   self.LR_LENGTH, self.WRITE_DATA, \
                                   self.LOCK, self.LOCK_VALUE, check])

        # Transmit data
        self.write_data(out_data)

        # Read received data
        valid_reply, _ = self.read_data(ID, 6)
        return valid_reply

    def set_temp_limit(self, ID, temp):
        # Set maximum temperature value
        t = int(temp)&0xff
        if t > 150:
            t = 150

        check = self.checksum(ID + self.TL_LENGTH +self.WRITE_DATA + \
                              self.LIMIT_TEMPERATURE + t)

        out_data = format_message([self.START, self.START, ID, \
                                   self.TL_LENGTH, self.WRITE_DATA, \
                                   self.LIMIT_TEMPERATURE, t, check])

        # Transmit data
        self.write_data(out_data)

        # Read received data
        valid_reply, _ = self.read_data(ID, 6)
        return valid_reply

#--------------------------Action functions-----------------------------
    def set_goal_position(self, ID, position):
        # Translate degree position into command format
        position = int(round(position*(1024./300)+ 511))&0x3ff
        p = [position&0xff, position>>8]

        # Construct message to output
        check = self.checksum(ID + self.GOAL_LENGTH + self.WRITE_DATA + \
                              self.GOAL_POSITION_L + p[0]+p[1])

        out_data = format_message([self.START, self.START, ID, \
                                   self.GOAL_LENGTH, self.WRITE_DATA, \
                                   self.GOAL_POSITION_L, p[0], p[1], check])

        # Transmit data
        self.write_data(out_data)

        # No return message if broadcast ID
        if ID == self.BROADCAST_ID:
            return True

        # Validate return packet
        valid_reply, data = self.read_data(ID, 6)
        return valid_reply

    def set_goal_pos_speed(self, ID, pos_deg, speed_rpm):
        # Translate degree position into command format
        position = int(round(pos_deg*(1024./300)+ 511))&0x3ff
        p = [position&0xff, position>>8]

        # Translate speed in RPM into command format
        speed = int(round(speed_rpm*(1023./114)))&0x3ff
        s = [speed&0xff, speed>>8]

        # Construct message to output
        check = self.checksum(ID + self.GOAL_SP_LENGTH + self.WRITE_DATA + \
                            self.GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1])

        out_data = format_message([self.START, self.START, ID, \
                                   self.GOAL_SP_LENGTH, self.WRITE_DATA, \
                                   self.GOAL_POSITION_L, p[0], p[1], s[0], \
                                   s[1], check])

        # Transmit data
        self.write_data(out_data)

        # No return message if broadcast ID
        if ID == self.BROADCAST_ID:
            return True

        # Validate return packet
        valid_reply, data = self.read_data(ID, 6)
        return valid_reply


    #----------------------Register reading functions-----------------------
    def read_temperature(self, ID):
        check = self.checksum(ID + self.TEM_LENGTH + self.READ_DATA + \
                              self.PRESENT_TEMPERATURE + self.BYTE_READ)
        out_data = format_message([self.START, self.START, ID, \
                                   self.TEM_LENGTH, self.READ_DATA, \
                                   self.PRESENT_TEMPERATURE, self.BYTE_READ, \
                                   check])

        # Transmit data
        self.write_data(out_data)

        valid_reply, data = self.read_data(ID, 7)
        if valid_reply:
            return ord(data[5])
        else:
            return None

    def read_load(self, ID):
        check = self.checksum(ID + self.LOAD_LENGTH + self.READ_DATA + \
                              self.PRESENT_LOAD_L + self.INT_READ)

        out_data = format_message([self.START, self.START, ID, \
                                   self.LOAD_LENGTH, self.READ_DATA, \
                                   self.PRESENT_LOAD_L, self.INT_READ, check])

        # Transmit data
        self.write_data(out_data)
        valid_reply, data = self.read_data(ID, 8)
        if valid_reply:
            raw_load = (ord(data[6]))*256+ (ord(data[5]))

            # Determine the direction bit (bit 10)
            if raw_load >> 10:
                load = -1*int(raw_load&0x3ff)
            else:
                load = raw_load&0x3ff

            # Return in % of maximum torque
            return (load/1024.)
        else:
            return None

    def read_position(self, ID):
        check = self.checksum(ID + self.POS_LENGTH  + self.READ_DATA + \
                              self.PRESENT_POSITION_L  + self.INT_READ)

        out_data = format_message([self.START, self.START, ID, \
                                   self.POS_LENGTH, self.READ_DATA, \
                                   self.PRESENT_POSITION_L, self.INT_READ, \
                                   check])

        # Transmit data
        self.write_data(out_data)

        valid_reply, data = self.read_data(ID, 8)

        if valid_reply:
            raw_pos = (ord(data[6]))*256+ (ord(data[5]))
            pos = (raw_pos - 511)*300./1024.
            return pos  # in degrees
        else:
            return -0xfff # Error

    def read_moving_status(self, ID):

        check = self.checksum(ID + self.MOVING_LENGTH   + self.READ_DATA + \
                              self.MOVING   + self.BYTE_READ)

        out_data = format_message([self.START, self.START, ID, \
                                   self.MOVING_LENGTH, self.READ_DATA, \
                                   self.MOVING, self.BYTE_READ, check])

        # Transmit data
        self.write_data(out_data)

        valid_reply, data = self.read_data(ID, 7)

        if valid_reply:
            return bool(ord(data[-2]))
        else:
            return None

    def is_register_lock(self, ID):
        # Return true if register of target ID is lock
        check = self.checksum(ID + self.LR_LENGTH + self.READ_DATA + \
                              self.LOCK + self.BYTE_READ)

        out_data = format_message([self.START, self.START, ID, \
                                   self.LR_LENGTH, self.READ_DATA, \
                                   self.LOCK, self.BYTE_READ, check])

        # Transmit data
        self.write_data(out_data)

        valid_reply, data = self.read_data(ID, 7)

        if valid_reply:
            return bool(ord(data[-2]))
        else:
            return None

    def read_voltage(self, ID):
        check = self.checksum(ID + self.VOLT_LENGTH + self.READ_DATA + \
                              self.PRESENT_VOLTAGE + self.BYTE_READ)

        out_data = format_message([self.START, self.START, ID, \
                                   self.VOLT_LENGTH, self.READ_DATA, \
                                   self.PRESENT_VOLTAGE, self.BYTE_READ, check])

        # Transmit data
        self.write_data(out_data)

        valid_reply, data = self.read_data(ID, 7)

        if valid_reply:
            return ord(data[-2])/10.
        else:
            return 0


if __name__ == "__main__":

    motor = ComAX12a(1000000, 18)
    # Set proper return delay_time (max =254)
    #motor.set_return_delay_time(254, 254)
    # Set proper returne respond level
    #motor.set_status_return_level(254,motor.RETURN_ALL)

    # Reset limit angles
    #motor.set_angle_limit(254, -150, 150)

    print("Start")
    print("")

    #Census servo ID
    print("Servo ID: {0}".format(motor.servo_census()))
    # print(motor.factory_reset(2,confirm = True))
    # time.sleep(1)
    # print("Servo ID: {0}".format(motor.servo_census()))
    #Set proper punch limit
    #print(motor.set_punch_limit(2,50))
    #Set proper torque values
    #print(motor.set_max_torque(motor.BROADCAST_ID,.80))
    #print(motor.set_torque_limit(motor.BROADCAST_ID,.80))

    # Move motors
    print(motor.set_goal_pos_speed(254, 0, 10,))
    print(motor.set_goal_pos_speed_blk(2, 0, 10, 2, 1000))
    print(motor.set_goal_pos_speed_blk(2, -90, 10, 3, 1000))
    motor.set_goal_pos_speed(3, 55, 25)
    motor.set_goal_pos_speed(4, 55, 25)
    print(motor.set_goal_pos_speed(2, -90, 2))

    #time.sleep(1)
    #print(motor.set_goal_pos_speed(2, -90, 10))
    #time.sleep(2)
    #print(motor.set_goal_pos_speed(3, 55, 10))
    #print(motor.set_goal_pos_speed(4, 55, 10))

    # while True:
        # # # print(motor.read_load(2))
        # print("Pos: {0} deg,  Torque: {1}, temp: {2} C".format(motor.read_position(2),motor.read_load(2),motor.read_temperature(2)))
        # print("")
        # time.sleep(.5)

