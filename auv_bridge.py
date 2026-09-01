import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import serial
import time
import board
import busio
import adafruit_bno055
from adafruit_ina219 import INA219
import ms5837

class AUVBridge(Node):
    def __init__(self):
        super().__init__('auv_bridge_node')
        
        # --- ROS 2 SETUP ---
        self.pwm_sub = self.create_subscription(Int32MultiArray, '/auv/cmd_pwm', self.pwm_callback, 10)
        self.sys_sub = self.create_subscription(String, '/auv/cmd_sys', self.sys_callback, 10)
        self.sensor_pub = self.create_publisher(String, '/auv/sensors', 10)
        
        # --- ARDUINO CONNECTION ---
        try:
            self.arduino = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
            self.get_logger().info('Arduino Motor Controller Connected (UART).')
        except Exception as e:
            self.arduino = None
            self.get_logger().error(f'Arduino connection failed: {e}')

        # --- SENSOR INITIALIZATION ---
        try:
            self.i2c = board.I2C() 
        except Exception as e:
            self.i2c = None
            self.get_logger().error(f'I2C Bus failed to initialize: {e}')

        self.bno = None
        if self.i2c:
            try:
                self.bno = adafruit_bno055.BNO055_I2C(self.i2c)
                self.get_logger().info('BNO055 Initialized.')
            except Exception as e:
                self.get_logger().warn(f'BNO055 Not Found: {e}')
            
        self.ina1 = None
        self.ina2 = None
        if self.i2c:
            try:
                self.ina1 = INA219(self.i2c, addr=0x41)
                self.get_logger().info('INA219 (Controller) Initialized.')
            except Exception: pass
            
            try:
                self.ina2 = INA219(self.i2c) 
                self.get_logger().info('INA219 (Thruster) Initialized.')
            except Exception: pass
            
        self.pressure = None
        if self.i2c:
            try:
                self.pressure = ms5837.MS5837_02BA(bus=1) 
                if self.pressure.init():
                    self.pressure.setFluidDensity(ms5837.DENSITY_FRESHWATER)
                    self.get_logger().info('MS5837 Initialized.')
                else:
                    self.pressure = None
            except Exception:
                pass

        # --- RUNTIME VARIABLES ---
        self.last_pwm = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500] 
        self.sensor_timer = self.create_timer(0.05, self.read_and_publish_sensors) 
        
        # 🟢 การตั้งค่า OFFSET แยก 3 แกนอิสระ
        self.yaw_offset = 300.0     # ค่าเริ่มต้นที่คุณวัดได้ (จะถูกทับเมื่อกด TARE YAW)
        self.roll_offset = 0.0      
        self.pitch_offset = 0.0     
        
        # Flags สำหรับรับคำสั่ง
        self.req_tare_roll = False  
        self.req_tare_pitch = False 
        self.req_tare_yaw = False   

    def pwm_callback(self, msg):
        if len(msg.data) == 8:
            self.last_pwm = msg.data

    # 🟢 รับคำสั่ง TARE แยก 3 แกน
    def sys_callback(self, msg):
        if msg.data == "TARE_ROLL":
            self.req_tare_roll = True
            self.get_logger().info('TARE ROLL Requested.')
        elif msg.data == "TARE_PITCH":
            self.req_tare_pitch = True
            self.get_logger().info('TARE PITCH Requested.')
        elif msg.data == "TARE_YAW":
            self.req_tare_yaw = True
            self.get_logger().info('TARE YAW Requested.')

    def send_to_arduino(self, pwm_list):
        if self.arduino:
            try:
                cmd_str = "PWM," + ",".join(map(str, pwm_list)) + "\n"
                self.arduino.write(cmd_str.encode('utf-8'))
            except Exception:
                pass

    def read_and_publish_sensors(self):
        #self.get_logger().info('--- Loop 20Hz is Running ---', throttle_duration_sec=1.0)
        
        yaw, pitch, roll = 0.0, 0.0, 0.0
        quat = (0.0, 0.0, 0.0, 0.0) 
        lin = (0.0, 0.0, 0.0)       
        gyro = (0.0, 0.0, 0.0)      
        temp = 0.0

        if self.bno:
            try:
                euler = self.bno.euler
                if euler and euler[0] is not None: 
                    raw_yaw, raw_pitch, raw_roll = euler
                    
                    # 🟢 จัดการ TARE แยกแกน
                    if self.req_tare_roll:
                        self.roll_offset = raw_roll
                        self.req_tare_roll = False
                    
                    if self.req_tare_pitch:
                        self.pitch_offset = raw_pitch
                        self.req_tare_pitch = False
                        
                    if self.req_tare_yaw:
                        self.yaw_offset = raw_yaw
                        self.req_tare_yaw = False
                    
                    yaw = (raw_yaw - self.yaw_offset) % 360.0
                    if yaw > 180.0: yaw -= 360.0
                        
                    pitch = (raw_pitch - self.pitch_offset) % 360.0
                    if pitch > 180.0: pitch -= 360.0
                        
                    roll = (raw_roll - self.roll_offset) % 360.0
                    if roll > 180.0: roll -= 360.0
                
                q = self.bno.quaternion
                if q and q[0] is not None: quat = q
                
                l = self.bno.linear_acceleration
                if l and l[0] is not None: lin = l
                
                g = self.bno.gyro
                if g and g[0] is not None: gyro = g

                t = self.bno.temperature
                if t is not None: temp = t

                # 🟢 เพิ่มการอ่านค่า 3-Axis Magnetometer
                m = self.bno.magnetic
                if m and m[0] is not None: mag = m
            except Exception as e:
                self.get_logger().warn(f'BNO055 Read Error: {e}', throttle_duration_sec=2.0)

        v1, v2 = 0.0, 0.0
        try:
            if self.ina1: v1 = self.ina1.bus_voltage
            if self.ina2: v2 = self.ina2.bus_voltage
        except Exception: pass

        depth = 0.0
        depth_offset = -0.46
        if self.pressure:
            try:
                self.pressure.read()
                depth = self.pressure.depth() - depth_offset
            except Exception: pass

        sensor_data = (f"DATA,"
                       f"{yaw:.2f},{pitch:.2f},{roll:.2f},"
                       f"{depth:.2f},{v1:.2f},{v2:.2f},"
                       f"{quat[0]:.3f},{quat[1]:.3f},{quat[2]:.3f},{quat[3]:.3f},"
                       f"{lin[0]:.2f},{lin[1]:.2f},{lin[2]:.2f},"
                       f"{gyro[0]:.2f},{gyro[1]:.2f},{gyro[2]:.2f},"
                       f"{temp:.1f},"
                       f"{mag[0]:.2f},{mag[1]:.2f},{mag[2]:.2f}")
        
        msg = String()
        msg.data = sensor_data
        self.sensor_pub.publish(msg)

        self.send_to_arduino(self.last_pwm)

def main(args=None):
    rclpy.init(args=args)
    node = AUVBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
