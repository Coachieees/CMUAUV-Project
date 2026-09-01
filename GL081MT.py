import serial
import time

# กำหนดพอร์ตของ UART3 และ UART5
PORT_X = '/dev/ttyAMA3' 
PORT_Y = '/dev/ttyAMA4'

try:
    ser_x = serial.Serial(PORT_X, baudrate=115200, timeout=0.1)
    ser_y = serial.Serial(PORT_Y, baudrate=115200, timeout=0.1)
    print("เซนเซอร์พร้อมทำงาน! กำลังอ่านค่าพร้อมกัน...")
except serial.SerialException as e:
    print(f"เกิดข้อผิดพลาดในการเปิดพอร์ต: {e}")
    exit()

# ฟังก์ชันสำหรับถอดรหัสข้อมูล
def parse_data(data):
    for i in range(len(data) - 3):
        if data[i] == 0xFF:
            checksum = (data[i] + data[i+1] + data[i+2]) & 0xFF
            if data[i+3] == checksum:
                return (data[i+1] << 8) | data[i+2]
    return None

try:
    while True:
        # 1. เคลียร์ข้อมูลขยะของทั้งสองพอร์ต
        ser_x.reset_input_buffer()
        ser_y.reset_input_buffer()
        
        # 2. ส่งคำสั่ง Trigger ไปยังเซนเซอร์ทั้ง 2 ตัวแทบจะพร้อมกัน
        ser_x.write(b'\x55')
        ser_y.write(b'\x55')
        
        # 3. หน่วงเวลาครั้งเดียว เพื่อรอให้เซนเซอร์ทั้งคู่ประมวลผลเสร็จ
        time.sleep(0.04)
        
        dist_x = None
        dist_y = None
        
        # 4. อ่านค่าเซนเซอร์แกน X
        if ser_x.in_waiting >= 4:
            dist_x = parse_data(ser_x.read(ser_x.in_waiting))
            
        # 5. อ่านค่าเซนเซอร์แกน Y
        if ser_y.in_waiting >= 4:
            dist_y = parse_data(ser_y.read(ser_y.in_waiting))
            
        # แสดงผล
        val_x = f"{dist_x} mm" if dist_x is not None else "Error"
        val_y = f"{dist_y} mm" if dist_y is not None else "Error"
        
        print(f"Distance X: {val_x} \t|\t Distance Y: {val_y}")
        
        # หน่วงเวลารอบการทำงานของลูปให้สอดคล้องกับ Control Loop
        time.sleep(0.05) 

except KeyboardInterrupt:
    print("\nหยุดการทำงาน")
    ser_x.close()
    ser_y.close()
