import websocket
import json
import pandas as pd
from datetime import datetime
import time
import threading
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from collections import deque
import numpy as np

class AUVLogger:
    def __init__(self):
        self.data = []
        self.is_recording = False
        self.ws = None
        
        # สำหรับเก็บข้อมูลกราฟแบบ real-time
        self.max_points = 100  # จำนวนจุดสูงสุดที่แสดงในกราฟ
        self.times = deque(maxlen=self.max_points)
        self.y_angles = deque(maxlen=self.max_points)
        self.z_angles = deque(maxlen=self.max_points)
        
        self.start_time = None
        self.setup_gui()
        self.setup_plot()

    def setup_plot(self):
        # สร้าง Figure สำหรับ matplotlib
        self.fig = Figure(figsize=(8, 4))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title('Real-time Angle Data')
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Angle (degrees)')
        self.ax.grid(True)
        
        # สร้างเส้นกราฟเริ่มต้น
        self.line_y, = self.ax.plot([], [], 'orange', label='Y Angle')
        self.line_z, = self.ax.plot([], [], 'blue', label='Z Angle')
        self.ax.legend()
        
        # เพิ่ม canvas ลงใน GUI
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("AUV Data Logger")
        self.root.geometry("1000x800")

        # สร้าง GUI elements
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Control Frame
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E))

        # Status Label
        self.status_var = tk.StringVar(value="สถานะ: ไม่ได้เชื่อมต่อ")
        ttk.Label(control_frame, textvariable=self.status_var).grid(row=0, column=0, columnspan=2, pady=5)

        # IP Input
        ttk.Label(control_frame, text="ESP32 IP:").grid(row=1, column=0, pady=5)
        self.ip_var = tk.StringVar(value="192.168.1.100")
        ttk.Entry(control_frame, textvariable=self.ip_var).grid(row=1, column=1, pady=5)

        # Connect Button
        self.connect_btn = ttk.Button(control_frame, text="เชื่อมต่อ", command=self.connect_websocket)
        self.connect_btn.grid(row=2, column=0, columnspan=2, pady=5)

        # Record Button
        self.record_btn = ttk.Button(control_frame, text="เริ่มบันทึก", command=self.toggle_recording, state='disabled')
        self.record_btn.grid(row=3, column=0, columnspan=2, pady=5)

        # Data Preview
        self.preview_text = tk.Text(control_frame, height=8, width=40)
        self.preview_text.grid(row=4, column=0, columnspan=2, pady=5)

        # Plot Frame
        self.plot_frame = ttk.Frame(main_frame)
        self.plot_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def update_plot(self):
        if len(self.times) > 0:
            self.line_y.set_data(list(self.times), list(self.y_angles))
            self.line_z.set_data(list(self.times), list(self.z_angles))
            
            # ปรับขอบเขตกราฟอัตโนมัติ
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw()

    def connect_websocket(self):
        if self.ws is None:
            try:
                websocket.enableTrace(True)
                self.ws = websocket.WebSocketApp(
                    f"ws://{self.ip_var.get()}:81/",
                    on_message=self.on_message,
                    on_error=self.on_error,
                    on_close=self.on_close,
                    on_open=self.on_open
                )
                threading.Thread(target=self.ws.run_forever).start()
            except Exception as e:
                self.status_var.set(f"สถานะ: เกิดข้อผิดพลาด - {str(e)}")
        else:
            self.ws.close()
            self.ws = None
            self.connect_btn.config(text="เชื่อมต่อ")
            self.record_btn.config(state='disabled')
            self.status_var.set("สถานะ: ไม่ได้เชื่อมต่อ")

    def toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.record_btn.config(text="หยุดบันทึก")
            self.status_var.set("สถานะ: กำลังบันทึกข้อมูล")
            self.start_time = time.time()
            # เคลียร์ข้อมูลกราฟเก่า
            self.times.clear()
            self.y_angles.clear()
            self.z_angles.clear()
        else:
            self.is_recording = False
            self.record_btn.config(text="เริ่มบันทึก")
            self.save_data()
            self.save_plot()
            self.status_var.set("สถานะ: บันทึกข้อมูลเสร็จสิ้น")

    def save_plot(self):
        plot_filename = f"AUV_plot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        self.fig.savefig(plot_filename)
        print(f"Plot saved to {plot_filename}")

    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            data['timestamp'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            
            if self.is_recording:
                self.data.append(data)
                
                # อัพเดตข้อมูลสำหรับกราฟ
                current_time = time.time() - self.start_time
                self.times.append(current_time)
                self.y_angles.append(data['angleY'])
                self.z_angles.append(data['angleZ'])
                
                # อัพเดตกราฟ
                self.update_plot()
            
            # Update preview
            preview = f"Angle Y: {data['angleY']:.2f}\n"
            preview += f"Angle Z: {data['angleZ']:.2f}\n"
            preview += f"kP_Y: {data['kP_Y']:.2f}, kI_Y: {data['kI_Y']:.2f}, kD_Y: {data['kD_Y']:.2f}\n"
            preview += f"kP_Z: {data['kP_Z']:.2f}, kI_Z: {data['kI_Z']:.2f}, kD_Z: {data['kD_Z']:.2f}\n"
            
            self.preview_text.delete(1.0, tk.END)
            self.preview_text.insert(tk.END, preview)
            
        except json.JSONDecodeError:
            print(f"Invalid JSON received: {message}")

    def on_error(self, ws, error):
        print(f"Error: {error}")
        self.status_var.set(f"สถานะ: เกิดข้อผิดพลาด - {str(error)}")

    def on_close(self, ws, close_status_code, close_msg):
        print("WebSocket connection closed")
        self.status_var.set("สถานะ: การเชื่อมต่อถูกปิด")
        self.record_btn.config(state='disabled')
        self.connect_btn.config(text="เชื่อมต่อ")
        self.ws = None

    def on_open(self, ws):
        print("WebSocket connection established")
        self.status_var.set("สถานะ: เชื่อมต่อสำเร็จ")
        self.connect_btn.config(text="ยกเลิกการเชื่อมต่อ")
        self.record_btn.config(state='normal')

    def save_data(self):
        if self.data:
            df = pd.DataFrame(self.data)
            filename = f"AUV_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            df.to_csv(filename, index=False)
            print(f"Data saved to {filename}")
            self.data = []  # Clear the data after saving

    def on_closing(self):
        if self.ws is not None:
            self.ws.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    logger = AUVLogger()
    logger.run()