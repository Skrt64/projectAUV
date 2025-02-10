import asyncio
import json
import websockets
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from collections import deque
import threading

class AUVMonitor:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("AUV PID Monitor")
        self.root.geometry("1200x800")
        
        # Data storage for plotting
        self.buffer_size = 200
        self.time_data = deque(maxlen=self.buffer_size)
        self.angleY_data = deque(maxlen=self.buffer_size)
        self.angleZ_data = deque(maxlen=self.buffer_size)
        
        # Websocket connection status
        self.ws = None
        self.is_connected = False
        self.is_running = False
        
        self.setup_gui()
        self.setup_websocket_thread()
        
    def setup_gui(self):
        # Create main frames
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.pack(side=tk.LEFT, padx=10, pady=10, fill=tk.Y)
        
        self.graph_frame = ttk.Frame(self.root)
        self.graph_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill=tk.BOTH, expand=True)
        
        # Connection controls
        conn_frame = ttk.LabelFrame(self.control_frame, text="Connection")
        conn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(conn_frame, text="ESP32 IP:").pack(padx=5, pady=2)
        self.ip_var = tk.StringVar(value="192.168.1.100")
        self.ip_entry = ttk.Entry(conn_frame, textvariable=self.ip_var)
        self.ip_entry.pack(padx=5, pady=2)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(padx=5, pady=5)
        
        # Control buttons
        ctrl_frame = ttk.LabelFrame(self.control_frame, text="Control")
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.start_btn = ttk.Button(ctrl_frame, text="Start", command=self.start_auv)
        self.start_btn.pack(padx=5, pady=5)
        
        self.stop_btn = ttk.Button(ctrl_frame, text="Stop", command=self.stop_auv)
        self.stop_btn.pack(padx=5, pady=5)
        
        # PID controls
        pid_frame = ttk.LabelFrame(self.control_frame, text="PID Parameters")
        pid_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Y-axis PID
        ttk.Label(pid_frame, text="Y Axis (Pitch)").pack(padx=5, pady=2)
        self.kp_y = self.create_pid_entry(pid_frame, "Kp:", "2.0")
        self.ki_y = self.create_pid_entry(pid_frame, "Ki:", "0.1")
        self.kd_y = self.create_pid_entry(pid_frame, "Kd:", "0.5")
        
        # Z-axis PID
        ttk.Label(pid_frame, text="Z Axis (Roll)").pack(padx=5, pady=2)
        self.kp_z = self.create_pid_entry(pid_frame, "Kp:", "2.0")
        self.ki_z = self.create_pid_entry(pid_frame, "Ki:", "0.1")
        self.kd_z = self.create_pid_entry(pid_frame, "Kd:", "0.5")
        
        self.update_pid_btn = ttk.Button(pid_frame, text="Update PID", command=self.update_pid)
        self.update_pid_btn.pack(padx=5, pady=5)
        
        # Setup graphs
        self.setup_plots()
        
    def create_pid_entry(self, parent, label, default):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(frame, text=label, width=5).pack(side=tk.LEFT)
        entry = ttk.Entry(frame, width=10)
        entry.insert(0, default)
        entry.pack(side=tk.LEFT, padx=5)
        return entry
        
    def setup_plots(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, self.graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.ax1.set_title('Angle Y (Pitch)')
        self.ax2.set_title('Angle Z (Roll)')
        
        self.line1, = self.ax1.plot([], [], 'b-', label='Angle Y')
        self.line2, = self.ax2.plot([], [], 'r-', label='Angle Z')
        
        self.ax1.set_ylim(-90, 90)
        self.ax2.set_ylim(-90, 90)
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax1.legend()
        self.ax2.legend()
        
    async def websocket_loop(self):
        while True:
            try:
                if self.is_connected:
                    uri = f"ws://{self.ip_var.get()}:81"
                    async with websockets.connect(uri) as websocket:
                        self.ws = websocket
                        while self.is_connected:
                            try:
                                message = await websocket.recv()
                                data = json.loads(message)
                                
                                # Update data for plotting
                                self.angleY_data.append(data['angleY'])
                                self.angleZ_data.append(data['angleZ'])
                                self.time_data.append(len(self.time_data))
                                
                                # Update plot
                                self.update_plot()
                                
                            except websockets.exceptions.ConnectionClosed:
                                break
                                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                print(f"WebSocket error: {e}")
                self.is_connected = False
                await asyncio.sleep(1)
                
    def setup_websocket_thread(self):
        def run_async_loop():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.websocket_loop())
            
        self.ws_thread = threading.Thread(target=run_async_loop, daemon=True)
        self.ws_thread.start()
        
    def update_plot(self):
        if len(self.time_data) > 0:
            self.line1.set_data(range(len(self.angleY_data)), self.angleY_data)
            self.line2.set_data(range(len(self.angleZ_data)), self.angleZ_data)
            
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            self.canvas.draw_idle()
            
    def toggle_connection(self):
        if not self.is_connected:
            self.is_connected = True
            self.connect_btn.configure(text="Disconnect")
        else:
            self.is_connected = False
            self.connect_btn.configure(text="Connect")
            
    async def send_command(self, command):
        if self.ws:
            try:
                await self.ws.send(json.dumps(command))
            except Exception as e:
                print(f"Error sending command: {e}")
                
    def start_auv(self):
        asyncio.run(self.send_command({"command": "start"}))
        self.is_running = True
        
    def stop_auv(self):
        asyncio.run(self.send_command({"command": "stop"}))
        self.is_running = False
        
    def update_pid(self):
        pid_values = {
            "pid": {
                "kp_y": float(self.kp_y.get()),
                "ki_y": float(self.ki_y.get()),
                "kd_y": float(self.kd_y.get()),
                "kp_z": float(self.kp_z.get()),
                "ki_z": float(self.ki_z.get()),
                "kd_z": float(self.kd_z.get())
            }
        }
        asyncio.run(self.send_command(pid_values))
        
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = AUVMonitor()
    app.run()