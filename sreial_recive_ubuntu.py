import serial
import csv
from datetime import datetime
import time
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for servers
import matplotlib.pyplot as plt
import os
import argparse
import threading

class AUVDataLogger:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200, headless=True, save_interval=60):
        self.serial_port = serial.Serial(port, baud_rate, timeout=1)
        self.data = []
        self.columns = [
            'angleY', 'angleZ', 'depth',
            'kP_Y', 'kI_Y', 'kD_Y',
            'kP_Z', 'kI_Z', 'kD_Z',
            'kP_D', 'kI_D', 'kD_D',
            'targetDepth', 'timestamp'
        ]
        
        # Create output directory
        self.output_dir = f'auv_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create CSV file with timestamp in filename
        self.filename = os.path.join(self.output_dir, 'data.csv')
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['local_timestamp'] + self.columns)
        
        self.headless = headless
        self.save_interval = save_interval  # How often to save plots (in seconds)
        self.last_save_time = time.time()
        
        # Setup plotting with three subplots
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 10))
        
        # Angle Y plot
        self.ax1.set_title('Pitch (Angle Y)')
        self.ax1.set_ylabel('Degrees')
        self.ax1.grid(True)
        
        # Angle Z plot
        self.ax2.set_title('Roll (Angle Z)')
        self.ax2.set_ylabel('Degrees')
        self.ax2.grid(True)
        
        # Depth plot
        self.ax3.set_title('Depth')
        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('Meters')
        self.ax3.grid(True)
        
        # Adjust layout to prevent overlap
        plt.tight_layout()
        
        self.time_window = 1000  # Number of samples to show in plot
        self.running = True
        
    def generate_plots(self):
        """Generate static plots from the collected data"""
        if len(self.data) < 2:
            return  # Not enough data to plot

        df = pd.DataFrame(self.data[-self.time_window:], columns=self.columns)
        timestamps = range(len(df))
        
        # Clear previous plots
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        
        # Redraw plots
        self.ax1.plot(timestamps, df['angleY'], 'r-', label='Angle Y')
        self.ax1.set_title('Pitch (Angle Y)')
        self.ax1.set_ylabel('Degrees')
        self.ax1.grid(True)
        self.ax1.legend()
        
        self.ax2.plot(timestamps, df['angleZ'], 'b-', label='Angle Z')
        self.ax2.set_title('Roll (Angle Z)')
        self.ax2.set_ylabel('Degrees')
        self.ax2.grid(True)
        self.ax2.legend()
        
        self.ax3.plot(timestamps, df['depth'], 'g-', label='Depth')
        self.ax3.plot(timestamps, [df['targetDepth'].iloc[-1]] * len(df), 'r--', 
                     label=f'Target Depth: {df["targetDepth"].iloc[-1]:.2f}m')
        self.ax3.set_title('Depth')
        self.ax3.set_ylabel('Meters')
        self.ax3.grid(True)
        self.ax3.legend()
        
        # Save the figure
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plt.savefig(os.path.join(self.output_dir, f'plot_{timestamp}.png'), dpi=150)
        
        # Also save the latest plot as "latest.png" for easy access
        plt.savefig(os.path.join(self.output_dir, 'latest.png'), dpi=150)

    def plot_update_thread(self):
        """Thread function to periodically update plots in headless mode"""
        while self.running:
            current_time = time.time()
            if current_time - self.last_save_time >= self.save_interval:
                self.generate_plots()
                self.last_save_time = current_time
            time.sleep(1)  # Check every second

    def start_logging(self):
        print(f"Starting data logging to {self.filename}")
        print(f"Plots will be saved to {self.output_dir} directory")
        print("Press Ctrl+C to stop logging")
        
        # Start plot update thread if in headless mode
        if self.headless:
            plot_thread = threading.Thread(target=self.plot_update_thread)
            plot_thread.daemon = True
            plot_thread.start()
        
        try:
            while True:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                    if line.startswith('DATA'):
                        # Parse the CSV data
                        try:
                            values = line.split(',')[1:]  # Remove 'DATA' prefix
                            if len(values) == len(self.columns) - 1:  # -1 because timestamp is added by the device
                                # Convert strings to appropriate types
                                values = [float(x) for x in values]
                                values.append(time.time())  # Add timestamp
                                
                                # Save to CSV
                                with open(self.filename, 'a', newline='') as f:
                                    writer = csv.writer(f)
                                    writer.writerow([datetime.now().timestamp()] + values)
                                
                                # Add to data list for plotting
                                self.data.append(values)
                                
                                # Print latest values with better formatting
                                print(f"\rPitch: {values[0]:6.2f}° | Roll: {values[1]:6.2f}° | "
                                     f"Depth: {values[2]:6.2f}m | Target: {values[12]:6.2f}m", end='')
                        except (ValueError, IndexError) as e:
                            print(f"\nError parsing data: {e} - Line: {line}")
                
                time.sleep(0.01)  # Small delay to prevent CPU hogging
                
        except KeyboardInterrupt:
            print("\nLogging stopped by user")
        finally:
            self.running = False
            self.serial_port.close()
            # Generate final plot
            self.generate_plots()
            print(f"\nFinal plots saved to {self.output_dir}")

def main():
    parser = argparse.ArgumentParser(description='AUV Data Logger for Ubuntu Server')
    parser.add_argument('--port', default='/dev/ttyUSB0', 
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, 
                       help='Baud rate (default: 115200)')
    parser.add_argument('--interval', type=int, default=60, 
                       help='Plot save interval in seconds (default: 60)')
    args = parser.parse_args()
    
    logger = AUVDataLogger(port=args.port, baud_rate=args.baud, save_interval=args.interval)
    logger.start_logging()

if __name__ == "__main__":
    main()
