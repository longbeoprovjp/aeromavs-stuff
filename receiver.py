#!/usr/bin/env python3
"""
HX-1 Data Receiver - Laptop/Host PC Script
Receives sensor data from Raspberry Pi via TCP/IP over Ethernet

This script acts as a TCP SERVER that waits for the Raspberry Pi to connect.
Once connected, it receives real-time data and displays it with live plots.

References:
- Python socket programming: https://docs.python.org/3/library/socket.html
- Socket tutorial: https://realpython.com/python-sockets/
- matplotlib animation: https://matplotlib.org/stable/api/animation_api.html
"""

import socket
import json
import csv
import time
from datetime import datetime
from pathlib import Path

# Try to import matplotlib for plotting
# Reference: https://matplotlib.org/
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    PLOTTING_AVAILABLE = True
except ImportError:
    print("matplotlib not installed. Install with: pip install matplotlib")
    PLOTTING_AVAILABLE = False

# =============================================================================
# CONFIGURATION
# =============================================================================

# Network Configuration
# The laptop acts as a TCP SERVER - it waits for the Raspberry Pi to connect
HOST = '0.0.0.0'  # Listen on all network interfaces (allows any IP to connect)
PORT = 5555       # Must match the port in Raspberry Pi script

# Data Storage
DATA_DIR = Path("./hx1_received_data")
DATA_DIR.mkdir(parents=True, exist_ok=True)

# Plot Configuration
MAX_PLOT_POINTS = 500  # Maximum number of points to show on plot (rolling window)
UPDATE_INTERVAL = 50   # Milliseconds between plot updates

# =============================================================================
# TCP SERVER FUNCTIONS
# =============================================================================

def start_server(host, port):
    """
    Start TCP server and wait for Raspberry Pi to connect.
    
    Server socket process:
    1. Create socket
    2. Bind to address (host:port)
    3. Listen for incoming connections
    4. Accept connection (blocks until client connects)
    
    Key difference from client:
    - SERVER: socket.bind() → socket.listen() → socket.accept()
    - CLIENT: socket.connect()
    
    Reference: https://docs.python.org/3/library/socket.html#example
    Tutorial: https://realpython.com/python-sockets/#echo-server
    
    Args:
        host: IP address to bind to ('0.0.0.0' = all interfaces)
        port: Port number to listen on
        
    Returns:
        tuple: (server_socket, client_socket) or (None, None) on error
    """
    try:
        # Create TCP socket
        # AF_INET = IPv4, SOCK_STREAM = TCP protocol
        # Reference: https://docs.python.org/3/library/socket.html#socket.socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Set socket options
        # SO_REUSEADDR allows reusing the port immediately after closing
        # Prevents "Address already in use" error when restarting quickly
        # Reference: https://docs.python.org/3/library/socket.html#socket.socket.setsockopt
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind socket to address
        # This reserves the port on this computer
        # Reference: https://docs.python.org/3/library/socket.html#socket.socket.bind
        server_socket.bind((host, port))
        
        # Listen for connections (queue up to 1 connection)
        # Reference: https://docs.python.org/3/library/socket.html#socket.socket.listen
        server_socket.listen(1)
        
        print(f"✓ Server listening on {host}:{port}")
        print("Waiting for Raspberry Pi to connect...")
        print("(Make sure Raspberry Pi script is running)")
        
        # Accept incoming connection (THIS BLOCKS until someone connects)
        # Returns: (client_socket, client_address)
        # Reference: https://docs.python.org/3/library/socket.html#socket.socket.accept
        client_socket, client_address = server_socket.accept()
        
        print(f"✓ Raspberry Pi connected from {client_address[0]}:{client_address[1]}")
        return server_socket, client_socket
        
    except OSError as e:
        if e.errno == 48 or e.errno == 98:  # Address already in use
            print(f"ERROR: Port {port} is already in use")
            print("Solution: Wait a few seconds or use a different port")
        else:
            print(f"Server error: {e}")
        return None, None
    
    except Exception as e:
        print(f"Unexpected error: {e}")
        return None, None

def receive_data(client_socket, timeout=1.0):
    """
    Receive one line of JSON data from Raspberry Pi.
    
    Data format: {"key": value, ...}\n
    Each data packet is terminated by a newline character.
    
    Reference: https://docs.python.org/3/library/socket.html#socket.socket.recv
    
    Args:
        client_socket: Connected socket from accept()
        timeout: Timeout in seconds (None = block forever)
        
    Returns:
        dict: Parsed JSON data, or None if no data/error
    """
    try:
        # Set timeout to avoid blocking forever
        client_socket.settimeout(timeout)
        
        # Receive data (up to 4096 bytes at a time)
        # recv() returns bytes, which we decode to string
        # Reference: https://docs.python.org/3/library/socket.html#socket.socket.recv
        data_bytes = client_socket.recv(4096)
        
        if not data_bytes:
            # Empty data means connection closed
            return None
        
        # Decode bytes to string
        data_string = data_bytes.decode('utf-8')
        
        # Split by newlines (in case multiple packets arrived)
        lines = data_string.strip().split('\n')
        
        # Parse each line as JSON
        # Reference: https://docs.python.org/3/library/json.html#json.loads
        parsed_data = []
        for line in lines:
            if line:  # Skip empty lines
                try:
                    data_dict = json.loads(line)
                    parsed_data.append(data_dict)
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                    print(f"  Received: {line}")
        
        return parsed_data if parsed_data else None
        
    except socket.timeout:
        # Timeout is normal - just means no data arrived in timeout period
        return None
    
    except ConnectionResetError:
        print("Connection reset by Raspberry Pi")
        return None
    
    except Exception as e:
        print(f"Receive error: {e}")
        return None

# =============================================================================
# DATA STORAGE
# =============================================================================

def setup_csv_logging(test_id):
    """
    Create CSV file for logging received data.
    
    Reference: https://docs.python.org/3/library/csv.html
    
    Args:
        test_id: Test identifier for filename
        
    Returns:
        tuple: (file_handle, csv_writer)
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = DATA_DIR / f"received_{test_id}_{timestamp}.csv"
    
    # Open file in write mode
    csv_file = open(filename, 'w', newline='')
    
    # Create CSV writer
    # Reference: https://docs.python.org/3/library/csv.html#csv.writer
    csv_writer = csv.writer(csv_file)
    
    # Write header row
    header = ['timestamp', 'time_ms', 'chamber_pressure_psi', 
              'tank_pressure_psi', 'thrust_n', 'chamber_temp_c']
    csv_writer.writerow(header)
    
    print(f"Logging received data to: {filename}")
    return csv_file, csv_writer, filename

def save_data_to_csv(csv_writer, data_dict):
    """
    Save one data packet to CSV.
    
    Args:
        csv_writer: CSV writer object
        data_dict: Dictionary containing sensor data
    """
    try:
        row = [
            data_dict.get('timestamp', ''),
            data_dict.get('time_ms', 0),
            data_dict.get('chamber_pressure_psi', 0),
            data_dict.get('tank_pressure_psi', 0),
            data_dict.get('thrust_n', 0),
            data_dict.get('chamber_temp_c', 0)
        ]
        csv_writer.writerow(row)
    except Exception as e:
        print(f"CSV write error: {e}")

# =============================================================================
# DATA VISUALIZATION
# =============================================================================

class RealTimePlotter:
    """
    Real-time data visualization using matplotlib.
    
    Creates 4 subplots showing:
    1. Chamber pressure vs time
    2. Tank pressure vs time
    3. Thrust vs time
    4. Temperature vs time
    
    Reference: https://matplotlib.org/stable/api/animation_api.html
    Tutorial: https://matplotlib.org/stable/gallery/animation/simple_anim.html
    """
    
    def __init__(self, max_points=500):
        """
        Initialize plot with empty data arrays.
        
        Args:
            max_points: Maximum number of points to display (rolling window)
        """
        self.max_points = max_points
        
        # Data storage (these will be updated as data arrives)
        self.time_data = []
        self.chamber_pressure_data = []
        self.tank_pressure_data = []
        self.thrust_data = []
        self.temp_data = []
        
        # Create figure with 4 subplots
        # Reference: https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.subplots.html
        self.fig, self.axes = plt.subplots(4, 1, figsize=(10, 8))
        self.fig.suptitle('HX-1 Real-Time Sensor Data', fontsize=14, fontweight='bold')
        
        # Create line objects for each subplot
        # Reference: https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.plot.html
        self.line_chamber, = self.axes[0].plot([], [], 'r-', linewidth=2, label='Chamber Pressure')
        self.line_tank, = self.axes[1].plot([], [], 'b-', linewidth=2, label='Tank Pressure')
        self.line_thrust, = self.axes[2].plot([], [], 'g-', linewidth=2, label='Thrust')
        self.line_temp, = self.axes[3].plot([], [], 'm-', linewidth=2, label='Temperature')
        
        # Configure each subplot
        self.axes[0].set_ylabel('Pressure (PSI)')
        self.axes[0].set_title('Chamber Pressure')
        self.axes[0].grid(True, alpha=0.3)
        self.axes[0].legend()
        
        self.axes[1].set_ylabel('Pressure (PSI)')
        self.axes[1].set_title('Tank Pressure')
        self.axes[1].grid(True, alpha=0.3)
        self.axes[1].legend()
        
        self.axes[2].set_ylabel('Force (N)')
        self.axes[2].set_title('Thrust')
        self.axes[2].grid(True, alpha=0.3)
        self.axes[2].legend()
        
        self.axes[3].set_xlabel('Time (s)')
        self.axes[3].set_ylabel('Temp (°C)')
        self.axes[3].set_title('Chamber Temperature')
        self.axes[3].grid(True, alpha=0.3)
        self.axes[3].legend()
        
        plt.tight_layout()
    
    def update_data(self, data_dict):
        """
        Add new data point to plot arrays.
        
        Args:
            data_dict: Dictionary containing sensor readings
        """
        # Extract data from dictionary
        time_s = data_dict.get('time_ms', 0) / 1000.0  # Convert ms to seconds
        chamber_p = data_dict.get('chamber_pressure_psi', 0)
        tank_p = data_dict.get('tank_pressure_psi', 0)
        thrust = data_dict.get('thrust_n', 0)
        temp = data_dict.get('chamber_temp_c', 0)
        
        # Append to data arrays
        self.time_data.append(time_s)
        self.chamber_pressure_data.append(chamber_p)
        self.tank_pressure_data.append(tank_p)
        self.thrust_data.append(thrust)
        self.temp_data.append(temp)
        
        # Keep only recent data (rolling window)
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.chamber_pressure_data.pop(0)
            self.tank_pressure_data.pop(0)
            self.thrust_data.pop(0)
            self.temp_data.pop(0)
    
    def animate(self, frame):
        """
        Animation update function called by FuncAnimation.
        
        Reference: https://matplotlib.org/stable/api/animation_api.html
        
        Args:
            frame: Frame number (not used, but required by FuncAnimation)
            
        Returns:
            tuple: Line objects that were updated
        """
        if not self.time_data:
            return self.line_chamber, self.line_tank, self.line_thrust, self.line_temp
        
        # Update line data
        # Reference: https://matplotlib.org/stable/api/_as_gen/matplotlib.lines.Line2D.html#matplotlib.lines.Line2D.set_data
        self.line_chamber.set_data(self.time_data, self.chamber_pressure_data)
        self.line_tank.set_data(self.time_data, self.tank_pressure_data)
        self.line_thrust.set_data(self.time_data, self.thrust_data)
        self.line_temp.set_data(self.time_data, self.temp_data)
        
        # Update axis limits to show all data
        if len(self.time_data) > 1:
            for i, data in enumerate([self.chamber_pressure_data, 
                                     self.tank_pressure_data,
                                     self.thrust_data, 
                                     self.temp_data]):
                # Set x-axis limits (time)
                self.axes[i].set_xlim(min(self.time_data), max(self.time_data))
                
                # Set y-axis limits with 10% padding
                if data:
                    y_min, y_max = min(data), max(data)
                    y_range = y_max - y_min
                    padding = 0.1 * y_range if y_range > 0 else 1
                    self.axes[i].set_ylim(y_min - padding, y_max + padding)
        
        return self.line_chamber, self.line_tank, self.line_thrust, self.line_temp

# =============================================================================
# STATISTICS CALCULATION
# =============================================================================

def calculate_statistics(data_arrays):
    """
    Calculate and print test statistics.
    
    Args:
        data_arrays: Dictionary of data arrays
    """
    print("\n" + "="*60)
    print("TEST STATISTICS")
    print("="*60)
    
    labels = {
        'chamber_pressure': ('Chamber Pressure', 'PSI'),
        'tank_pressure': ('Tank Pressure', 'PSI'),
        'thrust': ('Thrust', 'N'),
        'temperature': ('Temperature', '°C')
    }
    
    for key, (name, unit) in labels.items():
        data = data_arrays.get(key, [])
        if data:
            avg = sum(data) / len(data)
            max_val = max(data)
            min_val = min(data)
            
            print(f"\n{name}:")
            print(f"  Average: {avg:.2f} {unit}")
            print(f"  Maximum: {max_val:.2f} {unit}")
            print(f"  Minimum: {min_val:.2f} {unit}")
    
    # Calculate total impulse (area under thrust curve)
    # Total Impulse = ∫ F dt ≈ Σ F * Δt
    thrust_data = data_arrays.get('thrust', [])
    time_data = data_arrays.get('time', [])
    
    if len(thrust_data) > 1 and len(time_data) > 1:
        # Calculate time step
        dt = (time_data[-1] - time_data[0]) / (len(time_data) - 1)
        # Trapezoidal integration
        total_impulse = sum(thrust_data) * dt
        print(f"\nTotal Impulse: {total_impulse:.2f} N·s")
        print(f"  (Target from CDR: 1000 N·s)")
    
    print("="*60 + "\n")

# =============================================================================
# MAIN PROGRAM
# =============================================================================

def main():
    """
    Main program loop.
    
    1. Start TCP server
    2. Wait for Raspberry Pi connection
    3. Receive and display data
    4. Save to CSV
    5. Show live plots (if matplotlib available)
    """
    print("="*60)
    print("HX-1 DATA RECEIVER - Host PC")
    print("="*60)
    print(f"\nConfiguration:")
    print(f"  Listening on: {HOST}:{PORT}")
    print(f"  Data directory: {DATA_DIR}")
    
    # Start server
    server_socket, client_socket = start_server(HOST, PORT)
    if not server_socket:
        print("Failed to start server. Exiting.")
        return
    
    # Setup CSV logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    test_id = f"HX1_TEST_{timestamp}"
    csv_file, csv_writer, csv_filename = setup_csv_logging(test_id)
    
    # Setup plotting (if available)
    plotter = None
    if PLOTTING_AVAILABLE:
        plotter = RealTimePlotter(max_points=MAX_PLOT_POINTS)
        print("✓ Real-time plotting enabled")
    else:
        print("⚠ Plotting disabled (matplotlib not installed)")
    
    # Data collection arrays for statistics
    all_data = {
        'time': [],
        'chamber_pressure': [],
        'tank_pressure': [],
        'thrust': [],
        'temperature': []
    }
    
    print("\n" + "="*60)
    print("RECEIVING DATA")
    print("="*60)
    print("Press Ctrl+C to stop\n")
    
    # Animation setup (if plotting enabled)
    if plotter:
        # Create animation that calls animate() every UPDATE_INTERVAL ms
        # Reference: https://matplotlib.org/stable/api/animation_api.html
        ani = animation.FuncAnimation(
            plotter.fig,
            plotter.animate,
            interval=UPDATE_INTERVAL,
            blit=True,
            cache_frame_data=False
        )
        
        # Show plot window (non-blocking)
        plt.ion()  # Interactive mode on
        plt.show(block=False)
    
    packet_count = 0
    last_print_time = time.time()
    
    try:
        while True:
            # Receive data from Raspberry Pi
            data_packets = receive_data(client_socket, timeout=0.1)
            
            if data_packets is None:
                # No data received (timeout or error)
                # Update plot to keep it responsive
                if plotter:
                    plt.pause(0.001)
                continue
            
            # Process each received packet
            for data_dict in data_packets:
                # Save to CSV
                save_data_to_csv(csv_writer, data_dict)
                
                # Update plot
                if plotter:
                    plotter.update_data(data_dict)
                
                # Store for statistics
                all_data['time'].append(data_dict.get('time_ms', 0) / 1000.0)
                all_data['chamber_pressure'].append(data_dict.get('chamber_pressure_psi', 0))
                all_data['tank_pressure'].append(data_dict.get('tank_pressure_psi', 0))
                all_data['thrust'].append(data_dict.get('thrust_n', 0))
                all_data['temperature'].append(data_dict.get('chamber_temp_c', 0))
                
                packet_count += 1
                
                # Print status every 0.5 seconds
                if time.time() - last_print_time > 0.5:
                    print(f"Received {packet_count} packets | "
                          f"Latest: P={data_dict.get('chamber_pressure_psi', 0):.1f} PSI, "
                          f"F={data_dict.get('thrust_n', 0):.1f} N, "
                          f"T={data_dict.get('chamber_temp_c', 0):.0f}°C")
                    last_print_time = time.time()
            
            # Keep plot responsive
            if plotter:
                plt.pause(0.001)
    
    except KeyboardInterrupt:
        print("\n\nStopping data reception...")
    
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print(f"\nReceived {packet_count} total packets")
        
        csv_file.close()
        client_socket.close()
        server_socket.close()
        
        # Show statistics
        if packet_count > 0:
            calculate_statistics(all_data)
        
        print(f"Data saved to: {csv_filename}")
        
        # Keep plot window open
        if plotter:
            print("\nClose the plot window to exit...")
            plt.ioff()  # Turn off interactive mode
            plt.show()  # Block until window closed

if __name__ == "__main__":
    main()
