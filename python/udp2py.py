import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sgnl
import socket
import signal
import sys
from collections import deque
import threading

# UDP Configuration
udp_ip = "192.168.1.69"
udp_port = 12345



#udp_ip = "172.174.210.23"
#udp_port = 8000

fs = 100  # Sampling frequency
buffer_size = 10 * fs  # Buffer size for 5 seconds of data

# Butterworth bandpass filter (0.5-4 Hz)
[b_ppt, a_ppt] = sgnl.butter(1, [(0.5 * 2) / fs, (5 * 2) / fs], 'band')
[b_ecg, a_ecg] = sgnl.butter(5, [(5 * 2) / fs, (45 * 2) / fs], 'band')

zi_ppt = sgnl.lfilter_zi(b_ppt, a_ppt)
zi_ecg = sgnl.lfilter_zi(b_ecg, a_ecg)


# Set up UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reuse address
sock.bind((udp_ip, udp_port))
print(f"Listening on {udp_ip}:{udp_port}...")

# Cleanup function
def cleanup_and_exit(signum, frame):
    print("Cleaning up and exiting...")
    sock.close()
    sys.exit(0)

def calculate_hrv(peaks, fs=50):
    rr_intervals = np.diff(peaks) / fs
    if len(rr_intervals) < 2:
        return 0
    hrv = 60 / np.mean(rr_intervals)
    return hrv

def calculate_spo2(ir, red):
    """
    Calculate SpO2 using AC/DC ratios for red and IR signals.
    Parameters:
        ir (array-like): Infrared light intensity signal.
        red (array-like): Red light intensity signal.
        fs (float): Sampling frequency of the signals.
    Returns:
        float: Estimated SpO2 value.
    """
    # Parameters for bandpass filter (typical pulsatile frequencies: 0.5–4 Hz)
   
    # Filter signals to extract AC components
    ir_ac = ir
    red_ac = red
    
    # Calculate DC components (baseline)
    ir_dc = np.mean(ir)
    red_dc = np.mean(red)
    
    # Avoid division by zero
    if ir_dc == 0 or red_dc == 0:
        return 0
    
    # Calculate AC/DC ratios
    ac_dc_ir = np.std(ir_ac) / ir_dc
    ac_dc_red = np.std(red_ac) / red_dc
    
    # Compute R value
    r = ac_dc_red / ac_dc_ir
    
    # Empirical calibration formula (example values; adjust based on data)
    spo2 = 110 - 25 * r  # This formula can be replaced by a more precise calibration.
    
    # Clamp SpO2 to a valid range
    spo2 = max(0, min(100, spo2))
    
    return spo2

def preprocess(data_buffer, fs=50):
    if len(data_buffer) < 2:
        return data_buffer  # Not enough data to preprocess
    x = np.arange(len(data_buffer))
    trend = np.polyfit(x, data_buffer, 1)[0] * x
    detrended_data = data_buffer - trend
    if np.std(detrended_data) != 0:
        normalized_data = (detrended_data - np.mean(detrended_data)) / np.std(detrended_data)
    else:
        normalized_data = detrended_data - np.mean(detrended_data)
    return normalized_data

signal.signal(signal.SIGINT, cleanup_and_exit)  # Ctrl+C
signal.signal(signal.SIGTERM, cleanup_and_exit)  # Termination signal

# Circular buffers for IR and Red signals
data_buffer_ir = deque(maxlen=buffer_size)
data_buffer_red = deque(maxlen=buffer_size)
data_buffer_ecg = deque(maxlen=buffer_size)

signals = []
start_time = time.time()

peaks = [0]

# Prepare for live plotting
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 8))  # Create two subplots

line_ir, = ax1.plot([], [], lw=2, label="IR Signal")
line_red, = ax1.plot([], [], lw=2, color='k', label="Red Signal")
line_peaks, = ax1.plot([], [], lw=2, marker='o', color='r', label="Peaks")
#ax1.set_ylim(-100, 100)
ax1.set_xlim(0, buffer_size)
ax1.set_title("PPT")

line_hrv, = ax2.plot([], [], lw=2, color='b', label="Heart Rate Variability (HRV)")
ax2.set_title("Heart Rate Variability (HRV)")
ax2.set_ylabel("HRV (BPM)")
ax2.set_ylim(60, 100)
ax2.set_xlim(0, buffer_size)
ax2.hlines(60, 0, 60, colors='r', linestyles='dashed', label="Target")

find_peaks_params = {'height': 0.5, 'distance': fs // 2}
counter = 0
hrv_values = deque(maxlen=60)
spo2_values = deque(maxlen=60)

# Define thread for data reception
def receive_data():
    global data_buffer_ir, data_buffer_red
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            decoded_data = data.decode('utf-8').strip()
            try:
                ir_value, red_value, ecg_value = map(float, decoded_data.split(","))
                data_buffer_ir.append(ir_value)
                data_buffer_red.append(red_value)
                data_buffer_ecg.append(ecg_value)
            except ValueError:
                print(f"Malformed data: {decoded_data}")
    except KeyboardInterrupt:
        print("Streaming stopped.")

# Define thread for processing
def process_data():
    global data_buffer_ir, data_buffer_red, hrv_values, peaks, data_buffer_ir_np, data_buffer_red_np, spo2, spo2_values, data_buffer_ecg, data_buffer_ecg_np
    try:
        while True:
            
            if len(data_buffer_ir) > 1:
                data_buffer_ir_np = np.array(data_buffer_ir)
                data_buffer_red_np = np.array(data_buffer_red)
                data_buffer_ecg_np = np.array(data_buffer_ecg)

                data_buffer_ir_np, zi_new = sgnl.lfilter(b_ppt, a_ppt, data_buffer_ir_np, zi=zi_ppt * data_buffer_ir_np[0])[-buffer_size:]
                #data_buffer_red_np, zi_new = sgnl.lfilter(b_ppt, a_ppt, data_buffer_red_np, zi=zi_ppt * data_buffer_ir_np[0])[-buffer_size:]

                data_buffer_ecg_np, zi_new = sgnl.lfilter(b_ecg, a_ecg, data_buffer_ecg_np, zi=zi_ecg * data_buffer_ecg_np[0])[-buffer_size:]
                #data_buffer_red_np, zi_new = sgnl.lfilter(b, a, data_buffer_red_np, zi=zi * data_buffer_ir_np[0])[-buffer_size:]
                ## Preprocess and find peaks
                #spo2 = calculate_spo2(data_buffer_ir_np, data_buffer_red_np)
                #spo2_values.append(spo2)
                data_buffer_ir_np = preprocess(data_buffer_ir_np)
                #data_buffer_red_np = preprocess(data_buffer_red_np)
                #peaks, _ = sgnl.find_peaks(data_buffer_red_np, **find_peaks_params)

                # Calculate HRV
                #if len(peaks) > 1:
                #    hrv = calculate_hrv(peaks)
                #    hrv_values.append(hrv)
            time.sleep(0.01)  # Sleep to prevent overloading the CPU

    except KeyboardInterrupt:
        print("Processing stopped.")

# Start threads for receiving data and processing
thread_receive = threading.Thread(target=receive_data, daemon=True)
thread_process = threading.Thread(target=process_data, daemon=True)

thread_receive.start()
thread_process.start()

data_buffer_ir_np = np.array(data_buffer_ir)
data_buffer_red_np = np.array(data_buffer_red)

# Main loop for plotting
try:
    while True:
        if len(data_buffer_ir) > 1:

            # Update plot data
            line_ir.set_data(range(len(data_buffer_ir_np)), data_buffer_ir_np)
            #line_red.set_data(range(len(data_buffer_red_np)), data_buffer_red_np)
            
            #line_peaks.set_data(peaks, data_buffer_red_np[peaks])
            ax1.set_ylim(np.min([data_buffer_ir_np]), np.max([data_buffer_ir_np]))
            #if len(hrv_values) > 1:
            #    hrv_values_np = np.array(spo2_values)
            #    line_hrv.set_data(range(len(spo2_values)), spo2_values)
            line_hrv.set_data(range(len(data_buffer_ecg_np)), data_buffer_ecg_np)
            ax2.set_ylim(np.min([data_buffer_ecg_np]), np.max([data_buffer_ecg_np]))

            plt.pause(0.05)  # Reduced pause to improve performance
except KeyboardInterrupt:
    print("Program exited.")
finally:
    sock.close()
    plt.ioff()
    plt.show()
