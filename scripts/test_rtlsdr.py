"""Test RTL-SDR V4 device detection and data reception"""
import ctypes
import sys
import os

DLL_PATH = r"C:\SDRSharp\tmp\x64\rtlsdr.dll"

def main():
    print("=== RTL-SDR V4 Test ===")
    print()
    
    # Load DLL
    if not os.path.exists(DLL_PATH):
        print(f"ERROR: DLL not found: {DLL_PATH}")
        return 1
    
    dll = ctypes.windll.LoadLibrary(DLL_PATH)
    print(f"[OK] DLL loaded: {DLL_PATH}")
    
    # Get device count
    count = dll.rtlsdr_get_device_count()
    print(f"[OK] Devices found: {count}")
    
    if count == 0:
        print("\nNo RTL-SDR devices detected!")
        print("Check:")
        print("  1. Device is connected")
        print("  2. WinUSB driver installed (use Zadig)")
        return 1
    
    # Get device info
    for i in range(count):
        print(f"\n--- Device {i} ---")
        name = ctypes.create_string_buffer(256)
        manufacturer = ctypes.create_string_buffer(256)
        serial = ctypes.create_string_buffer(256)
        
        result = dll.rtlsdr_get_device_usb_strings(i, manufacturer, serial, name)
        if result == 0:
            print(f"  Name: {name.value.decode('utf-8', errors='replace')}")
            print(f"  Manufacturer: {manufacturer.value.decode('utf-8', errors='replace')}")
            print(f"  Serial: {serial.value.decode('utf-8', errors='replace')}")
        else:
            print(f"  Could not read strings (error {result})")
        
        # Try to open device
        dev = ctypes.c_void_p()
        result = dll.rtlsdr_open(ctypes.byref(dev), i)
        if result != 0 or not dev:
            print(f"  Open failed (error {result})")
            continue
        
        print(f"  [OK] Device opened")
        
        # Get center freq
        freq = dll.rtlsdr_get_center_freq(dev)
        print(f"  Center freq: {freq} Hz ({freq/1e6:.2f} MHz)")
        
        # Get sample rate
        sr = dll.rtlsdr_get_sample_rate(dev)
        print(f"  Sample rate: {sr} S/s")
        
        # Get tuner type
        tuner_type = dll.rtlsdr_get_tuner_type(dev)
        tuner_names = {0: "Unknown", 1: "E4000", 2: "FC0012", 3: "FC0013", 4: "FC2580", 5: "R820T", 6: "R828D"}
        print(f"  Tuner: {tuner_names.get(tuner_type, f'Type {tuner_type}')}")
        
        # Set freq to FM broadcast (100 MHz)
        dll.rtlsdr_set_center_freq(dev, 100_000_000)
        
        # Get some samples
        buf_len = 4096
        buf = ctypes.create_string_buffer(buf_len)
        n_read = ctypes.c_uint32(0)
        
        result = dll.rtlsdr_read_sync(dev, buf, buf_len, ctypes.byref(n_read))
        if result == 0 and n_read.value > 0:
            print(f"  [OK] Received {n_read.value} samples ({buf_len // 2} complex samples)")
            # Show first few I/Q values
            samples = list(buf.raw[:32])
            iq_pairs = [(samples[j] - 127.5, samples[j+1] - 127.5) for j in range(0, min(16, len(samples)), 2)]
            print(f"  First 8 I/Q pairs: {iq_pairs}")
        else:
            print(f"  Read failed (error {result}, read {n_read.value})")
        
        # Close device
        dll.rtlsdr_close(dev)
        print(f"  [OK] Device closed")
    
    print("\n=== Test Complete ===")
    return 0

if __name__ == "__main__":
    sys.exit(main())
