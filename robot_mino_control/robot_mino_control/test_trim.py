#!/usr/bin/env python3
import serial
import time
import sys
import threading

# Konfigurasi Port Serial (Sesuaikan dengan port Arduino Nano 33 BLE kamu)
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"[INFO] Berhasil membuka port {SERIAL_PORT}")
except Exception as e:
    print(f"[EROR] Gagal membuka port: {e}")
    sys.exit(1)

# Variabel Global untuk Kontrol Gerakan
target_w_left = 0.0
target_w_right = 0.0
keep_running = True

def serial_write_loop():
    """Loop thread untuk mengirim data secara konstan (mencegah timeout 500ms)"""
    global target_w_left, target_w_right, keep_running
    while keep_running:
        # Kirim data dengan format polos sesuai "bacaSerialKomunikasi()" Arduino kamu
        command_string = f"{target_w_left:.4f},{target_w_right:.4f}\n"
        try:
            ser.write(command_string.encode('utf-8'))
        except Exception as e:
            print(f"\n[EROR] Gagal menulis ke serial: {e}")
            break
        time.sleep(0.05) # Mengirim data setiap 50ms (20Hz)

# Jalankan thread pengirim data serial
tx_thread = threading.Thread(target=serial_write_loop, daemon=True)
tx_thread.start()

print("\n" + "="*50)
print("       MINO ROBOT - SERIAL TRIM TESTER")
print("="*50)
print("Aturan main:")
print("Masukkan nilai kecepatan dasar dan trim untuk mencari keseimbangan fisik.")
print("Ketik 'stop' atau '0' untuk menghentikan robot.")
print("Ketik 'keluar' untuk menutup program.")
print("="*50)

# Nilai default awal
base_speed = 1.5   # Kecepatan dasar dalam rad/s
trim_left = 1.000
trim_right = 1.000

try:
    while True:
        print(f"\n[STATUS AKTIF] Base Speed: {base_speed} rad/s | Trim L: {trim_left:.3f} | Trim R: {trim_right:.3f}")
        user_input = input("Ubah nilai (Format: base_speed,trim_L,trim_R) -> ").strip().lower()
        
        if user_input == 'keluar':
            break
        elif user_input in ['stop', '0']:
            target_w_left = 0.0
            target_w_right = 0.0
            print("[INFO] Robot Dihentikan.")
            continue
            
        try:
            parts = user_input.split(',')
            if len(parts) == 3:
                base_speed = float(parts[0])
                trim_left = float(parts[1])
                trim_right = float(parts[2])
                
                # Hitung kecepatan akhir setelah dikalikan faktor trim
                target_w_left = base_speed * trim_left
                target_w_right = base_speed * trim_right
                
                print(f"[OK] Mengirim output fisik -> L: {target_w_left:.4f} rad/s, R: {target_w_right:.4f} rad/s")
            else:
                print("[WARNING] Format salah! Gunakan: base_speed,trim_L,trim_R (Contoh: 1.5,1.0,1.025)")
        except ValueError:
            print("[WARNING] Input harus berupa angka desimal.")

except KeyboardInterrupt:
    print("\n[INFO] Menutup aplikasi...")
finally:
    keep_running = False
    time.sleep(0.1)
    # Kirim perintah stop final sebelum menutup port
    ser.write(b"0.0,0.0\n")
    ser.close()
    print("[INFO] Port serial ditutup. Selesai.")