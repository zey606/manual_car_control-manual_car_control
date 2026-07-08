import serial
import time

try:
    # Lidar'ın bağlı olduğu gerçek portu 115200 hızıyla yakala
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    
    # İşte o veri musluğunu açacak sihirli dokunuş: Sinyalleri el ile tetikle
    ser.dtr = False
    ser.rts = True
    
    print("[+] Lidar DTR/RTS sinyal kilitleri yazılımla kırıldı!")
    print("[+] Veri musluğu açıldı. Bu terminali KAPATMA, ROS 2'yi diğer terminalden başlat.")
    
    # Bağlantıyı açık tutmak için sonsuz döngüye gir
    while True:
        time.sleep(1)
except Exception as e:
    print("[-] Hata oluştu:", e)
