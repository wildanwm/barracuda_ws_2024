import cv2
import numpy as np

# Mendefinisikan rentang warna hitam dalam ruang warna HSV
lower_black = np.array([5, 0, 40])
upper_black = np.array([190, 190, 80])  # Ini adalah nilai yang dapat disesuaikan sesuai dengan kondisi pencahayaan

# Mengakses kamera
cap = cv2.VideoCapture(0)

x1, y1 = 100, 50
x2, y2 = 540, 430

while True:
    # Membaca frame dari kamera
    ret, frame = cap.read()
    if not ret:
        print("Error: Tidak bisa membaca frame dari kamera.")
        break

    cropped_frame = frame[y1:y2, x1:x2]

    # Mengonversi frame ke ruang warna HSV
    hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

    # Menerapkan threshold untuk mendapatkan wilayah hitam
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Menemukan kontur
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Memfilter dan menggambar kotak hitam pada gambar asli
    if hierarchy is not None:
        # Menghitung bounding box dari kontur
        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))

        # Menggambar persegi panjang pada gambar asli
        cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Menampilkan hasil
    cv2.imshow('Detected Boxes', cropped_frame)
    cv2.imshow('Detected Mask', mask)

    # Keluar dengan menekan 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Membersihkan dan menutup jendela
cap.release()
cv2.destroyAllWindows()
