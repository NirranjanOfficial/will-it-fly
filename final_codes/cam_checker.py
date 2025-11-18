import cv2
import keyboard
import time

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Error: Could not open camera.")
    exit()

print("📷 Camera opened successfully. Press 'q' to quit.")

while True:
    if keyboard.is_pressed("q"):
        print("🛑 Stopping feed (keyboard interrupt).")
        break

    ret, frame = cap.read()
    if not ret:
        print("⚠ Frame not received, retrying...")
        time.sleep(0.1)  # ✅ Add small delay before retrying
        continue  # ✅ IMPORTANT: Skip to next iteration
    
    cv2.imshow("FRAME", frame)  

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ✅ Always cleanup
cap.release()
cv2.destroyAllWindows()
print("✅ Camera feed ended.")