import cv2
import numpy as np
from glob import glob
import matplotlib.pyplot as plt

# ========================
# CONFIG
# ========================
IMG_SIZE = 640   # Change to 720 if needed
bg_path = "grass.jpeg"   # relative path to ONE background image

# ========================
# FUNCTIONS
# ========================
def draw_marker(size=200):
    """Creates the marker: white square with a red filled circle"""
    marker = np.ones((size, size, 3), dtype=np.uint8) * 255  # white square
    cv2.circle(marker, (size//2, size//2), size//3, (0,0,255), -1)   # red solid circle
    return marker

def get_bbox(corners, img_w, img_h):
    """Convert square corners to YOLO bbox format"""
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    
    x_center = (x_min + x_max) / 2 / img_w
    y_center = (y_min + y_max) / 2 / img_h
    w = (x_max - x_min) / img_w
    h = (y_max - y_min) / img_h
    return x_center, y_center, w, h

def apply_augmentations(img):
    """Simulate drone camera with brightness/blur/noise"""
    import random
    # Random brightness
    if random.random() < 0.5:
        factor = 0.6 + 0.8 * random.random()
        img = np.clip(img * factor, 0, 255).astype(np.uint8)

    # Gaussian blur
    if random.random() < 0.3:
        k = np.random.choice([3,5])
        img = cv2.GaussianBlur(img, (k, k), 0)

    # Add noise
    if random.random() < 0.3:
        noise = np.random.randint(0, 20, img.shape, dtype=np.uint8)
        img = cv2.add(img, noise)

    return img

# ========================
# MAIN (one image only)
# ========================

# --- Load background safely ---
bg = cv2.imread(bg_path)
if bg is None:
    raise FileNotFoundError(f"❌ Could not read background: {bg_path}")
bg = cv2.resize(bg, (IMG_SIZE, IMG_SIZE))

labels = []

# Example: place 1 marker (can adjust to >1 if you want)
marker_size = 150
marker = draw_marker(size=marker_size)

rows, cols, _ = marker.shape
pts1 = np.float32([[0,0],[cols,0],[0,rows],[cols,rows]])

# Perspective tilt
shift = 20
pts2 = np.float32([
    [np.random.randint(0,shift), np.random.randint(0,shift)],
    [cols-np.random.randint(0,shift), np.random.randint(0,shift)],
    [np.random.randint(0,shift), rows-np.random.randint(0,shift)],
    [cols-np.random.randint(0,shift), rows-np.random.randint(0,shift)]
])

M = cv2.getPerspectiveTransform(pts1, pts2)
warped = cv2.warpPerspective(marker, M, (cols, rows))

# Placement
x_off = np.random.randint(0, IMG_SIZE - cols)
y_off = np.random.randint(0, IMG_SIZE - rows)

roi = bg[y_off:y_off+rows, x_off:x_off+cols]
mask = (warped < 250).any(axis=2)  # only paste non-white
roi[mask] = warped[mask]
bg[y_off:y_off+rows, x_off:x_off+cols] = roi

# Bounding box
corners = [(x_off + x, y_off + y) for x,y in pts2]
x_center, y_center, w, h = get_bbox(corners, IMG_SIZE, IMG_SIZE)
labels.append(f"0 {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}")

# Final augmentation
bg = apply_augmentations(bg)

# ========================
# Show result
# ========================
print("YOLO Label(s):")
print("\n".join(labels))

plt.imshow(cv2.cvtColor(bg, cv2.COLOR_BGR2RGB))
plt.axis("off")
plt.show()
