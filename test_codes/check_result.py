# import cv2
# import random
# import os
# from ultralytics import YOLO



# def main():

#     # --- Settings ---
#     model_path = 'yolov8n.pt'  # Your trained model path
#     data_yaml = 'data.yaml'  # Validation dataset config YAML
#     image_folder = 'test/images'  # Folder with test images

#     # --- Load model ---
#     model = YOLO(model_path)

#     # --- Run validation to get metrics and loss ---
#     metrics = model.val(data=data_yaml)

#     # Extract and print metrics
#     print(f"mAP50-95: {metrics.box.map:.4f}")  # mAP 0.5:0.95
#     print(f"mAP50: {metrics.box.map50:.4f}")   # mAP 0.5
#     print(f"mAP75: {metrics.box.map75:.4f}")   # mAP 0.75
#     print(f"Precision: {metrics.box.p:.4f}")
#     print(f"Recall: {metrics.box.r:.4f}")

#     # Loss might be stored in metrics.loss (depends on ultralytics version)
#     # If not, loss can sometimes be accessed via metrics.loss_box, loss_cls, loss_obj
#     if hasattr(metrics, 'loss'):
#         print(f"Validation Loss: {metrics.loss:.4f}")
#     elif hasattr(metrics, 'loss_box'):
#         total_loss = metrics.loss_box + getattr(metrics, 'loss_cls', 0) + getattr(metrics, 'loss_obj', 0)
#         print(f"Validation Loss (sum): {total_loss:.4f}")
#     else:
#         print("Validation loss not available from val() output.")

#     # --- Load a random test image ---
#     img_name = random.choice(os.listdir(image_folder))
#     img_path = os.path.join(image_folder, img_name)
#     img = cv2.imread(img_path)

#     # --- Run inference on the image ---
#     results = model(img)[0]

#     # --- Annotate and display the image ---
#     annotated_img = results.plot()

#     cv2.imshow("YOLOv8 Detection", annotated_img)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()



#---------------------------------------
import cv2
import random
import os
from ultralytics import YOLO

def main():

    # --- Settings ---
    model_path = 'best.pt'  # here i have entered the COCO set, rewrite the best.pt of my model!
    image_folder = 'test/images'  # Folder with test images

    # --- Load model ---
    model = YOLO(model_path)

    # --- Print all classes the model was trained on ---
    print("Classes the model was trained on:")
    if isinstance(model.names, dict):
        for cls_id, cls_name in model.names.items():
            print(f"{cls_id}: {cls_name}")
    else:
        for cls_id, cls_name in enumerate(model.names):
            print(f"{cls_id}: {cls_name}")

    # --- Load a random test image ---
    img_name = random.choice(os.listdir(image_folder))
    img_path = os.path.join(image_folder, img_name)
    img = cv2.imread(img_path)

    if img is None:
        raise Exception(f"Failed to read image: {img_path}")

    # --- Run inference on the image ---
    results = model(img)[0]

    # --- Extract detected classes, confidences ---
    boxes = results.boxes
    class_ids = boxes.cls.cpu().numpy().astype(int) if boxes else []
    confidences = boxes.conf.cpu().numpy() if boxes else []
    names = model.names

    # Print detected objects or "Nothing detected"
    if len(class_ids) == 0:
        print(f"No objects detected in {img_name}")
    else:
        print(f"Detections on {img_name}:")
        for cls_id, conf in zip(class_ids, confidences):
            print(f"  - {names[cls_id]}: {conf:.2f}")

    # --- Annotate and display the image ---
    annotated_img = results.plot()

    cv2.imshow("YOLOv8 Detection", annotated_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()