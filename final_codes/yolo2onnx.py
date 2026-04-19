from ultralytics import YOLO

model = YOLO("best(S6).pt")

model.export(
    format="onnx",
    opset=12,
    imgsz=640,
    dynamic=False,
    simplify=True,
    nms=False,     
    half=False,
    verbose=True
)

# instead of this code we can also run an prompt in the CLI
'''
yolo export model=best(S6).pt format=onnx opset=12 imgsz=640 dynamic=False simplify=True device=cpu nms=False half=False

'''
# future ref:
'''
format = onnx , defines the format i need to convert into
opset = 12 , used when we need to convert for tensorRT format, use 12 or 13(suggested in forums)
imgsz = 640 , used when we need to mention which resolution the model is trained for(here 640 x 640)
dynamic = False , coz the resolution is static
simplify = True , something related to onnx graph
verbose = True , to look into logs while converting
nms = False , non max supression layer
half = False , reduces the model size and drops accuracy by half
'''
