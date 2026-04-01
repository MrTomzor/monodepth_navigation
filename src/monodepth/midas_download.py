import cv2
import torch # type: ignore
import numpy as np  
device = torch.device("cpu")
print("before model download")
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
print("after model download")
