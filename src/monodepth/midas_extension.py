import cv2
import torch # type: ignore
import numpy as np  

class MidasExtension:
    def __init__(self, model_type):
        print("MidasExtension init")

        # self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.device = torch.device("cpu")
        print("before model download")
        self.midas = torch.hub.load("intel-isl/MiDaS", model_type)
        print("after model download")
        self.midas.to(self.device)
        self.midas.eval()
    
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

        if model_type.startswith("DPT_"):
            self.transform = midas_transforms.dpt_transform
        else:
            self.transform = midas_transforms.small_transform

        print("MidasExtension init finished")



    
    def run(self, bgr_image: np.ndarray) -> np.ndarray:
        img = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img).to(self.device)

        # Prediction and resize to original resolution
        with torch.no_grad():
            prediction = self.midas(input_batch)#.squeeze()

            if len(prediction.shape) == 3:
                prediction = prediction.unsqueeze(1)
            elif len(prediction.shape) == 2:
                prediction = prediction.unsqueeze(0).unsqueeze(0)

            prediction = torch.nn.functional.interpolate(
                prediction,
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

            
        depth_map = prediction.cpu().numpy()

        depth_map_normalized = cv2.normalize(depth_map, None, 0, 1, 
                                             norm_type=cv2.NORM_MINMAX, 
                                             dtype=cv2.CV_64F)
        depth_map_8u = (depth_map_normalized * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_map_8u, cv2.COLORMAP_MAGMA)

        print("MiDaS Depth shape:", depth_map.shape)
        print("MiDaS Depth min/max:", depth_map.min(), depth_map.max(), '\n')

        return depth_color, depth_map

