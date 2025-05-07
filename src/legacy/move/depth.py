import sys

import cv2
import torch
import matplotlib
import numpy as np

from depth_anything_v2.dpt import DepthAnythingV2

def depth_map(frame, debug=False):
    input_size = 518
    encoder = "vits" # ['vits', 'vitb', 'vitl', 'vitg']
    grayscale = True
    
    DEVICE = 'cpu'
    
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }
    
    depth_anything = DepthAnythingV2(**model_configs[encoder])
    depth_anything.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
    depth_anything = depth_anything.to(DEVICE).eval()
    
    cmap = matplotlib.colormaps.get_cmap('Spectral_r')
    
    
    depth = depth_anything.infer_image(frame, input_size)
    
    depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
    depth = depth.astype(np.uint8)
    
    if grayscale:
        depth = np.repeat(depth[..., np.newaxis], 3, axis=-1)
    else:
        depth = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
    
    if debug:
        cv2.imwrite('depth.png', depth)

    return depth

if __name__ == "__main__":
    img_path = sys.argv[1]
    raw_image = cv2.imread(img_path)
    depth_map(raw_image, debug=True)