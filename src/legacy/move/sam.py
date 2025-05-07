import sys

import numpy as np
from PIL import Image
from ultralytics import FastSAM, SAM

def multiply(mask, image):
    if type(image) == str:
        image = Image.open(image).convert("RGB")  # Ensure the image is in RGB

    image_array = np.array(image)
    mask_array = np.array(mask)

    mask_normalized = mask_array / 255.0

    # Multiply the image by the mask
    result_array = (image_array * mask_normalized[:, :, np.newaxis]).astype(np.uint8)

    # Convert the result back to an image
    return Image.fromarray(result_array)

def sam(image_orig, debug=False):
    model = SAM("sam2_b.pt")  # Alternate model
    
    # Run inference with point prompts
    results = model(image_orig, points=[[640, 360], [600, 360], [500, 360], [400, 360]], classes=0)

    # `results[0].masks` is the Mask object (batch of masks)
    # Convert them to a NumPy-compatible array (batch, H, W)
    masks = results[0].masks.data

    # Keep track of the best mask
    best_mask = None
    best_mask_nonzero_count = 0

    # Iterate over each mask
    for i in range(masks.shape[0]):
        single_mask = masks[i]

        # Normalize from [0,1] to [0,255] and convert to uint8
        normalized_array = (single_mask.numpy() * 255).astype(np.uint8)

        # Count how many non-black (non-zero) pixels
        nonzero_count = np.count_nonzero(normalized_array)
        
        print("nn",nonzero_count)
        # Check if this mask is the largest so far
        if nonzero_count > best_mask_nonzero_count and nonzero_count < 300000:
            best_mask_nonzero_count = nonzero_count
            best_mask = normalized_array

        if debug:
            output_path = f'mask_{i}.png'
            image = Image.fromarray(normalized_array, mode='L')
            image.save(output_path)
            print(f"Saved largest mask to {output_path}")

    # If we found at least one mask
    if best_mask is not None:
        # Convert the best mask array to a PIL grayscale image
        image = Image.fromarray(best_mask, mode='L')

        # Resize to original input shape (note: `orig_shape` is (height, width))
        orig_shape = results[0].masks.orig_shape
        image = image.resize(orig_shape[::-1], resample=Image.NEAREST)

        # Call your `multiply()` function (not shown here).
        # Adjust as needed if `multiply()` expects a PIL RGB image, etc.
        image = multiply(image, image_orig)

        if debug:
            output_path = 'mask.png'
            image.save(output_path)
            print(f"Saved largest mask to {output_path}")

        # Return the PIL image of the largest mask
        return image
    else:
        # No masks found, or they were empty
        if debug:
            print("No valid masks were found.")
        return None
    
if __name__ == "__main__":
    sam(sys.argv[1], debug=True)