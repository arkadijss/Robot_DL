# MIT License

# Copyright (c) 2022 Intelligent Systems Lab Org

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# File author: Shariq Farooq Bhat

import argparse

import torch
import cv2 as cv
from tqdm import tqdm

from zoedepth.models.builder import build_model
from zoedepth.utils.config import change_dataset, get_config
from zoedepth.utils.easydict import EasyDict as edict


@torch.no_grad()
def infer(model, images, **kwargs):
    """Inference with flip augmentation"""

    # images.shape = N, C, H, W
    def get_depth_from_prediction(pred):
        if isinstance(pred, torch.Tensor):
            pred = pred  # pass
        elif isinstance(pred, (list, tuple)):
            pred = pred[-1]
        elif isinstance(pred, dict):
            pred = pred["metric_depth"] if "metric_depth" in pred else pred["out"]
        else:
            raise NotImplementedError(f"Unknown output type {type(pred)}")
        return pred

    pred1 = model(images, **kwargs)
    pred1 = get_depth_from_prediction(pred1)

    pred2 = model(torch.flip(images, [3]), **kwargs)
    pred2 = get_depth_from_prediction(pred2)
    pred2 = torch.flip(pred2, [3])

    mean_pred = 0.5 * (pred1 + pred2)

    return mean_pred


def build_model_metric():
    config = {
        "model": "zoedepth",
        "version_name": "v1",
        "input_height": 240,
        "input_width": 320,
        "pretrained_resource": "local::/workspace/checkpoints/depth_anything_metric_depth_indoor.pt",
    }
    config = edict(config)
    model = build_model(config).cuda()
    model.eval()
    return model


def prepare_sample(img):
    img_norm = (img / 255.0).astype("float32")
    img_torch = torch.from_numpy(img_norm.transpose((2, 0, 1))).unsqueeze(0)
    return img_torch


if __name__ == "__main__":
    img_path = "extraction/1.0.1/bag_images/1705515112.69.png"
    img = cv.imread(img_path)
    img_torch = prepare_sample(img)
    model = build_model_metric()
    pred = infer(model, img_torch)
    print(pred.shape)
