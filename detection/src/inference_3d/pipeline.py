import time
import numpy as np
import copy
import torch
import pytransform3d.transformations as pytra
import pytransform3d.rotations as pyrot
from scipy.spatial import KDTree
from sklearn.decomposition import PCA

from .models import PointNet2SemSegSSG, decode_pointnet_output, PN2GraspnessNet
from pointnet2_ops import pointnet2_utils
from .utils import *
from .geometry_utils import *


Tensor = torch.Tensor
NdArray = np.ndarray

def detection3d_log(msg, level='info'):
    color = 32
    if level == 'info':
        color = 32
    elif level == 'warn':
        color = 33
    elif level == 'err':
        color = 31
    else:
        color = 34
    print(f"\033[{color}m[{time.time()}] node_detection::pipeline => {msg}\033[0m")


class GraspNotFoundError(Exception):
    """
    自定义异常,没有找到抓取位置时抛出此异常
    """
    def __init__(self, *args: object) -> None:
        super().__init__(*args)
