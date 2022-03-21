import torch
import torch.nn as nn
import torch.nn.parallel
import torch.utils.data
import torch.nn.functional as F
from .pointnet_utils import PointNetEncoder


class PointNet(nn.Module):
    def __init__(self, num_class):
        super(PointNet, self).__init__()
        self.k = num_class
        self.feat = PointNetEncoder(global_feat=False, feature_transform=True, channel=3)
        self.conv1 = torch.nn.Conv1d(1088, 512, 1)
        self.conv2 = torch.nn.Conv1d(512, 256, 1)
        self.conv3 = torch.nn.Conv1d(256, 128, 1)
        self.conv4 = torch.nn.Conv1d(128, self.k, 1)
        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(128)

    def forward(self, x):
        batchsize = x.size()[0]
        n_pts = x.size()[2]
        x, trans, trans_feat = self.feat(x)
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = self.conv4(x)
        x = x.transpose(2, 1).contiguous()
        x = torch.sigmoid(x)
        return x
    

def decode_pointnet_output(pred, graspness_thres = 0.3, objness_thres = 0.5):
	"""
	解析网络输出的结果
	:param: pred : 网络的输出结果, shape=(1,num_point,2)
	:param: graspness_thres, 为graspness的阈值
	:param: objness_thres, 为object的阈值

	:return: pred_graspness shape(num_point, ), pred_objectness shape (num_point, ) np.ndarray
	"""
	pred_graspness, pred_objectness = pred[:, :, 0], pred[:, :, 1]
	pred_objectness = pred_objectness >= objness_thres

	pred_graspness[pred_graspness > graspness_thres] = True
	pred_graspness[pred_graspness <= graspness_thres] = False

	return pred_graspness.cpu().detach().squeeze().numpy(), pred_objectness.cpu().detach().squeeze().numpy()
