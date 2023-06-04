import torch
import torch.nn as nn
import torch.nn.parallel
import torch.utils.data
import torch.nn.functional as F


def decode_pointnet_output(pred, graspness_thres=0.3, objness_thres=0.5, return_numpy=True):
	"""
	解析网络输出的结果
	:param: pred : 网络的输出结果, shape=(1,num_point,2)
	:param: graspness_thres, 为graspness的阈值
	:param: objness_thres, 为object的阈值
	:param: return_numpy, 返回值是否为numpy类型的数组

	:return: pred_graspness shape(num_point, ), pred_objectness shape (num_point, ) np.ndarray
	"""
	pred_graspness, pred_objectness = pred[:, :, 0], pred[:, :, 1]
	pred_objectness = pred_objectness >= objness_thres

	pred_graspness[pred_graspness > graspness_thres] = True
	pred_graspness[pred_graspness <= graspness_thres] = False

	# 返回的两个数组都是bool类型的, 每个点表示是否为graspness和object
	# pred_graspness = torch.logical_and(pred_graspness, pred_objectness)
	if return_numpy:
		return pred_graspness.cpu().squeeze().numpy(), pred_objectness.cpu().squeeze().numpy()
	return pred_graspness.squeeze(), pred_objectness.squeeze()
