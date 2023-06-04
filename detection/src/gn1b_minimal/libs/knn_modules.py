import torch
from knn_pytorch import knn_pytorch

# import knn_pytorch
def knn(ref, query, k=1):
  """ Compute k nearest neighbors for each query point.
  """
  device = ref.device
  ref = ref.float().to(device)
  query = query.float().to(device)
  inds = torch.empty(query.shape[0], k, query.shape[2]).long().to(device)
  knn_pytorch.knn(ref, query, inds)
  return inds
