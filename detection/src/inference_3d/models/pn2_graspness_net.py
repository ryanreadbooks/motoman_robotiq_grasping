"""
written by ryanreadbooks
brief: 
date: 2022/5/26
"""

import torch.nn as nn

from .pointnet2_backbone import PointNet2Decoder, PointNet2Encoder


class PN2GraspnessNet(nn.Module):
    def __init__(self, n_in, n_out):
        super(PN2GraspnessNet, self).__init__()
        self.n_in = n_in
        self.n_out = n_out
        ### backbone encoder
        self.encoder = PointNet2Encoder(n_in_feature=n_in)
        ### graspness branch with decoder
        self.graspness_branch_decoder = PointNet2Decoder(n_in_feature=n_in)
        self.graspness_head = nn.Sequential(
            nn.Conv1d(128, 64, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm1d(64),
            nn.ReLU(True),
            nn.Conv1d(64, 16, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm1d(16),
            nn.ReLU(True),
            nn.Conv1d(16, n_out, kernel_size=3, padding=1, bias=True),
            nn.Sigmoid()
        )
        ### objectness branch with decoder
        self.objectness_branch_decoder = PointNet2Decoder(n_in_feature=n_in)
        self.objectness_head = nn.Sequential(
            nn.Conv1d(128, 64, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm1d(64),
            nn.ReLU(True),
            nn.Conv1d(64, 16, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm1d(16),
            nn.ReLU(True),
            nn.Conv1d(16, n_out, kernel_size=3, padding=1, bias=True),
            nn.Sigmoid()
        )

    def forward(self, x):
        l_xyz, l_features = self.encoder(x)  # list with encoded features
        g1 = self.graspness_branch_decoder(l_xyz, l_features)  # (bs, 128, n)
        graspness = self.graspness_head(g1)  # (bs, 1, n)
        o1 = self.objectness_branch_decoder(l_xyz, l_features)  # (bs, 128, n)
        objectness = self.objectness_head(o1)  # (bs, 1, n)

        return graspness.transpose(2, 1), objectness.transpose(2, 1)    # (bs, n, 1) (bs, n, 1)
