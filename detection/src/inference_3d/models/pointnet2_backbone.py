import pytorch_lightning as pl
import torch
import torch.nn as nn
from pointnet2_ops.pointnet2_modules import PointnetFPModule, PointnetSAModule
from torch.utils.data import DataLoader

from .pointnet2_ssg_cls import PointNet2ClassificationSSG


class PointNet2Encoder(PointNet2ClassificationSSG):
    """
    PointNet2编码器
    """
    def __init__(self, n_in_feature, use_xyz=True):
        self.n_in_feature = n_in_feature
        super(PointNet2Encoder, self).__init__(use_xyz=use_xyz)

    def _build_model(self):
        self.SA_modules = nn.ModuleList()
        self.SA_modules.append(
            PointnetSAModule(
                npoint=1024,
                radius=0.1,
                nsample=32,
                # 这里第一个mlp的输入为3
                mlp=[self.n_in_feature, 32, 32, 64],
                use_xyz=self.use_xyz,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=256,
                radius=0.2,
                nsample=32,
                mlp=[64, 64, 64, 128],
                use_xyz=self.use_xyz,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=64,
                radius=0.4,
                nsample=32,
                mlp=[128, 128, 128, 256],
                use_xyz=self.use_xyz,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=16,
                radius=0.8,
                nsample=32,
                mlp=[256, 256, 256, 512],
                use_xyz=self.use_xyz,
            )
        )

    def forward(self, pointcloud):
        r"""
            Forward pass of the network

            Parameters
            ----------
            pointcloud: Variable(torch.cuda.FloatTensor)
                (B, N, 3 + input_channels) tensor
                Point cloud to run predicts on
                Each point in the point-cloud MUST
                be formatted as (x, y, z, features...)
        """
        xyz, features = self._break_up_pc(pointcloud)
        l_xyz, l_features = [xyz], [features]
        # 从前往后处理特征
        for i in range(len(self.SA_modules)):
            li_xyz, li_features = self.SA_modules[i](l_xyz[i], l_features[i])
            l_xyz.append(li_xyz)
            l_features.append(li_features)

        return l_xyz, l_features


class PointNet2Decoder(nn.Module):
    """
    PointNet2解码器
    """
    def __init__(self, n_in_feature):
        super(PointNet2Decoder, self).__init__()
        self.n_in_feature = n_in_feature
        self.FP_modules = nn.ModuleList()
        # 下面的+3和输入的第一个mlp的输入尺寸要匹配（也就是第24行的位置）
        self.FP_modules.append(PointnetFPModule(mlp=[128 + self.n_in_feature, 128, 128, 128]))
        self.FP_modules.append(PointnetFPModule(mlp=[256 + 64, 256, 128]))
        self.FP_modules.append(PointnetFPModule(mlp=[256 + 128, 256, 256]))
        self.FP_modules.append(PointnetFPModule(mlp=[512 + 256, 256, 256]))

    def forward(self, l_xyz, l_features):
        l_features_minus2 = self.FP_modules[-1](l_xyz[-2], l_xyz[-1], l_features[-2], l_features[-1])
        l_features_minus3 = self.FP_modules[-2](l_xyz[-3], l_xyz[-2], l_features[-3], l_features_minus2)
        l_features_minus4 = self.FP_modules[-3](l_xyz[-4], l_xyz[-3], l_features[-4], l_features_minus3)
        l_features_minus5 = self.FP_modules[-4](l_xyz[-5], l_xyz[-4], l_features[-5], l_features_minus4)

        return l_features_minus5


class PointNet2Backbone(PointNet2ClassificationSSG):
    """
    PointNet2主干网络
    """
    def __init__(self, n_in_feature, n_out, use_xyz=True):
        self.n_cls = n_out
        self.n_in_feature = n_in_feature
        super(PointNet2Backbone, self).__init__(use_xyz=use_xyz)

    def _build_model(self):
        self.SA_modules = nn.ModuleList()
        self.SA_modules.append(
            PointnetSAModule(
                npoint=1024,
                radius=0.1,
                nsample=32,
                # 这里第一个mlp的输入为3
                mlp=[self.n_in_feature, 32, 32, 64],
                use_xyz=self.use_xyz,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=256,
                radius=0.2,
                nsample=32,
                mlp=[64, 64, 64, 128],
                use_xyz=self.use_xyz,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=64,
                radius=0.4,
                nsample=32,
                mlp=[128, 128, 128, 256],
                use_xyz=self.use_xyz,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=16,
                radius=0.8,
                nsample=32,
                mlp=[256, 256, 256, 512],
                use_xyz=self.use_xyz,
            )
        )

        self.FP_modules = nn.ModuleList()
        # 下面的+3和输入的第一个mlp的输入尺寸要匹配（也就是第24行的位置）
        self.FP_modules.append(PointnetFPModule(mlp=[128 + self.n_in_feature, 128, 128, 128]))
        self.FP_modules.append(PointnetFPModule(mlp=[256 + 64, 256, 128]))
        self.FP_modules.append(PointnetFPModule(mlp=[256 + 128, 256, 256]))
        self.FP_modules.append(PointnetFPModule(mlp=[512 + 256, 256, 256]))

        self.fc_layer = nn.Sequential(
            nn.Conv1d(128, 128, kernel_size=1, bias=False),
            nn.BatchNorm1d(128),
            nn.LeakyReLU(True),
            nn.Dropout(0.5),
            nn.Conv1d(128, 128, kernel_size=1),
            nn.BatchNorm1d(128),
            nn.LeakyReLU(True),
            nn.Conv1d(128, self.n_cls, kernel_size=1)
        )

    def forward(self, pointcloud):
        r"""
            Forward pass of the network

            Parameters
            ----------
            pointcloud: Variable(torch.cuda.FloatTensor)
                (B, N, 3 + input_channels) tensor
                Point cloud to run predicts on
                Each point in the point-cloud MUST
                be formatted as (x, y, z, features...)
        """
        xyz, features = self._break_up_pc(pointcloud)
        l_xyz, l_features = [xyz], [features]
        # 从前往后处理特征
        for i in range(len(self.SA_modules)):
            li_xyz, li_features = self.SA_modules[i](l_xyz[i], l_features[i])
            l_xyz.append(li_xyz)
            l_features.append(li_features)
        # 从后往前处理特征
        for i in range(-1, -(len(self.FP_modules) + 1), -1):
            l_features[i - 1] = self.FP_modules[i](
                l_xyz[i - 1], l_xyz[i], l_features[i - 1], l_features[i]
            )
        # 此处的l_features是一个list,l_features[0]为最终经过fp之后的特征
        return self.fc_layer(l_features[0])  # (B, n_feature, N)
