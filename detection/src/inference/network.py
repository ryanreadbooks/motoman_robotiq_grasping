# coding=utf-8
import math
import torch
import torch.nn.functional as F
from torch import nn
import torchvision.models.resnet as resnet
from torchvision import models
from itertools import chain
from .key_constants_pool import *
from .se_layer import ChannelSpatialSELayer


class _PSPModule(nn.Module):
    def __init__(self, in_channels, bin_sizes, norm_layer):
        super(_PSPModule, self).__init__()
        out_channels = in_channels // len(bin_sizes)
        self.stages = nn.ModuleList([self._make_stages(in_channels, out_channels, b_s, norm_layer)
                                     for b_s in bin_sizes])
        self.bottleneck = nn.Sequential(
            nn.Conv2d(in_channels + (out_channels * len(bin_sizes)), out_channels,
                      kernel_size=3, padding=1, bias=False),
            norm_layer(out_channels),
            nn.ReLU(inplace=True),
            nn.Dropout2d(0.1)
        )

    def _make_stages(self, in_channels, out_channels, bin_sz, norm_layer):
        prior = nn.AdaptiveAvgPool2d(output_size=bin_sz)
        conv = nn.Conv2d(in_channels, out_channels, kernel_size=1, bias=False)
        bn = norm_layer(out_channels)
        relu = nn.ReLU(inplace=True)
        return nn.Sequential(prior, conv, bn, relu)

    def forward(self, features):
        h, w = features.size()[2], features.size()[3]
        pyramids = [features]
        pyramids.extend([F.interpolate(stage(features), size=(h, w), mode='bilinear',
                                       align_corners=True) for stage in self.stages])
        output = self.bottleneck(torch.cat(pyramids, dim=1))    # (B,512,H,W)
        return output


class PSPNet(nn.Module):
    def __init__(self, n_angle_cls, in_channels=3, backbone='resnet50', pretrained=True):
        super(PSPNet, self).__init__()
        norm_layer = nn.BatchNorm2d
        model = getattr(resnet, backbone)(pretrained)
        m_out_sz = model.fc.in_features

        self.initial = nn.Sequential(*list(model.children())[:4])
        if in_channels != 3:
            self.initial[0] = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.initial = nn.Sequential(*self.initial)

        self.layer1 = model.layer1
        self.layer2 = model.layer2
        self.layer3 = model.layer3
        self.layer4 = model.layer4

        self.psp_module = _PSPModule(m_out_sz, bin_sizes=[1, 2, 3, 6], norm_layer=norm_layer)

        self.up_conv1 = nn.Conv2d(512, 256, kernel_size=1, stride=1)
        self.up_conv2 = nn.Conv2d(256, 256, kernel_size=1, stride=1)

        # 抓取点预测
        self.graspness_head = nn.Sequential(nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1, bias=False),
                                            nn.BatchNorm2d(256),
                                            nn.ReLU(),
                                            nn.Dropout(0.5),

                                            nn.Conv2d(256, 128, kernel_size=3, stride=1, padding=1, bias=False),
                                            nn.BatchNorm2d(128),
                                            nn.ReLU(),
                                            nn.Dropout(0.1),

                                            nn.Conv2d(128, 1, kernel_size=1, stride=1))
        # 抓取角度分类
        n_out = n_angle_cls + 1
        self.angle_head = nn.Sequential(nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1, bias=False),
                                        nn.BatchNorm2d(256),
                                        nn.ReLU(),
                                        nn.Dropout(0.5),

                                        nn.Conv2d(256, 128, kernel_size=3, stride=1, padding=1, bias=False),
                                        nn.BatchNorm2d(128),
                                        nn.ReLU(),
                                        nn.Dropout(0.1),

                                        nn.Conv2d(128, n_out, kernel_size=1, stride=1))

        # 抓取宽度
        self.width_head = nn.Sequential(nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1, bias=False),
                                        nn.BatchNorm2d(256),
                                        nn.ReLU(),
                                        nn.Dropout(0.5),

                                        nn.Conv2d(256, 128, kernel_size=3, stride=1, padding=1, bias=False),
                                        nn.BatchNorm2d(128),
                                        nn.ReLU(),
                                        nn.Dropout(0.1),

                                        nn.Conv2d(128, 1, kernel_size=1, stride=1))

    def forward(self, end_points):
        x = end_points[KeyNetInput]
        x = self.initial(x)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        x = self.psp_module(x)

        x = self.up_conv1(F.interpolate(x, scale_factor=8, mode='bilinear', align_corners=True))
        feature = self.up_conv2(F.interpolate(x, scale_factor=4, mode='bilinear', align_corners=True))

        graspness = self.graspness_head(feature)
        feature = graspness.sigmoid() * feature
        angle = self.angle_head(feature)
        width = self.width_head(feature)

        end_points[KeyGraspnessOut] = graspness
        end_points[KeyGraspAngleOut] = angle
        end_points[KeyGraspWidthOut] = width

        return end_points

    def get_backbone_params(self):
        return chain(self.initial.parameters(), self.layer1.parameters(), self.layer2.parameters(),
                     self.layer3.parameters(), self.layer4.parameters())

    def get_decoder_params(self):
        return chain(self.master_branch.parameters(), self.auxiliary_branch.parameters())

    def freeze_bn(self):
        for module in self.modules():
            if isinstance(module, nn.BatchNorm2d): module.eval()


class FeatureExtractor(nn.Module):
    def __init__(self, in_channels=3, backbone='resnet18', pretrained=False):
        super().__init__()
        self.norm_layer = nn.BatchNorm2d
        model = getattr(resnet, backbone)(pretrained)
        self.m_out_sz = model.fc.in_features

        self.initial = nn.Sequential(*list(model.children())[:4])
        if in_channels != 3:
            self.initial[0] = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.initial = nn.Sequential(*self.initial)
        self.layer1 = model.layer1
        self.layer2 = model.layer2
        self.layer3 = model.layer3
        self.layer4 = model.layer4

    def forward(self, x):
        x = self.initial(x)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        return x


class MixedPSPNet(nn.Module):
    def __init__(self, n_angle_cls, backbone='resnet18', pretrained=True):
        super(MixedPSPNet, self).__init__()
        # 主干网络，rgb和d分别提取
        self.rgb_feat_extractor = FeatureExtractor(in_channels=3, backbone=backbone, pretrained=pretrained)
        self.depth_feat_extractor = FeatureExtractor(in_channels=1, backbone=backbone, pretrained=pretrained)
        # 两个分支分别都有ppm模块
        self.rgb_feat_psp = _PSPModule(self.rgb_feat_extractor.m_out_sz,
                                       bin_sizes=[1, 2, 3, 6],
                                       norm_layer=self.rgb_feat_extractor.norm_layer)
        self.depth_feat_psp = _PSPModule(self.depth_feat_extractor.m_out_sz,
                                         bin_sizes=[1, 2, 3, 6],
                                         norm_layer=self.depth_feat_extractor.norm_layer)
        # 上采样后进行卷积进一步处理
        self.rgb_up_conv1 = nn.Sequential(
            nn.Conv2d(256, 64, kernel_size=3, stride=1, padding=1, bias=False),
            nn.BatchNorm2d(64),
            nn.GELU()
        )
        self.depth_up_conv1 = nn.Sequential(
            nn.Conv2d(256, 64, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(64),
            nn.GELU()
        )

        # 上采样后进行卷积进一步处理
        self.rgb_up_conv2 = nn.Sequential(
            nn.Conv2d(128, 64, kernel_size=3, stride=1, padding=1, bias=False),
            nn.BatchNorm2d(64),
            nn.GELU()
        )
        self.depth_up_conv2 = nn.Sequential(
            nn.Conv2d(128, 64, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(64),
            nn.GELU()
        )
        n_out = n_angle_cls + 1
        self.graspness_head = nn.Sequential(
            ChannelSpatialSELayer(num_channels=128),
            nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1, bias=False),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Conv2d(128, 1, kernel_size=1, stride=1)
        )

        self.angle_head = nn.Sequential(
            ChannelSpatialSELayer(num_channels=128),
            nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1, bias=False),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Conv2d(128, n_out, kernel_size=1, stride=1)
        )

        self.width_head = nn.Sequential(
            ChannelSpatialSELayer(num_channels=128),
            nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1, bias=False),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Conv2d(128, 1, kernel_size=1, stride=1)
        )

    def forward(self, end_points):
        rgb_feat = self.rgb_feat_extractor(end_points[KeyColorImg])
        depth_feat = self.depth_feat_extractor(end_points[KeyDepthImg])

        rgb_feat = self.rgb_feat_psp(rgb_feat)  # (bs, 128, 12, 12)
        depth_feat = self.depth_feat_psp(depth_feat)  # (bs, 128, 12, 12)

        mixed = rgb_feat * depth_feat  # (bs, 128, 12, 12)
        rgb_feat = torch.cat([rgb_feat, mixed], dim=1)  # (bs, 256, 12, 12)
        depth_feat = torch.cat([depth_feat, mixed], dim=1)  # (bs, 256, 12, 12)
        rgb_feat = self.rgb_up_conv1(F.interpolate(rgb_feat, scale_factor=8, mode='bilinear', align_corners=True))  # (bs, 64, 96, 96)
        depth_feat = self.depth_up_conv1(F.interpolate(depth_feat, scale_factor=8, mode='bilinear', align_corners=True))  # (bs, 64, 96, 96)

        mixed2 = rgb_feat * depth_feat  # # (bs, 64, 96, 96)
        rgb_feat = torch.cat([rgb_feat, mixed2], dim=1)  # (bs, 128, 96, 96)
        depth_feat = torch.cat([depth_feat, mixed2], dim=1)  # (bs, 128, 96, 96）
        rgb_feat = self.rgb_up_conv2(F.interpolate(rgb_feat, scale_factor=4, mode='bilinear', align_corners=True))  # (bs, 64, 384, 384)
        depth_feat = self.depth_up_conv2(F.interpolate(depth_feat, scale_factor=4, mode='bilinear', align_corners=True))  # (bs, 64, 384, 384)

        # mixed features from rgb features and depth features
        x = torch.cat([rgb_feat, depth_feat], dim=1)   # (bs, 128, 384, 384)

        # 三个分支分别处理
        graspness = self.graspness_head(x)
        angle = self.angle_head(x)
        width = self.width_head(x)

        end_points[KeyGraspnessOut] = graspness
        end_points[KeyGraspAngleOut] = angle
        end_points[KeyGraspWidthOut] = width

        return end_points
