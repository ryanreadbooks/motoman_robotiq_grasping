# coding=utf-8
"""
written by ryanreadbooks
date: 2021/11/25
"""

# 数据集数据的key
KeyColorImg = 'color_img'
KeyDepthImg = 'depth_img'
KeyOriginalImage = 'original_image'
KeyOriginalImageDebug = 'original_image_debug'
KeyOriginalImageDebug4Pred = 'original_image_debug4_pred'
KeyGraspness = 'graspness_area'
KeyGraspAngle = 'grasp_angle_area'  # 离散的角度分类编号的key
KeyGraspWidth = 'grasp_width_area'
KeyGraspAngleRaw = 'grasp_angle_area_raw'  # 连续的角度值的key
KeyNetInput = 'net_input'

# 网络直接输出结果的key
KeyGraspnessOut = 'graspness_out'
KeyGraspAngleOut = 'grasp_angle_out'
KeyGraspWidthOut = 'grasp_width_out'

# loss输出用到的key
KeyGraspnessLoss = 'graspness_loss'
KeyAngleLoss = 'angle_loss'
KeyWidthLoss = 'width_loss'

# 网络的输出经过decode的后处理后的结果key
KeyGraspnessPred = 'graspness_pred'
KeyGraspAnglePred = 'grasp_angle_pred'
KeyGraspWidthPred = 'grasp_width_pred'

# 断点保存的时候用到的key
KeyStateDictNetParams = 'key_state_dict_param'
KeyStateDictOptimState = 'key_optim_state_param'
KeyStateDictEpoch = 'key_current_epoch'
KeyStateDictLoss = 'key_loss_at_checkpoint'

# 一些用在evaluation中的metric
KeyMetricAccuracy = 'metric/accuracy'
