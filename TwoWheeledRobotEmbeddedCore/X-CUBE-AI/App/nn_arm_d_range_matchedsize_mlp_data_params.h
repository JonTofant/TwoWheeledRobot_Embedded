/**
  ******************************************************************************
  * @file    nn_arm_d_range_matchedsize_mlp_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Wed Oct  7 09:00:12 2026
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_PARAMS_H
#define NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_nn_arm_d_range_matchedsize_mlp_data_weights_params[1]))
*/

#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_CONFIG               (NULL)


#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_ACTIVATIONS_SIZES \
  { 1160, }
#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_ACTIVATIONS_SIZE     (1160)
#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_ACTIVATIONS_COUNT    (1)
#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_ACTIVATION_1_SIZE    (1160)



#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_WEIGHTS_SIZES \
  { 93972, }
#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_WEIGHTS_SIZE         (93972)
#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_WEIGHTS_COUNT        (1)
#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_WEIGHT_1_SIZE        (93972)



#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_nn_arm_d_range_matchedsize_mlp_activations_table[1])

extern ai_handle g_nn_arm_d_range_matchedsize_mlp_activations_table[1 + 2];



#define AI_NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_WEIGHTS_TABLE_GET() \
  (&g_nn_arm_d_range_matchedsize_mlp_weights_table[1])

extern ai_handle g_nn_arm_d_range_matchedsize_mlp_weights_table[1 + 2];


#endif    /* NN_ARM_D_RANGE_MATCHEDSIZE_MLP_DATA_PARAMS_H */
