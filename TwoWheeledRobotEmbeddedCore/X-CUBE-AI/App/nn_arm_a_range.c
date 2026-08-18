/**
  ******************************************************************************
  * @file    nn_arm_a_range.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Tue Aug 18 13:49:33 2026
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "nn_arm_a_range.h"
#include "nn_arm_a_range_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_nn_arm_a_range
 
#undef AI_NN_ARM_A_RANGE_MODEL_SIGNATURE
#define AI_NN_ARM_A_RANGE_MODEL_SIGNATURE     "8a409865b625aa113ec4b79c48fee034"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Tue Aug 18 13:49:33 2026"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NN_ARM_A_RANGE_N_BATCHES
#define AI_NN_ARM_A_RANGE_N_BATCHES         (1)

static ai_ptr g_nn_arm_a_range_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_nn_arm_a_range_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  _actor_actor_0_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)
/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  _actor_actor_2_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4096, AI_STATIC)
/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _actor_actor_2_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)
/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  actions_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)
/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  actions_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)
/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  obs_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 13, AI_STATIC)
/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  output_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)
/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  _actor_actor_0_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)
/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _actor_actor_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)
/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _actor_actor_2_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)
/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _actor_actor_1_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)
/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  actions_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)
/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  actions_tanh_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)
/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  current_a_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 2, AI_STATIC)
/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  _actor_actor_0_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 832, AI_STATIC)
/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _actor_actor_0_Gemm_output_0_bias, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_actor_actor_0_Gemm_output_0_bias_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _actor_actor_2_Gemm_output_0_weights, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 64, 64, 1, 1), AI_STRIDE_INIT(4, 4, 256, 16384, 16384),
  1, &_actor_actor_2_Gemm_output_0_weights_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  _actor_actor_2_Gemm_output_0_bias, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_actor_actor_2_Gemm_output_0_bias_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  actions_weights, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 64, 2, 1, 1), AI_STRIDE_INIT(4, 4, 256, 512, 512),
  1, &actions_weights_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  actions_bias, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &actions_bias_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  obs_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 13, 1, 1), AI_STRIDE_INIT(4, 4, 4, 52, 52),
  1, &obs_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  output_scale, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &output_scale_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _actor_actor_0_Gemm_output_0_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_actor_actor_0_Gemm_output_0_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _actor_actor_1_Relu_output_0_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_actor_actor_1_Relu_output_0_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _actor_actor_2_Gemm_output_0_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_actor_actor_2_Gemm_output_0_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _actor_actor_1_1_Relu_output_0_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_actor_actor_1_1_Relu_output_0_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  actions_output, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &actions_output_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  actions_tanh_output, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &actions_tanh_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  current_a_output, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &current_a_output_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _actor_actor_0_Gemm_output_0_weights, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 13, 64, 1, 1), AI_STRIDE_INIT(4, 4, 52, 3328, 3328),
  1, &_actor_actor_0_Gemm_output_0_weights_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  current_a_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &actions_tanh_output, &output_scale),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &current_a_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  current_a_layer, 7,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &current_a_chain,
  NULL, &current_a_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  actions_tanh_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &actions_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &actions_tanh_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  actions_tanh_layer, 6,
  NL_TYPE, 0x0, NULL,
  nl, forward_tanh,
  &actions_tanh_chain,
  NULL, &current_a_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  actions_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_actor_actor_1_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &actions_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &actions_weights, &actions_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  actions_layer, 5,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &actions_chain,
  NULL, &actions_tanh_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _actor_actor_1_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_actor_actor_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_actor_actor_1_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _actor_actor_1_1_Relu_output_0_layer, 4,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_actor_actor_1_1_Relu_output_0_chain,
  NULL, &actions_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _actor_actor_2_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_actor_actor_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_actor_actor_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_actor_actor_2_Gemm_output_0_weights, &_actor_actor_2_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _actor_actor_2_Gemm_output_0_layer, 3,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_actor_actor_2_Gemm_output_0_chain,
  NULL, &_actor_actor_1_1_Relu_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _actor_actor_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_actor_actor_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_actor_actor_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _actor_actor_1_Relu_output_0_layer, 2,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_actor_actor_1_Relu_output_0_chain,
  NULL, &_actor_actor_2_Gemm_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _actor_actor_0_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &obs_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_actor_actor_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_actor_actor_0_Gemm_output_0_weights, &_actor_actor_0_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _actor_actor_0_Gemm_output_0_layer, 1,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_actor_actor_0_Gemm_output_0_chain,
  NULL, &_actor_actor_1_Relu_output_0_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 20748, 1, 1),
    20748, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 512, 1, 1),
    512, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NN_ARM_A_RANGE_IN_NUM, &obs_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NN_ARM_A_RANGE_OUT_NUM, &current_a_output),
  &_actor_actor_0_Gemm_output_0_layer, 0, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 20748, 1, 1),
      20748, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 512, 1, 1),
      512, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NN_ARM_A_RANGE_IN_NUM, &obs_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NN_ARM_A_RANGE_OUT_NUM, &current_a_output),
  &_actor_actor_0_Gemm_output_0_layer, 0, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/


/******************************************************************************/
AI_DECLARE_STATIC
ai_bool nn_arm_a_range_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_nn_arm_a_range_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    obs_output_array.data = AI_PTR(g_nn_arm_a_range_activations_map[0] + 204);
    obs_output_array.data_start = AI_PTR(g_nn_arm_a_range_activations_map[0] + 204);
    
    _actor_actor_0_Gemm_output_0_output_array.data = AI_PTR(g_nn_arm_a_range_activations_map[0] + 256);
    _actor_actor_0_Gemm_output_0_output_array.data_start = AI_PTR(g_nn_arm_a_range_activations_map[0] + 256);
    
    _actor_actor_1_Relu_output_0_output_array.data = AI_PTR(g_nn_arm_a_range_activations_map[0] + 256);
    _actor_actor_1_Relu_output_0_output_array.data_start = AI_PTR(g_nn_arm_a_range_activations_map[0] + 256);
    
    _actor_actor_2_Gemm_output_0_output_array.data = AI_PTR(g_nn_arm_a_range_activations_map[0] + 0);
    _actor_actor_2_Gemm_output_0_output_array.data_start = AI_PTR(g_nn_arm_a_range_activations_map[0] + 0);
    
    _actor_actor_1_1_Relu_output_0_output_array.data = AI_PTR(g_nn_arm_a_range_activations_map[0] + 256);
    _actor_actor_1_1_Relu_output_0_output_array.data_start = AI_PTR(g_nn_arm_a_range_activations_map[0] + 256);
    
    actions_output_array.data = AI_PTR(g_nn_arm_a_range_activations_map[0] + 0);
    actions_output_array.data_start = AI_PTR(g_nn_arm_a_range_activations_map[0] + 0);
    
    actions_tanh_output_array.data = AI_PTR(g_nn_arm_a_range_activations_map[0] + 8);
    actions_tanh_output_array.data_start = AI_PTR(g_nn_arm_a_range_activations_map[0] + 8);
    
    current_a_output_array.data = AI_PTR(g_nn_arm_a_range_activations_map[0] + 0);
    current_a_output_array.data_start = AI_PTR(g_nn_arm_a_range_activations_map[0] + 0);
    
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool nn_arm_a_range_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_nn_arm_a_range_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    _actor_actor_0_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _actor_actor_0_Gemm_output_0_bias_array.data = AI_PTR(g_nn_arm_a_range_weights_map[0] + 0);
    _actor_actor_0_Gemm_output_0_bias_array.data_start = AI_PTR(g_nn_arm_a_range_weights_map[0] + 0);
    
    _actor_actor_2_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _actor_actor_2_Gemm_output_0_weights_array.data = AI_PTR(g_nn_arm_a_range_weights_map[0] + 256);
    _actor_actor_2_Gemm_output_0_weights_array.data_start = AI_PTR(g_nn_arm_a_range_weights_map[0] + 256);
    
    _actor_actor_2_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _actor_actor_2_Gemm_output_0_bias_array.data = AI_PTR(g_nn_arm_a_range_weights_map[0] + 16640);
    _actor_actor_2_Gemm_output_0_bias_array.data_start = AI_PTR(g_nn_arm_a_range_weights_map[0] + 16640);
    
    actions_weights_array.format |= AI_FMT_FLAG_CONST;
    actions_weights_array.data = AI_PTR(g_nn_arm_a_range_weights_map[0] + 16896);
    actions_weights_array.data_start = AI_PTR(g_nn_arm_a_range_weights_map[0] + 16896);
    
    actions_bias_array.format |= AI_FMT_FLAG_CONST;
    actions_bias_array.data = AI_PTR(g_nn_arm_a_range_weights_map[0] + 17408);
    actions_bias_array.data_start = AI_PTR(g_nn_arm_a_range_weights_map[0] + 17408);
    
    output_scale_array.format |= AI_FMT_FLAG_CONST;
    output_scale_array.data = AI_PTR(g_nn_arm_a_range_weights_map[0] + 17416);
    output_scale_array.data_start = AI_PTR(g_nn_arm_a_range_weights_map[0] + 17416);
    
    _actor_actor_0_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _actor_actor_0_Gemm_output_0_weights_array.data = AI_PTR(g_nn_arm_a_range_weights_map[0] + 17420);
    _actor_actor_0_Gemm_output_0_weights_array.data_start = AI_PTR(g_nn_arm_a_range_weights_map[0] + 17420);
    
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/


AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_nn_arm_a_range_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NN_ARM_A_RANGE_MODEL_NAME,
      .model_signature   = AI_NN_ARM_A_RANGE_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 5336,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_bool ai_nn_arm_a_range_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NN_ARM_A_RANGE_MODEL_NAME,
      .model_signature   = AI_NN_ARM_A_RANGE_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 5336,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}

AI_API_ENTRY
ai_error ai_nn_arm_a_range_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_nn_arm_a_range_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_error ai_nn_arm_a_range_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
    ai_error err;
    ai_network_params params;

    err = ai_nn_arm_a_range_create(network, AI_NN_ARM_A_RANGE_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE)
        return err;
    if (ai_nn_arm_a_range_data_params_get(&params) != true) {
        err = ai_nn_arm_a_range_get_error(*network);
        return err;
    }
#if defined(AI_NN_ARM_A_RANGE_DATA_ACTIVATIONS_COUNT)
    if (activations) {
        /* set the addresses of the activations buffers */
        for (int idx=0;idx<params.map_activations.size;idx++)
            AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
    }
#endif
#if defined(AI_NN_ARM_A_RANGE_DATA_WEIGHTS_COUNT)
    if (weights) {
        /* set the addresses of the weight buffers */
        for (int idx=0;idx<params.map_weights.size;idx++)
            AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
    }
#endif
    if (ai_nn_arm_a_range_init(*network, &params) != true) {
        err = ai_nn_arm_a_range_get_error(*network);
    }
    return err;
}

AI_API_ENTRY
ai_buffer* ai_nn_arm_a_range_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    ((ai_network *)network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}

AI_API_ENTRY
ai_buffer* ai_nn_arm_a_range_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    ((ai_network *)network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}

AI_API_ENTRY
ai_handle ai_nn_arm_a_range_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_nn_arm_a_range_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if (!net_ctx) return false;

  ai_bool ok = true;
  ok &= nn_arm_a_range_configure_weights(net_ctx, params);
  ok &= nn_arm_a_range_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_nn_arm_a_range_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_nn_arm_a_range_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_NN_ARM_A_RANGE_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

