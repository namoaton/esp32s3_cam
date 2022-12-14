/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "tensorflow/lite/kernels/internal/reference/reduce.h"

#include "tensorflow/lite/c/builtin_op_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/kernels/internal/quantization_util.h"
#include "tensorflow/lite/kernels/internal/reference/integer_ops/mean.h"
#include "tensorflow/lite/kernels/internal/tensor_ctypes.h"
#include "tensorflow/lite/kernels/internal/types.h"
#include "tensorflow/lite/kernels/kernel_util.h"

namespace tflite
{
  namespace ops
  {
    namespace micro
    {
      namespace reduce
      {
        struct OpDataReduce
        {
          int32_t multiplier;
          int shift;
          int temp_buffer_idx;
          int resolved_axis_idx;
          int input_zp;
          float input_scale;
          int output_zp;
          float output_scale;
          int num_output_elements;
        };
        constexpr int kMaxNumberOfAxis = 4;
        constexpr int kMaxNumberOfReducedAxis = 2;

        TfLiteStatus PrepareSimple(TfLiteContext *context, TfLiteNode *node)
        {
          // Inputs Tensor (dtype depends on quantization):
          // [0] = Input
          // [1] = Axis

          // Outputs Tensor (dtype depends on quantization):
          // [0] = Output

          // Validate number of inputs and outputs
          TF_LITE_ENSURE_EQ(context, node->inputs->size, 2);
          TF_LITE_ENSURE_EQ(context, node->outputs->size, 1);

          // Validate axis type
          const TfLiteTensor *axis = GetInput(context, node, 1);
          TF_LITE_ENSURE_TYPES_EQ(context, axis->type, kTfLiteInt32);
          return kTfLiteOk;
        }

        TfLiteStatus PrepareMeanOrSum(TfLiteContext *context, TfLiteNode *node)
        {
          TF_LITE_ENSURE_OK(context, PrepareSimple(context, node));
          // TODO(b/144955155): Support uint8(b/144955155) and int8(b/144955018)
          return kTfLiteOk;
        }

        void ResolveAxis(const int *axis_data, int axis_count,
                         tflite::MeanParams *op_params)
        {
          int i = 0;
          for (; i < axis_count; ++i)
          {
            op_params->axis[i] = static_cast<int16>(axis_data[i]);
          }
          for (; i < 4; ++i)
          {
            op_params->axis[i] = 1;
          }
          op_params->axis_count = axis_count;
        }

        TfLiteStatus EvalMean(TfLiteContext *context, TfLiteNode *node)
        {
          const TfLiteTensor *input = GetInput(context, node, 0);
          const TfLiteTensor *axis = GetInput(context, node, 1);
          TfLiteTensor *output = GetOutput(context, node, 0);
          TfLiteReducerParams *params =
              reinterpret_cast<TfLiteReducerParams *>(node->builtin_data);

          OpDataReduce* op_data = static_cast<OpDataReduce *>(node->user_data) ;
          int num_axis = static_cast<int>(NumElements(axis));
          int temp_index[kMaxNumberOfAxis];
          int resolved_axis[kMaxNumberOfReducedAxis];

          tflite::MeanParams op_params;
          ResolveAxis(GetTensorData<int>(axis), num_axis, &op_params);
          // Special case mean implementation exists for 4D mean across axes 1 and 2.
          bool special_case_4d_axes_1_and_2 =
              input->dims->size == 4 && op_params.axis_count == 2 &&
              ((op_params.axis[0] == 1 && op_params.axis[1] == 2) ||
               (op_params.axis[0] == 2 && op_params.axis[1] == 1));
          switch (input->type)
          {
          case kTfLiteFloat32:
          {

            if (params->keep_dims && special_case_4d_axes_1_and_2)
            {
              reference_ops::Mean(op_params, GetTensorShape(input),
                                  GetTensorData<float>(input), GetTensorShape(output),
                                  GetTensorData<float>(output));
            }
            else
            {
              TF_LITE_ENSURE(
                  context,
                  reference_ops::Mean(GetTensorData<float>(input), input->dims->data,
                                      input->dims->size, GetTensorData<float>(output),
                                      output->dims->data, output->dims->size,
                                      GetTensorData<int>(axis), num_axis,
                                      params->keep_dims, temp_index, resolved_axis,
                                      GetTensorData<float>(output)));
            }
          }
          break;
          case kTfLiteInt8:
          {
            // Defer to specialized implementation for 4D Mean across axes 1 & 2.
            if (params->keep_dims && special_case_4d_axes_1_and_2)
            {
              tflite::reference_integer_ops::Mean(
                  op_params, op_data->multiplier, op_data->shift,
                  GetTensorShape(input),
                  GetTensorData<int8_t>(input), op_data->input_zp,
                  GetTensorShape(output),
                  GetTensorData<int8_t>(output), op_data->output_zp);
            }
            else //if (op_data->input_zp == op_data->output_zp &&
                 //    op_data->input_scale == op_data->output_scale)
            {
              // int32_t *temp_buffer = static_cast<int32_t *>(
              //     context->GetScratchBuffer(context, op_data->temp_buffer_idx));
              TF_LITE_ENSURE(
                  context,
                  reference_ops::Mean(GetTensorData<int8_t>(input), input->dims->data,
                                      input->dims->size, GetTensorData<int8_t>(output),
                                      output->dims->data, output->dims->size,
                                      GetTensorData<int>(axis), num_axis,
                                      params->keep_dims, temp_index, resolved_axis, 
                                      GetTensorData<int8_t>(output)));
            }
            /*else {
              // int32_t *temp_buffer = static_cast<int32_t *>(
              //     context->GetScratchBuffer(context, op_data->temp_buffer_idx));
              TF_LITE_ENSURE(
                  context,
                  reference_ops::QuantizedMeanOrSum(GetTensorData<int8_t>(input), op_data->input_zp,
                      op_data->input_scale, input->dims->data, input->dims->size,
                      GetTensorData<int8_t>(output),
                      op_data->output_zp, op_data->output_scale, output->dims->data,
                      output->dims->size,GetTensorData<int>(axis),
                      num_axis, params->keep_dims, temp_index, resolved_axis,
                      GetTensorData<int8_t>(output), false));
            }*/
          }
          break;
          default:
            // TODO(b/144955155): Support uint8(b/144955155) and int8(b/144955018)
            TF_LITE_ENSURE_MSG(context, false,
                               "Currently, only float32 input type "
                               "is supported.");
          }
          return kTfLiteOk;
        }
      } // namespace reduce

      TfLiteRegistration *Register_MEAN()
      {
        static TfLiteRegistration r = {/*init=*/nullptr,
                                       /*free=*/nullptr,
                                       /*prepare=*/reduce::PrepareMeanOrSum,
                                       /*invoke=*/reduce::EvalMean,
                                       /*profiling_string=*/nullptr,
                                       /*builtin_code=*/0,
                                       /*custom_name=*/nullptr,
                                       /*version=*/0};
        return &r;
      }
    } // namespace micro
  }   // namespace ops
} // namespace tflite
