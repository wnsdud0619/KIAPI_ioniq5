// Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
// modify from
// https://github.com/NVIDIA/TensorRT/tree/master/plugin/batchedNMSPlugin
#ifndef TRT_BATCHED_NMS__BATCHED_NMS__TRT_BATCHED_NMS_HPP_
#define TRT_BATCHED_NMS__BATCHED_NMS__TRT_BATCHED_NMS_HPP_

#include "NvInferPluginUtils.h"
#include "NvInferRuntime.h"
#include "NvInferVersion.h"
#include "trt_batched_nms/trt_plugin_helper.hpp"

#include <memory>
#include <string>
#include <vector>

namespace mmdeploy
{

#if NV_TENSORRT_MAJOR > 7
#define TRT_NOEXCEPT noexcept
#else
#define TRT_NOEXCEPT
#endif

enum NMSReturnType { RETURN_DETS = 1, RETURN_INDEX = 1 << 1 };

class TRTBatchedNMS : public nvinfer1::IPluginV2DynamicExt
{
public:
  TRTBatchedNMS(const std::string & name, nvinfer1::plugin::NMSParameters params, bool returnIndex);

  TRTBatchedNMS(const std::string & name, const void * data, size_t length);

  explicit TRTBatchedNMS(const std::string & name) : mLayerName(name) {}

  ~TRTBatchedNMS() TRT_NOEXCEPT override = default;

  int getNbOutputs() const TRT_NOEXCEPT override;

  nvinfer1::DimsExprs getOutputDimensions(
    int outputIndex, const nvinfer1::DimsExprs * inputs, int nbInputs,
    nvinfer1::IExprBuilder & exprBuilder) TRT_NOEXCEPT override;

  size_t getWorkspaceSize(
    const nvinfer1::PluginTensorDesc * inputs, int nbInputs,
    const nvinfer1::PluginTensorDesc * outputs, int nbOutputs) const TRT_NOEXCEPT override;

  int enqueue(
    const nvinfer1::PluginTensorDesc * inputDesc, const nvinfer1::PluginTensorDesc * outputDesc,
    const void * const * inputs, void * const * outputs, void * workSpace,
    cudaStream_t stream) TRT_NOEXCEPT override;

  size_t getSerializationSize() const TRT_NOEXCEPT override;

  void serialize(void * buffer) const TRT_NOEXCEPT override;

  void configurePlugin(
    const nvinfer1::DynamicPluginTensorDesc * inputs, int nbInputs,
    const nvinfer1::DynamicPluginTensorDesc * outputs, int nbOutputs) TRT_NOEXCEPT override;

  bool supportsFormatCombination(
    int pos, const nvinfer1::PluginTensorDesc * ioDesc, int nbInputs,
    int nbOutputs) TRT_NOEXCEPT override;

  const char * getPluginType() const TRT_NOEXCEPT override;

  const char * getPluginVersion() const TRT_NOEXCEPT override;

  nvinfer1::IPluginV2DynamicExt * clone() const TRT_NOEXCEPT override;

  nvinfer1::DataType getOutputDataType(
    int index, const nvinfer1::DataType * inputTypes, int nbInputs) const TRT_NOEXCEPT override;

  void setClipParam(bool clip);

  // IPluginV2 Methods
  int initialize() TRT_NOEXCEPT override { return STATUS_SUCCESS; }

  void terminate() TRT_NOEXCEPT override {}

  void destroy() TRT_NOEXCEPT override { delete this; }

  void setPluginNamespace(const char * pluginNamespace) TRT_NOEXCEPT override
  {
    mNamespace = pluginNamespace;
  }

  const char * getPluginNamespace() const TRT_NOEXCEPT override { return mNamespace.c_str(); }

  void attachToContext(
    [[maybe_unused]] cudnnContext * cudnnContext, [[maybe_unused]] cublasContext * cublasContext,
    [[maybe_unused]] nvinfer1::IGpuAllocator * gpuAllocator) TRT_NOEXCEPT override
  {
  }

  void detachFromContext() TRT_NOEXCEPT override {}

private:
  const std::string mLayerName;
  std::string mNamespace;

  nvinfer1::plugin::NMSParameters param{};
  bool mClipBoxes{};
  bool mReturnIndex{};
};

class TRTBatchedNMSCreator : public nvinfer1::IPluginCreator
{
public:
  TRTBatchedNMSCreator();

  ~TRTBatchedNMSCreator() TRT_NOEXCEPT override = default;

  const char * getPluginName() const TRT_NOEXCEPT override;

  const char * getPluginVersion() const TRT_NOEXCEPT override;

  const char * getPluginNamespace() const TRT_NOEXCEPT override { return mNamespace.c_str(); }

  const nvinfer1::PluginFieldCollection * getFieldNames() TRT_NOEXCEPT override { return &mFC; }

  nvinfer1::IPluginV2DynamicExt * createPlugin(
    const char * name, const nvinfer1::PluginFieldCollection * fc) TRT_NOEXCEPT override;

  nvinfer1::IPluginV2DynamicExt * deserializePlugin(
    const char * name, const void * serialData, size_t serialLength) TRT_NOEXCEPT override;

  void setPluginNamespace(const char * pluginNamespace) TRT_NOEXCEPT override
  {
    mNamespace = pluginNamespace;
  }

private:
  nvinfer1::PluginFieldCollection mFC;
  std::vector<nvinfer1::PluginField> mPluginAttributes;
  std::string mNamespace;
};
}  // namespace mmdeploy
#endif  // TRT_BATCHED_NMS__BATCHED_NMS__TRT_BATCHED_NMS_HPP_
