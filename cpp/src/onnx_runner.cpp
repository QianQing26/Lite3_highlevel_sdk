/**
 * @file onnx_runner.cpp
 * @brief ONNX Runtime inference wrapper — implementation (PIMPL)
 */

#include "lite3_highlevel/onnx_runner.hpp"
#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <cstring>

namespace lite3 {

struct OrtImpl {
    Ort::Env             env;
    Ort::SessionOptions  opts;
    Ort::MemoryInfo      mem_info;
    Ort::Session         session;
    const char*          input_name;
    const char*          output_name;

    OrtImpl(const std::string& model_path)
        : env(ORT_LOGGING_LEVEL_WARNING, "Lite3Highlevel")
        , mem_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
        , session(env, model_path.c_str(), opts)
    {
        input_name  = session.GetInputName(0, Ort::AllocatorWithDefaultOptions());
        output_name = session.GetOutputName(0, Ort::AllocatorWithDefaultOptions());
    }
};

OnnxRunner::OnnxRunner(const std::string& model_path) {
    ort_ = std::make_unique<OrtImpl>(model_path);

    auto input_info  = ort_->session.GetInputTypeInfo(0);
    auto output_info = ort_->session.GetOutputTypeInfo(0);
    auto in_shape  = input_info.GetTensorTypeAndShapeInfo().GetShape();
    auto out_shape = output_info.GetTensorTypeAndShapeInfo().GetShape();
    obs_dim_ = static_cast<int>(in_shape.back());
    act_dim_ = static_cast<int>(out_shape.back());

    std::cout << "[OnnxRunner] Loaded " << model_path << std::endl;
    std::cout << "  Input : " << ort_->input_name << " [1, " << obs_dim_ << "]" << std::endl;
    std::cout << "  Output: " << ort_->output_name << " [1, " << act_dim_ << "]" << std::endl;

    // Warm-up inference
    std::vector<float> dummy_obs(obs_dim_, 0.0f);
    std::vector<float> dummy_out(act_dim_);
    infer(dummy_obs.data(), dummy_out.data());
}

OnnxRunner::~OnnxRunner() = default;

void OnnxRunner::infer(const float* obs, float* out) {
    std::vector<int64_t> shape = {1, obs_dim_};
    auto tensor = Ort::Value::CreateTensor<float>(
        ort_->mem_info, const_cast<float*>(obs), static_cast<size_t>(obs_dim_),
        shape.data(), shape.size());

    auto outputs = ort_->session.Run(
        Ort::RunOptions{nullptr},
        &ort_->input_name, &tensor, 1,
        &ort_->output_name, 1);

    const float* data = outputs[0].GetTensorData<float>();
    std::memcpy(out, data, act_dim_ * sizeof(float));
}

} // namespace lite3
