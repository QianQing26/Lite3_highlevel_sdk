/**
 * @file onnx_runner.hpp
 * @brief ONNX Runtime inference wrapper (C++ API)
 */
#pragma once

#include <string>
#include <vector>
#include <memory>

namespace lite3 {

// Forward-declare Ort types (PIMPL pattern — hides onnxruntime headers)
struct OrtImpl;

class OnnxRunner {
public:
    /**
     * @param model_path  Path to .onnx model file
     */
    explicit OnnxRunner(const std::string& model_path);
    ~OnnxRunner();

    int obsDim() const { return obs_dim_; }
    int actDim() const { return act_dim_; }

    /**
     * @brief Run inference.
     * @param obs  Observation vector (size == obsDim())
     * @param out  Output buffer (size >= actDim()), filled with raw action
     */
    void infer(const float* obs, float* out);

private:
    std::unique_ptr<OrtImpl> ort_;
    int obs_dim_ = 0;
    int act_dim_ = 0;
};

} // namespace lite3
