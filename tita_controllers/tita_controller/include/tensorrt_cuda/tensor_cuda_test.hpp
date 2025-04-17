#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class Logger : public nvinfer1::ILogger
{
  void log(Severity severity, const char *msg) noexcept override
  {
    (void)severity;
    (void)msg;
  }
};

class CudaTest
{
private:
  bool cuda_init = false;
  nvinfer1::ICudaEngine *get_engine(const std::string &engine_file_path);
  void cuda_memory_init(void);
  // Cuda pipeline config.
  float *buffers[3];
  size_t input_size_0 = 33 * sizeof(float);
  size_t input_size_1 = 33 * 10 * sizeof(float);
  size_t output_size = 8 * sizeof(float);
  cudaStream_t stream;
  nvinfer1::ICudaEngine *engine_;
  nvinfer1::IExecutionContext *context;
  Logger gLogger;

public:
  void do_inference(const float *input_0, const float *input_1, float *output);
  bool get_cuda_init(void);
  explicit CudaTest(const std::string &engine_file_path);
  ~CudaTest();
};

// nvinfer1::ICudaEngine * CudaTest::get_engine(const std::string & engine_file_path)
// {
//   std::ifstream file(engine_file_path, std::ios::binary);
//   if (!file.good()) {
//     return nullptr;
//   }
//   std::vector<char> engine_data(
//     (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
//   file.close();
//   nvinfer1::IRuntime * runtime = nvinfer1::createInferRuntime(gLogger);
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
//   return runtime->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr);
// #pragma GCC diagnostic pop
// }

// void CudaTest::cuda_memory_init(void)
// {
//   cudaStreamCreate(&stream);
//   cudaMalloc(reinterpret_cast<void **>(&buffers[0]), input_size_0);
//   cudaMalloc(reinterpret_cast<void **>(&buffers[1]), input_size_1);
//   cudaMalloc(reinterpret_cast<void **>(&buffers[2]), output_size);
// }
// // Function to do inference
// void CudaTest::do_inference(
//   const float * input_0, const float * input_1, float * output)
// {
//   cudaMemcpyAsync(buffers[0], input_0, input_size_0, cudaMemcpyHostToDevice, stream);
//   cudaMemcpyAsync(buffers[1], input_1, input_size_1, cudaMemcpyHostToDevice, stream);
// // context->enqueue(1, (void**)buffers, stream, nullptr);
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
//   context->enqueueV2(reinterpret_cast<void **>(buffers), stream, nullptr);
// #pragma GCC diagnostic pop
//   cudaMemcpyAsync(output, buffers[2], output_size, cudaMemcpyDeviceToHost, stream);
//   cudaStreamSynchronize(stream);
// }

// bool CudaTest::get_cuda_init(void) { return cuda_init; }

// CudaTest::CudaTest(const std::string & engine_file_path)
// {
//   engine_ = get_engine(engine_file_path);
//   if (engine_ != nullptr) {
//     context = engine_->createExecutionContext();
//     cuda_memory_init();
//     cuda_init = true;
//   } else {
//     cuda_init = false;
//   }
// }

// CudaTest::~CudaTest()
// {
//   cudaStreamDestroy(stream);
//   for (void * buf : buffers) {
//     cudaFree(buf);
//   }
// }
