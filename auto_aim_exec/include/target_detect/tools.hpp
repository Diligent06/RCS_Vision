#ifndef TOOLS_HPP
#define TOOLS_HPP

#include "cuda.h"
#include "NvInfer.h"

#include <vector>

size_t get_dim_size(const nvinfer1::Dims & dims);

size_t read_engine_file(const char * engine_file_path, std::vector<uint8_t>  & engine_string);

class Logger : public nvinfer1::ILogger{
    void log(Severity Severity, const char * msg) noexcept override;
};

#endif