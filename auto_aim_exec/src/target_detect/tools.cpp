#include "target_detect/tools.hpp"

#include <iostream>

#include "sys/stat.h"
#include "sys/fcntl.h"
#include <unistd.h>
void Logger::log(Severity Severity, const char * msg) noexcept{
    if(Severity <= Severity::kWARNING){
        std::cout<<msg<<std::endl;
    }
}

size_t get_dim_size(const nvinfer1::Dims & dims){
    size_t ret = 1;
    for(int i = 0; i < dims.nbDims; ++i){
        ret *= dims.d[i];
    }
    return ret;
}



size_t read_engine_file(const char * engine_file_path, std::vector<uint8_t>  & engine_string){
    //!
    //! \brief: read the engine_file
    //! \returns: size of engine file
    //!
    struct stat buf;
    int engine_fd = open(engine_file_path, O_RDONLY);
    fstat(engine_fd, &buf);
    size_t engine_size = buf.st_size;
    engine_string.reserve(engine_size);
    read(engine_fd, engine_string.data(), engine_size);
    return engine_size;
}