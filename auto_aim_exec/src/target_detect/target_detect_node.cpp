#include "target_detect/target_detect_node.hpp"

namespace target_detect
{
  TargetDetectNode::TargetDetectNode(rclcpp::NodeOptions const& options) : Node("target_detect", options)
  {
    // read the serialized engine file
    Logger logger;
    const char *engine_file_path = "/home/ros/Desktop/user/onnx_to_engine/nx_float16_armor_det.engine";
    std::vector<uint8_t> engine_string;
    size_t engine_size = read_engine_file(engine_file_path, engine_string);

    // decoder engine and create execute context
    nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(logger);
    obj_engine = runtime->deserializeCudaEngine(engine_string.data(), engine_size);
    obj_context = obj_engine->createExecutionContext();

    // allocate the memory for input and output
    obj_input_idx = obj_engine->getBindingIndex("x"), obj_output_idx = obj_engine->getBindingIndex("y");
    obj_input_dims = obj_engine->getBindingDimensions(obj_input_idx);
    obj_output_dims = obj_engine->getBindingDimensions(obj_output_idx);
    obj_input_size = get_dim_size(obj_input_dims), obj_output_size = get_dim_size(obj_output_dims);
    RCLCPP_INFO(this->get_logger(), "input size %d", obj_input_size);
    cudaMalloc(&obj_buffers[obj_input_idx], obj_input_size * sizeof(float));
    cudaMalloc(&obj_buffers[obj_output_idx], obj_output_size * sizeof(float));

    RCLCPP_INFO(this->get_logger(), "execute context build successful\n");

    this->output_pub = this->create_publisher<auto_aim_interface::msg::ObjectDetOutput>(
        "/object_det_output", 10);
    int iHeightMax = 1024, iWidthMax = 1280;
    ori_buf_size = iHeightMax * iWidthMax * 3;
    inter_buf_size = obj_input_size;

    cudaHostAlloc((void **)&ori_buf, ori_buf_size * sizeof(uint8_t), cudaHostAllocMapped);
    cudaMalloc((void **)&image_resize_buf, inter_buf_size * sizeof(uint8_t));
    cudaMalloc((void **)&image_float_buf, inter_buf_size * sizeof(float32_t));
    cudaMalloc((void **)&input_buf, inter_buf_size * sizeof(float32_t));
    auto callback_ = [this](sensor_msgs::msg::Image::SharedPtr msg) -> void
    {
      // RCLCPP_INFO(this->get_logger(), "in callback");
      cudaMemcpy(this->ori_buf, msg->data.data(), msg->height * msg->width * 3, cudaMemcpyHostToHost);
      // cudaMemcpy(output_msg_temp.image.data(), msg->data.data(), msg->height * msg->width * 3, cudaMemcpyHostToHost);
      output_msg_temp.raw_image = *msg;
      // output_msg_temp.image.resize(msg->height * msg->width * 3);
      this->Pre_Process(this->obj_input_height, this->obj_input_width);
      this->do_obj_inference();
    //   if(frame_temp++ % 100 == 0){
    //   RCLCPP_INFO(this->get_logger(), "hello %d", frame_temp / 100);
    //  }
    };
    sub_ = create_subscription<sensor_msgs::msg::Image>("image_raw", 4, callback_);
  }
  void TargetDetectNode::do_obj_inference()
  {
    auto_aim_interface::msg::ObjectDetOutput::UniquePtr output_msg(new auto_aim_interface::msg::ObjectDetOutput());


    output_msg->width = this->obj_input_width;
    output_msg->height = this->obj_input_height;
    // cudaMemcpy(output_msg_temp.image.data(), this->image_resize_buf, obj_input_size, cudaMemcpyDeviceToHost);
    // output_msg_temp.image.resize(obj_input_size);
    //output_msg->image = output_msg_temp.image;
    output_msg->raw_image = output_msg_temp.raw_image;

    cudaMemcpy(obj_buffers[obj_input_idx], this->input_buf,
               obj_input_size * sizeof(float32_t), cudaMemcpyDeviceToDevice);
    obj_context->executeV2(obj_buffers);

    output_msg->na = obj_output_dims.d[1];
    output_msg->no = obj_output_dims.d[2];
    
    cudaMemcpy(output_msg_temp.output.data(), obj_buffers[obj_output_idx], obj_output_size * sizeof(float32_t), cudaMemcpyDeviceToHost);
    output_msg_temp.output.resize(obj_output_size);
    output_msg->output = output_msg_temp.output;
    // RCLCPP_INFO(this->get_logger(), "publish successful");
    output_pub->publish(std::move(output_msg));
  }
  void TargetDetectNode::Pre_Process(int dst_height, int dst_width)
  {
    cudaHostGetDevicePointer((void **)&this->dev_ori_buf, (void*)this->ori_buf, 0);
    NppiSize srcSize{image_width, image_height};
    NppiSize dstSize{dst_width, dst_height};
    NppiRect srcROI = {0, 0, image_width, image_height};
    NppiRect dstROI = {0, 0, dst_width, dst_height};

    std::unique_lock<std::mutex> resize_buf_lk(resize_buf_mutex, std::defer_lock);
    resize_buf_lk.lock();
    nppiResize_8u_C3R(dev_ori_buf, image_width * 3, srcSize, srcROI,
                      image_resize_buf, dst_width * 3, dstSize, dstROI, NPPI_INTER_LINEAR);
    nppiConvert_8u32f_C3R(image_resize_buf, dst_width * 3,
                          image_float_buf, dst_width * 3 * sizeof(float32_t), dstSize);
    resize_buf_lk.unlock();

    nppiMulC_32f_C3IR(input_scale, image_float_buf, dst_width * 3 * sizeof(float32_t), dstSize);
    nppiSwapChannels_32f_C3IR(image_float_buf, dst_width * 3 * sizeof(float32_t), dstSize, this->DstOrder);

    std::lock_guard<std::mutex> input_buf_lk(this->input_buf_mutex);
    float32_t *r_plane = (float32_t *)(this->input_buf);
    float32_t *g_plane = (float32_t *)(this->input_buf + dst_width * dst_height);
    float32_t *b_plane = (float32_t *)(this->input_buf + dst_width * dst_height * 2);
    float32_t *dst_planes[3] = {r_plane, g_plane, b_plane};
    nppiCopy_32f_C3P3R(image_float_buf, dst_width * 3 * sizeof(float32_t),
                       dst_planes, dst_width * sizeof(float32_t), dstSize);
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(target_detect::TargetDetectNode)
