#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_MAKE_MOSAIC_KERNEL_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_MAKE_MOSAIC_KERNEL_HPP

#include "../CompressionContext.hpp"

namespace module::output::compressor::vaapi::cl {

    std::pair<cl::program, cl::kernel> make_mosaic_kernel(const CompressionContext::OpenCLContext& context,
                                                          const uint32_t& width,
                                                          const uint32_t& height,
                                                          const uint32_t& format);

}  // namespace module::output::compressor::vaapi::cl

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_MAKE_MOSAIC_KERNEL_HPP
