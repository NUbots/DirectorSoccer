#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_UPLOAD_TO_SURFACE_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_UPLOAD_TO_SURFACE_HPP

#include <va/va.h>
#include <vector>

namespace module::output::compressor::vaapi::operation {

    void upload_to_surface(VADisplay dpy,
                           const std::vector<uint8_t>& data,
                           const uint32_t& width,
                           const uint32_t& height,
                           const uint32_t& format,
                           VASurfaceID surface_id);

}  // namespace module::output::compressor::vaapi::operation

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_OPERATION_UPLOAD_TO_SURFACE_HPP
