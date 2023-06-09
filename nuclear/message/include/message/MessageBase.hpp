#ifndef MESSAGE_MESSAGEBASE_HPP
#define MESSAGE_MESSAGEBASE_HPP

#include <nuclear>

namespace message {
    template <typename T>
    class MessageBase : public std::enable_shared_from_this<T> {};
}  // namespace message

namespace NUClear::util::serialise {

    template <typename T>
    struct Serialise<T, std::enable_if_t<std::is_base_of<::message::MessageBase<std::remove_cv_t<T>>, T>::value, T>> {

        using protobuf_type = typename T::protobuf_type;

        static inline std::vector<char> serialise(const T& in) {

            protobuf_type proto = in;

            std::vector<char> output(proto.ByteSizeLong());
            proto.SerializeToArray(output.data(), output.size());

            return output;
        }

        [[nodiscard]] static inline T deserialise(const char* in, const size_t& length) {

            // Make a buffer
            protobuf_type out;

            // Deserialize it
            if (!out.ParseFromArray(in, length)) {
                throw std::runtime_error("Message failed to deserialise.");
            }

            return out;
        }

        [[nodiscard]] static inline T deserialise(const std::vector<char>& in) {
            return deserialise(in.data(), in.size());
        }

        static inline uint64_t hash() {

            // We have to construct an instance to call the reflection functions
            protobuf_type type;

            // We have to remove the 'protobuf' namespace
            std::string typeName = type.GetTypeName().substr(9);

            // We base the hash on the name of the protocol buffer, removing the protobuf prefix on typeName
            return XXH64(typeName.c_str(), typeName.size(), 0x4e55436c);
        }
    };

}  // namespace NUClear::util::serialise

#endif  // MESSAGE_MESSAGEBASE_HPP
