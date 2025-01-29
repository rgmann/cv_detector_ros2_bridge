// Copyright [2025] [Robert Vaughan]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __MESSAGE_H__
#define __MESSAGE_H__

#include <vector>
#include <memory>
#include <cstdlib>
#include <cstring>

class message {
public:

    typedef std::shared_ptr<message> ptr;

    static constexpr size_t HEADER_LENGTH = 4;
    static constexpr size_t MAX_BODY_LENGTH = 1024;

    message() : data_(HEADER_LENGTH + MAX_BODY_LENGTH), body_length_(0) {}

    bool decode_header()
    {
        char header[HEADER_LENGTH + 1] = {0};
        std::memcpy(header, data(), HEADER_LENGTH);

        body_length_ = std::atoi(header);
        if (body_length_ > MAX_BODY_LENGTH)
        {
            body_length_ = 0;
            return false;
        }
        return true;
    }

    uint8_t* data()
    {
        return data_.data();
    }

    const uint8_t* data() const
    {
        return data_.data();
    }

    uint8_t* body()
    {
        return (data_.data() + HEADER_LENGTH);
    }
    const uint8_t* body() const
    {
        return (data_.data() + HEADER_LENGTH);
    }

    size_t body_length() const
    {
        return body_length_;
    }

private:

    std::vector<uint8_t> data_;
    size_t body_length_;

};

#endif // __MESSAGE_H__