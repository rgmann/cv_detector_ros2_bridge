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

#ifndef __DETECTIONS_LIST_CLIENT_H__
#define __DETECTIONS_LIST_CLIENT_H__

#include <boost/asio.hpp>
#include "sync_queue.hpp"
#include "message.hpp"

class detections_list_client {
public:

    detections_list_client(
        boost::asio::io_context& context,
        const boost::asio::ip::tcp::resolver::results_type& endpoints,
        sync_queue<message::ptr>& queue);

    void close();

private:

    void do_connect(const boost::asio::ip::tcp::resolver::results_type& endpoints);

    void do_read_header();

    void do_read_body();


private:

    boost::asio::io_context& io_context_;
    boost::asio::ip::tcp::socket socket_;
    sync_queue<message::ptr>& queue_;
    message::ptr msg_;
};

#endif // __DETECTIONS_LIST_CLIENT_H__