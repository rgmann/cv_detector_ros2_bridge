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

#include "detections_list_client.hpp"

using namespace boost::asio::ip;

detections_list_client::detections_list_client(
    boost::asio::io_context& context,
    const tcp::resolver::results_type& endpoints,
    sync_queue<message::ptr>& queue)
    : io_context_(context)
    , socket_(context)
    , queue_(queue)
{
    do_connect(endpoints);
}

void detections_list_client::close()
{
    boost::asio::post(io_context_, [this]() { socket_.close(); });
}

void detections_list_client::do_connect(const tcp::resolver::results_type& endpoints)
{
    boost::asio::async_connect(socket_, endpoints,
        [this](boost::system::error_code ec, tcp::endpoint)
        {
            if (!ec)
            {
                do_read_header();
            }
        });
}

void detections_list_client::do_read_header()
{
    msg_ = std::make_shared<message>();

    boost::asio::async_read(socket_,
        boost::asio::buffer(msg_->data(), message::HEADER_LENGTH),
        [this](boost::system::error_code ec, std::size_t /*length*/)
        {
            if (!ec && msg_->decode_header())
            {
                do_read_body();
            }
            else
            {
                socket_.close();
            }
        }
    );
}

void detections_list_client::do_read_body()
{
    boost::asio::async_read(socket_,
        boost::asio::buffer(msg_->body(), msg_->body_length()),
        [this](boost::system::error_code ec, std::size_t /*length*/)
        {
            if (!ec)
            {
                queue_.push(msg_);
                do_read_header();
            }
            else
            {
                socket_.close();
            }
        }
    );
}