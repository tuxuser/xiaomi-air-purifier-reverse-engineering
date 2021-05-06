/* Copyright (C) 2020 Oxan van Leeuwen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "esphome.h"

#include <memory>
#include <sstream>

class MiiotUart: public Component, public UARTDevice {
public:
    MiiotUart(UARTComponent *parent) : UARTDevice(parent) {}

    // Implement this in your app
    virtual std::string handle_miiot(std::string command, std::vector<std::string> args) = 0;

    void setup() override {
        // nothing to do here
    }

    void loop() override {
        auto ch = read();
        if (ch == '\r') {
            // Append null terminator
            this->recv_buf_.push_back('\0');
            // Incoming MIIOT message is complete
            ESP_LOGV("miiot_uart", "recv: data=[%s] len=%i", this->recv_buf_.data(), this->recv_buf_.capacity());
            
            // Tokenize the command string
            std::string word;
            std::vector<std::string> args;
            std::stringstream sstream(this->recv_buf_.data());
            
            std::getline(sstream, word, ' ');
            auto command = word;
            while (std::getline(sstream, word, ' '))
                args.push_back(word);

            // Send message to handler
            auto miiot_out = this->handle_miiot(command, args);
            send(miiot_out);

            // Empty recv buffer
            this->recv_buf_.erase(this->recv_buf_.begin(), this->recv_buf_.end());
        }
        else if(ch != '\n') {
            // Append char to recv buffer
            this->recv_buf_.push_back(ch);
        }
    }

    void send(std::string reply) {
        ESP_LOGV("miiot_uart", "send: data=[%s] len=%i", reply.c_str(), reply.length());
        if (reply.back() != '\r') {
            reply += '\r';
        }
        write_str(reply.c_str());
    }

protected:
    std::vector<char> recv_buf_{};
};
