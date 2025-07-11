#ifndef PCA9685_HW_INTERFACE_UTILS_HPP_
#define PCA9685_HW_INTERFACE_UTILS_HPP_

#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <iostream>


template <typename T>
std::vector<T> parse_list(const std::string& input_str) {
    std::vector<T> output_list;

    std::cout << "[parse_list] Parsing input string: " << input_str << std::endl;

    // Remove square brackets if present
    std::string clean_str = input_str;
    if (!clean_str.empty() && clean_str.front() == '[') {
        clean_str.erase(0, 1);  // Remove '['
    }
    if (!clean_str.empty() && clean_str.back() == ']') {
        clean_str.pop_back();   // Remove ']'
    }

    std::istringstream iss(clean_str);
    std::string token;

    while (std::getline(iss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t\n\r"));
        token.erase(token.find_last_not_of(" \t\n\r") + 1);

        try {
            std::istringstream converter(token);
            T value;
            converter >> value;

            if (converter.fail()) {
                throw std::invalid_argument("Conversion failed");
            }

            output_list.push_back(value);
        }
        catch (const std::invalid_argument&) {
            // Replace with ROS logging if needed
            std::cerr << "Invalid value: " << token << std::endl;
            return {};
        }
    }

    return output_list;
};


#endif