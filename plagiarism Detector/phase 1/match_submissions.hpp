#include <array>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>

// Function to check if the two sequences match approximately
bool is_fuzzy_match(const std::vector<int>& seq1, const std::vector<int>& seq2, size_t len1, size_t len2) {
    size_t match_count = 0;
    size_t max_mismatches = static_cast<size_t>(0.2 * std::max(len1, len2));

    for (size_t i = 0; i < std::min(len1, len2); ++i) {
        if (seq1[i] == seq2[i]) {
            match_count++;
        }
    }

    size_t mismatches = (len1 + len2) - 2 * match_count;
    return mismatches <= max_mismatches;
}

// Function to match submissions
std::array<int, 5> match_submissions(std::vector<int>& submission1, std::vector<int>& submission2) {
    std::array<int, 5> result = {0, 0, 0, -1, -1}; 
    int max_len = 0;
    int start1 = -1;
    int start2 = -1;
    int total_length = 0;
    int match_count = 0;
    bool long_match_found = false;
    std::unordered_map<int, bool> visited1;
    std::unordered_map<int, bool> visited2;

    for (size_t i = 0; i < submission1.size(); i++) {
        for (size_t j = 0; j < submission2.size(); j++) {
            size_t len = 0;
            while (i + len < submission1.size() && j + len < submission2.size() &&
                   submission1[i + len] == submission2[j + len] &&
                   !visited1[i + len] && !visited2[j + len]) {
                len++;
            }

            if (len >= 10) {
                for (size_t k = 0; k < len; ++k) {
                    visited1[i + k] = true;
                    visited2[j + k] = true;
                }

                if (len >= 30) {
                    long_match_found = true;
                    if (len > max_len) {
                        max_len = len;
                        start1 = static_cast<int>(i);
                        start2 = static_cast<int>(j);
                    }
                }

                match_count++;
                total_length += len;
                j += len - 1;
            }
        }
    }

    if (long_match_found || match_count >= 10) {
        result[0] = 1;
    }
    result[1] = total_length;
    result[2] = max_len;
    result[3] = start1;
    result[4] = start2;

    for (const auto& i : result) {
        std::cout << i << " ";
    }
    std::cout <<"\n";

    return result;  
}
