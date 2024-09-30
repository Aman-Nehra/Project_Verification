// kmp.h
#ifndef KMP_H
#define KMP_H

#include <string>

// Function declarations
void computeKMPTable(std::string pattern, int* lps);
int KMPSearch(std::string pattern, std::string text);

#endif