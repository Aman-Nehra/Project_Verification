#include<iostream>
#include "trie.h"

int main()
{
	// Input keys (use only 'a' through 'z'
	// and lower case)
    Trie* tr = new Trie();
	tr->root = tr->getNode();

    // Construct Trie
    // tr->processContext();

    // Search for different keys
    char output[][32] = {"Not present in trie", "Present in trie"};


    std::string line ;
    std::getline(std::cin, line);
    int comp = tr->printAutoSuggestions(tr->root, line); // line contains the query so basically this will process context and then call the suggestion Rec 

    if (comp == -1) std::cout << "No other strings found with this prefix" << std::endl;
    else if (comp == 0) std::cout << "No string found with this prefix" << std::endl;
    
	return 0;
}