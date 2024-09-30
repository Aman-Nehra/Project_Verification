#include "trie.h"
#include "kmp.h"
#include <iostream>
#include<fstream>

Trie::Trie()
{
	root = NULL;
}

// Returns new trie node (initialized to NULLs)
struct TrieNode* Trie::getNode(void)
{
	TrieNode * newNode = new TrieNode ; 
	for (unsigned i = 0 ; i<ALPHABET_SIZE ; i++) (newNode->children)[i]=nullptr ;  // all children initialised to nullptr
	return newNode ; 
}

// If not present, inserts key into trie. If the
// key is prefix of trie node, just marks leaf node
void Trie::insert(struct TrieNode* & root, const std::string key)
{

	// This whole is root points to null
	if(root == NULL){
		// If this the first insert that means root is absent 
		root = Trie::getNode() ;
	}
	else {
		// Finding if present or not
		TrieNode * temp = root ;
		for(int i=0 ;i<key.size() ; i++) {	
			if(temp->children[CHAR_TO_INDEX(key[i])] == nullptr) {
				// Basically this is not matched match for other key
				// If in this part means that key is not insterted and we need to brack out from i to insert this key
				temp->children[CHAR_TO_INDEX(key[i])] = Trie::getNode();
			}
			temp = temp->children[CHAR_TO_INDEX(key[i])] ; 
			if(i==key.size()-1) temp->isWordEnd = true ;
		}
	}
}

bool Trie::search(struct TrieNode *root, std::string key)
{
	TrieNode * temp = root ;	
	for(int i=0 ;i<key.size() ; i++) {
		if(temp->children[CHAR_TO_INDEX(key[i])] == nullptr) return false ;
		else temp = temp->children[CHAR_TO_INDEX(key[i])] ;
	}
	return true ;
}

// Returns 0 if current node has a child
// If all children are NULL, return 1.
bool Trie::isLastNode(struct TrieNode* root)
{
	if(root == NULL) return 1 ;
	for(int i = 0 ; i<ALPHABET_SIZE ; i++) if(root->children[i] != nullptr) return 0 ;
	return 1 ; 
}

// Recursive function to print auto-suggestions for given
// node.
void Trie::suggestionsRec(struct TrieNode* root,std::string currPrefix , std::string* results , int& idx)
{
	// found a std::string in Trie with the given prefix
	TrieNode * temp = root ;
	for(int i = 0 ; i<currPrefix.size() ; i++) temp = temp->children[CHAR_TO_INDEX(currPrefix[i])] ;
	// Now temp arrived at the last node of the currprefix now we need to return all the possible values , basically all children from this node 
	for(int i = 0; i<ALPHABET_SIZE ; i++) {
		if(temp->children[i] != nullptr) {
			std::string To_return = currPrefix+(char(i+int('a'))) ; 
			// std::cout << To_return << std::endl ;
			results[idx] = To_return ; 
			idx++ ;
			suggestionsRec(root,To_return,results,idx) ;
		}
	}
}

// print suggestions for given query prefix.
int Trie::printAutoSuggestions(TrieNode* root, const std::string query)
{

	// Now pass the query to the proces Context such that it makes the trie of all the relevant words .
	std::string word = Trie::processContext(query); // After this trie is maded 

	// Precceding conatins the word just before the word to be autocompleted
	std::string precceding = "" ;
	std::string currPrefix = "" ;

	// This stores all the words obtained by completing currPrefix
	std::string results[1000] ;
	int idx = 0;
	bool space_encountered = 0 ;

	for(int i = 0 ; i<word.size() ; i++){
		if(word[i] == ' ') {
			space_encountered = 1 ;
			continue;
		}
		if(space_encountered) currPrefix += word[i] ;
		else precceding += word[i] ;
	}

	// Now precedding stores the word precedding .
	// Now we have to match the pattern of word with our line and count the occurences of each of them 

	suggestionsRec(root,currPrefix,results,idx) ;
	int arr[1000] ; // To store occurences
	int Arr_preceding_Absent[1000] ; // To store occurences

	std::ifstream lorem("lorem.txt") ;

	int max_occurence = 0 ;
	int max_occurence_preceding_absent = 0 ;

	std::string line ;
	while(getline(lorem,line)){
		std::string pattern ;
		for(int j = 0  ;j<idx ; j++){
			// std::cout << results[j] << std::endl ; 
			pattern = precceding+" "+results[j]+" ";
			arr[j] = KMPSearch(pattern,line) ;
			pattern = results[j] ;
			Arr_preceding_Absent[j] = KMPSearch(pattern,line) ;
			// std::cout << arr[j] << std::endl ;
			max_occurence = std::max(max_occurence,arr[j]);
			max_occurence_preceding_absent = std::max(max_occurence_preceding_absent,Arr_preceding_Absent[j]);
		}
	}

	int answer = 0 ;

	for(int j = 0  ;j<idx ; j++){
		if(max_occurence != 0){
			if(arr[j] == max_occurence){
				std::cout << results[j] << "\n" ;
				if(max_occurence == 1){
					if(results[j] == currPrefix) {
						answer = -1;
					} 
				} 
			}
		}
		else{
			if(Arr_preceding_Absent[j] == max_occurence_preceding_absent){
				std::cout << results[j] << "\n" ;
				if(max_occurence_preceding_absent == 1){
					if(results[j] == currPrefix) {
						answer = -1;
					} 
				} 
			}
		}
	}

	// Sorting the array based om the 

	// Now Finding the relevant match 

	if(answer == -1) return -1 ;
	if(max_occurence == 0) {
		if(max_occurence_preceding_absent > 0) return 1 ;
		return 0 ;
	}
	else return 1 ; // dummy return
}

// Process the file "lorem.txt" to insert the words in lorem.txt and store the relevant context as needed.
std::string Trie::processContext(const std::string query)
{
	std::ifstream txt_data("lorem.txt"); // Opening the lorem.txt to find all the relevant words 

	// Error handling is lorem is absent
	if(!txt_data) {
		std::cerr << "Error opening the file" << std::endl ;
	}

	// supoose I make trie of all the words that has the same context and then print suggestion on them in increasing order :)
	// If i have n words then I have n-1 spaces 
	// access the words after the (n-2)th spaces 

	std::string ctx_word = "" ; // context word
	std::string word = "" ; // word
	int space_count = 0 ;

	// Now iterating to get least two words to the string
	for(int i = query.size() -1; i>=0 ; i--){
		if(query[i] == ' ') {
			space_count ++ ;
			if(space_count == 2) break ;
			if(space_count == 1) {
				word = ctx_word ;
				ctx_word.clear() ;
				continue ;
			}
		}
		ctx_word = query[i] + ctx_word ; 
	}

	// Now we need to make search context word in the text using a while loop
	std::string lorem_word ;		
	bool addNextWordTrie = 0 ;
	while (txt_data>>lorem_word)
	{
		if(addNextWordTrie) {
			this->insert(this->root,lorem_word) ;
			addNextWordTrie = 0 ;
		}
		if(lorem_word == ctx_word) addNextWordTrie = 1 ;
	}
	  
	// now Trie constructed for all the relevant words .

	int word_len = word.size();
	word = word.substr(0,word_len-2) ; // removing the #
	word = ctx_word + " " + word ;
	return word ; //Returning the preifx

}
