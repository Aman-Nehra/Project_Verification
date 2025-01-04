#include "structures.hpp"
#include<thread>
#include<mutex>
#include<queue>
#include<chrono>
#include<condition_variable>
#include<unordered_map>
#include <array>
#include <span>
#include <cmath>
// -----------------------------------------------------------------------------

// You are free to add any STL includes above this comment, below the --line--.
// DO NOT add "using namespace std;" or include any other files/libraries.
// Also DO NOT add the include "bits/stdc++.h"

// OPTIONAL: Add your helper functions and classes here


// ******************************* PLAG CHECKING CODE **********************************
// Node for a suffix tree
/*
The below class is a Node and is by the Trie data structure
----> Each node contains an unordered map, with the value being Pointer to the children and key being it's value in the vector (submission 1 here).
----> Also contains , isWordEnd to mark end word in the trie
----> Index store the index of that node in the vector 
*/
class Node {
    public : 
        std::unordered_map<int,std::shared_ptr<Node>> children;
        int index ; // To store the index of the token , that is it's position in the submission 1 in our case , used to report result 3 and result 4
        bool isWordEnd ; 
        Node() ; // Constructor
};

/*
Contains a constructor and a destructor to avoid memory overflow 
---> Contains root , Insert method and a buildsuffixTree that passes all the suffix to Insert method to make a suffix trie of a vector
 */
class SuffixTree {
    public:
        std::shared_ptr<Node> root ; 
        SuffixTree();
        std::shared_ptr<Node>  getNode() ; 
        void Insert(std::vector<int> & keys, int n) ; 
        void buildsuffixTree(std::vector<int> & pattern) ;

};

// Declaration here , defination below 
void fix_overlapping(std::vector<bool> & matched , std::vector<int> & match_index , int & curr_match_len , int & overlap_start , int & overlap_end ,std::shared_ptr<Node> temp, int  j) ; 

void add_to_match(std::vector<bool> & matched , std::vector<int> & match_index , int & curr_match_len,int  j,std::shared_ptr<Node> temp) ; 

void ShortMatching(std::shared_ptr<Node> root, std::vector<int> & sub2 , std::vector<int> & match_index,int & idx,std::vector<bool> & matched,int & numPat_matched, bool & LongMatch) ; 

std::pair<int, int> match_sub(std::shared_ptr<Node> root ,const int & sub1_size, std::vector<int> &submission2) ; 

// Function of this class : 
// 1. Intialises with a set of past submissions for the plagarism checking 
// 2. Method : add-submission => 
//            -> Accepts a pointer to a new submission and processes it in paralleln using multithreading 

// When to raise a flag : 
// 1. If length >= 75 tokens exist between the submission
// 2. If >= 10 patterns matches are found 
// Other conditions : 
//                  -> If time difference >= 1 seconds , only later submission is flagged 
//                  -> If time difference <1 seconds , both the submissions are flagged 


// ****************************************Patchwork Plag*************************************
// Flag a submission if >=20 distinct matches are found across multiple previous submissions . 



class plagiarism_checker_t {
    // You should NOT modify the public interface of this class.
public:
    plagiarism_checker_t(void);//constructor //when past submissions are not given(not to be matched for plag with past submissions)
    plagiarism_checker_t(std::vector<std::shared_ptr<submission_t>> 
                            __submissions); //constructor when previous submissions are given
    ~plagiarism_checker_t(void); //destructor
    void add_submission(std::shared_ptr<submission_t> __submission);  //for adding submission for checking plag

protected:
    // TODO: Add members and function signatures here

    std::unordered_map<int,std::shared_ptr<submission_t>> submission_map ; // Map between submission id and pointer 
    std::unordered_map<int, std::pair<std::vector<int>, std::chrono::milliseconds>> past_submissions; // Map between submission id and tokens,timestamp
    std::unordered_map<int,std::shared_ptr<Node>> past_sub_suffix; // Map stores the root of suffix tree of each submission (key = id)
    
    std::queue<std::pair<int,std::chrono::milliseconds>> taskQueue ; // This queue stores the id of the task to be processed

    std::mutex queue_mutex ; // This mutex ensures that their is not data race in the task queue itself 
    std::mutex past_sub_suffix_mutex ; // A mutex for the suffix tree of the past submissions 
    
    std::condition_variable condition ; // This is for synchronisation of the addsubmission with the plag checker thread during in the background 

    std::thread plag_checker ; //  This thread has a functionality to check the plag of the submissions in the queue

    bool stop_thread ; // This is a flag to stop the thread when the destructor is called

    std::unordered_map<int,bool> isPlagged ; // This vector stores the flag of the submission , if it is plagged or not

    // Making helper functions 
    void process_tasks() ; 
    void check_for_plag(std::shared_ptr<submission_t> submission, std::vector<int> & tokens, std::chrono::milliseconds timeStamp) ; 
    void make_suffix_tree(int sub_id , std::vector<int>  tokens) ;
    // End TODO
};
