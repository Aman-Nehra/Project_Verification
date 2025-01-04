#include "plagiarism_checker.hpp"
// You should NOT add ANY other includes to this file.
// Do NOT add "using namespace std;".

// ******************************* PLAG CHECKING CODE **********************************

Node::Node()  {isWordEnd = false ; index = -1 ;} ; // Constructor

SuffixTree::SuffixTree() {
    root = getNode(); // Initialize root here
}
        
std::shared_ptr<Node> SuffixTree::getNode(){
    return std::make_shared<Node>();
}

// n is the size of the vector whose suffix trie is being build
void SuffixTree::Insert(std::vector<int> & keys,  int n){
    auto temp = root ;
    int curr_suffix_size = keys.size() ; 

    for(int i=0 ;i<keys.size() ; i++) {	
        // If node for the key not found, then insert one for it.
        if(temp->children.find(keys[i]) == temp->children.end()) {
            temp->children[keys[i]] = SuffixTree::getNode();
        }

        // Store the first occurence in the suffix tree only 
        if (temp->children[keys[i]]->index == -1) 
            temp->children[keys[i]]->index = n-curr_suffix_size+i ; // Storing the index of the token in the submission 1
        
        temp = temp->children[keys[i]] ; 
        if(i==keys.size()-1){
            temp->isWordEnd = true ;
        }

    }
}

void SuffixTree::buildsuffixTree(std::vector<int> & pattern){
    for(int i = 1 ; i<=pattern.size() ; i++){
        std::vector<int> sliced_vec(pattern.end()-i,pattern.end()) ; 
        Insert(sliced_vec,pattern.size()) ; 
    } 
}


// Declaration here , defination below 
void fix_overlapping(std::vector<bool> & matched , std::vector<int> & match_index , int & curr_match_len , int & overlap_start , int & overlap_end ,Node * temp, int  j) ; 

/*
-----> matched : keeps check of which tokens in the submission1 (in our case) is matched
-----> match_index : keeps the track that the tokens in the sub2 is matched to which index token in sub1
-----> curr match len : keeps track of the length of the token that has been matched  
-----> j : is the index in the sub2 where the match ended
-----> temp : pointer to the last matched index wrt submission 1 (onw whose trie is build) in the TRIE 
*/
void add_to_match(std::vector<bool> & matched , std::vector<int> & match_index , int & curr_match_len,int  j,std::shared_ptr<Node> temp){
    int overlap_start = -1 ; 
    int overlap_end = -1;
    for (int i = 0; i<curr_match_len ; i++){
        if(matched[temp->index-i] == false){
            match_index[j-i] = temp->index-i ;
            matched[temp->index-i] = true ;
        }
        else {
            if(overlap_start == -1){
                overlap_start = i ;  
            }
            overlap_end = i ;
        }
    }

    // Calling the function to check in which region overlapping is there 
    if(overlap_start != -1){
        // Overlapping is present
        fix_overlapping(matched , match_index , curr_match_len , overlap_start , overlap_end , temp , j) ; 
    }

    return ; 
}


/*
Same arguements almost
-----> overlap start : denotes the index from where overlap starts  
-----> overlap end : denotes the index where overlap ends 
***** -> Overlap means that if there is match between sub1 and sub2 currently , and that region of the sub1 was already matched
         with some other part of sub2, then I declare a overlap
*/
void fix_overlapping(std::vector<bool> & matched , std::vector<int> & match_index , int & curr_match_len , int & overlap_start , int & overlap_end ,std::shared_ptr<Node> temp, int  j){

    // If overlap is started at a point and it ends before the last index of current matched pattern then the current matched pattern is longer 
    // than the previous pattern so remove that and write the current one 
    if(overlap_end < curr_match_len -1 ){
        for(int i = overlap_start ; i<= overlap_end ; i++){
            matched[temp->index-i] = true ; 
            match_index[j-i] = temp->index - i ; 
        }
    }
    else{
        if(overlap_start <= 14) {
            // In this case most of this sub sequence is already matched with someone else so drop this and continue 
            // So change the matched variable for these <=9 tokens to false
            for(int i = 0 ; i<overlap_start ; i++) {
                match_index[j-i] = -1 ;
                matched[temp->index-i] = false ;
            }
            return ; 
        } 
    }
    return ;    
}

/*
This matches the sub2 and sub1 using the suffix trie of sub1 
*/
void ShortMatching(std::shared_ptr<Node> root, std::vector<int> & sub2 , std::vector<int> & match_index,int & idx,std::vector<bool> & matched,int &  numPat_matched, bool & LongMatch)
{
    auto temp = root ; 
    int curr_match_len = 0 ;
    for(int j = idx ; j<sub2.size() ; j++){
        if(temp->children.find(sub2[j]) != temp->children.end()){ 
            temp = temp->children[sub2[j]] ; 
            curr_match_len ++ ;
            // As soon as we find a match of length 15, then we can increament the count of the matched tokens
            if(curr_match_len >= 75){
                LongMatch = 1 ; 
            }

            if(temp->isWordEnd == 1){  
                // Case when the token is matched and suffix's end is reached 
                if(curr_match_len >= 15){
                    numPat_matched += curr_match_len/15 ; 
                    add_to_match(matched,match_index,curr_match_len,j,temp) ; 
                    idx += curr_match_len;
                    return ; 
                    // No need to go to suffix link as we have already matched the token at the word end
                }
                else {
                    idx ++ ; 
                    return ; 
                }
            }
            if(j == sub2.size()-1) {
                idx = sub2.size() ; 
            }
        }

        else if(curr_match_len >= 15){
            if(curr_match_len >= 75){
                LongMatch = 1 ; 
            }
            numPat_matched += curr_match_len/15 ; 
            j = j-1 ; // Beacuse in last loop when temp was intialised j was incremented by one after that due to loop 
            add_to_match(matched,match_index,curr_match_len,j,temp) ; 
            idx += curr_match_len;
            return ; 
        }
        else{
            idx ++ ; 
            return ; 
        }
    }
}

void ShortMatchingModified(std::shared_ptr<Node> root, std::vector<int> & sub2 , std::vector<int> & match_index,int & idx,std::vector<bool> & matched,int &  numPat_matched, bool & LongMatch)
{
    auto temp = root ; 
    int curr_match_len = 0 ;
    for(int j = idx ; j<sub2.size() ; j++){
    
        if(match_index[j] != -1){
            if(temp->children.find(sub2[j]) != temp->children.end()){ 
                temp = temp->children[sub2[j]] ; 
                curr_match_len ++ ;
                // As soon as we find a match of length 15, then we can increament the count of the matched tokens
                if(curr_match_len >= 75){
                    LongMatch = 1 ; 
                }

                if(temp->isWordEnd == 1){  
                    // Case when the token is matched and suffix's end is reached 
                    if(curr_match_len >= 15){
                        numPat_matched += curr_match_len/15 ; 
                        add_to_match(matched,match_index,curr_match_len,j,temp) ; 
                        idx += curr_match_len;
                        return ; 
                        // No need to go to suffix link as we have already matched the token at the word end
                    }
                    else {
                        idx ++ ; 
                        return ; 
                    }
                }
                if(j == sub2.size()-1) {
                    idx = sub2.size() ; 
                }
            }

            else if(curr_match_len >= 15){
                if(curr_match_len >= 75){
                    LongMatch = 1 ; 
                }
                numPat_matched += curr_match_len/15 ; 
                j = j-1 ; // Beacuse in last loop when temp was intialised j was incremented by one after that due to loop 
                add_to_match(matched,match_index,curr_match_len,j,temp) ; 
                idx += curr_match_len;
                return ; 
            }
            else{
                idx ++ ; 
                return ; 
            }
        }
        else {
            if(curr_match_len>=15){
                if(curr_match_len >= 75){
                    LongMatch = 1 ; 
                }
                numPat_matched += curr_match_len/15 ; 
                j = j-1 ; // Beacuse in last loop when temp was intialised j was incremented by one after that due to loop 
                add_to_match(matched,match_index,curr_match_len,j,temp) ; 
                idx += curr_match_len;
                return ; 
            }
            else{
                if(curr_match_len == 0) idx ++ ;
                else idx += curr_match_len ; 
                return ; 
            }
            if(j == sub2.size()-1) {
                idx = sub2.size() ; 
            }
        }
    }
}

std::pair<int, int> match_sub(std::shared_ptr<Node> root ,const int & sub1_size, std::vector<int> &submission2) {
    // TODO: Write your code here
    std::pair<int,int> result = {-1,0} ; // -1 denotes no long pattern match , 0 denotes the number of exact matches found till now. 
    
    int idx = 0 ; // match the submission2 from this index

    std::vector<int> match_index(submission2.size(),-1) ; // To store the index of the matched token in the submission 1
    std::vector<bool> matched(sub1_size,false) ; // To ensure no character is matched twice

    bool LongMatch = false ; 
    int numPat_matched = 0 ; 

    while (idx < submission2.size()) {
        ShortMatching(root, submission2, match_index, idx, matched, numPat_matched, LongMatch);
    }

    result = {LongMatch,numPat_matched} ; 

    return result; // dummy return
    // End TODO
}

void match_sub_patchwork(std::shared_ptr<Node> root ,int sub1_size, std::vector<int> &submission2, std::vector<int> & match_index, int & numPat_matched) {
    int idx = 0 ; 
    bool LongMatch = 0  ;
    std::vector<bool> matched(sub1_size,false) ; // To ensure no character is matched twice
    while (idx < submission2.size()) {
        if (match_index[idx] != -1) {
            idx++;
            continue;
        }
        ShortMatchingModified(root, submission2, match_index, idx, matched, numPat_matched,LongMatch);
    }
}


// TODO: Implement the methods of the plagiarism_checker_t class

plagiarism_checker_t::plagiarism_checker_t(std::vector<std::shared_ptr<submission_t>> __submissions) {
    
    for (auto& submission : __submissions) {
        
        // Map the given id to the submission_t* pointer
        submission_map[submission->id] = submission ; 
        
        tokenizer_t tokenizer(submission->codefile);
        auto tokens = tokenizer.get_tokens();

        past_submissions[submission->id] = std::make_pair(tokens, std::chrono::milliseconds{0});

        // Also make the suffix tree of the submission
        SuffixTree suffix_tree;
        suffix_tree.buildsuffixTree(tokens); 
        past_sub_suffix[submission->id] = suffix_tree.root ; 

        isPlagged[submission->id] = true ;
    }
    
    // Run this process in the background for plagarism detection
    stop_thread = false ; 
    plag_checker = std::thread(&plagiarism_checker_t::process_tasks, this);

}

// Default constructor 
plagiarism_checker_t::plagiarism_checker_t(){
    stop_thread = false ; 
    plag_checker = std::thread(&plagiarism_checker_t::process_tasks, this);
}

// Destructor ensures that when called all the process is ended. 
plagiarism_checker_t::~plagiarism_checker_t() {
    
    ####### std::lock_guard<std::mutex> lock(queue_mutex);
    stop_thread = true; // Signal the thread to stop
    
    condition.notify_all(); // Wake up the thread if it's waiting
    if (plag_checker.joinable()) {
        plag_checker.join(); // Wait for the thread to finish
    }

}

void plagiarism_checker_t::add_submission(std::shared_ptr<submission_t> __submission) {

     // Measuring the time as soon as file arrives 
    auto now = std::chrono::system_clock::now() ;
    auto timeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) ; 

    queue_mutex.lock();
    submission_map[__submission->id] = __submission ;
    taskQueue.push(std::make_pair(__submission->id,timeStamp));
    queue_mutex.unlock();

    // Notify the plagiarism checker thread
    condition.notify_one();
    return ; // dummy 
}


void plagiarism_checker_t::process_tasks() {

    while (true) {
        // Wait for tasks or stop signal
        std::unique_lock<std::mutex> lock(queue_mutex);
        ##### condition.wait(lock, [this]() { return !taskQueue.empty() || stop_thread; });
        if (stop_thread && taskQueue.empty()) break; // Stop if no more tasks
        auto [task_id,timeStamp] = taskQueue.front();
        auto submission = submission_map[task_id];
        taskQueue.pop();
        queue_mutex.unlock(); // Unlock the mutex before processing the task, beacuse I have already taken the task out of the queue
        
        // Now tokenizing the given file
        tokenizer_t tokenizer(submission->codefile);
        auto tokens = tokenizer.get_tokens();

        isPlagged[task_id] = false ;
        past_submissions[task_id] = std::make_pair(tokens,timeStamp) ;

        // Check for plagiarism
        check_for_plag(submission, tokens, timeStamp);
    }
}

void plagiarism_checker_t::make_suffix_tree(int sub_id , std::vector<int>  tokens) {
    SuffixTree suffix_tree;
    suffix_tree.buildsuffixTree(tokens);
    ###### std::lock_guard<std::mutex> lock(past_sub_suffix_mutex);
    past_sub_suffix[sub_id] = suffix_tree.root;
    return ; 
}


void plagiarism_checker_t::check_for_plag(std::shared_ptr<submission_t> submission, std::vector<int> & tokens, std::chrono::milliseconds timeStamp) {

    // Make a thread that makes a suffix Tree of the current submission parallely 
    std::thread t1(&plagiarism_checker_t::make_suffix_tree, this, static_cast<int>(submission->id), tokens);

    // Make a global vector for patchwork plagaraism 
    std::vector<int> patchwork(tokens.size(),-1) ;
    int pat_mt_glb = 0 ; // counter of number of global pattern matches
    
    // data contains both the tokens and the timestamp of the submission
    for (auto& [old_sub_id, data] : past_submissions) {

        if(old_sub_id == submission->id) continue ; // Skip the current submission
        
        auto& old_sub = submission_map[old_sub_id]; // Access submission

        // Locking ensures safe access to the suffix tree map
        ###### std::shared_ptr<Node> suf_root_oldSub;
        {
            std::lock_guard<std::mutex> lock(past_sub_suffix_mutex);
            suf_root_oldSub = past_sub_suffix[old_sub_id]; // Access suffix tree
        }

        auto& tokens_old_sub = data.first; // Access tokens


        // Make a thread that matches the submissions parallely for patchwork  
        std::thread t2(match_sub_patchwork,suf_root_oldSub,tokens_old_sub.size(),std::ref(tokens),std::ref(patchwork),std::ref(pat_mt_glb));

        // Match submissions
        auto result = match_sub(suf_root_oldSub , tokens_old_sub.size(), tokens) ;

        // If a match is found
        if (result.first == 1 || result.second >= 10) {
            auto time_diff = timeStamp - data.second; // Compare timestamps
            if (time_diff <= std::chrono::milliseconds{1000}) {
                if(!isPlagged[old_sub->id]){
                    if(old_sub->student != nullptr){
                        old_sub->student->flag_student(old_sub);
                    }
                    if(old_sub->professor != nullptr){
                        old_sub->professor->flag_professor(old_sub);
                    }
                    isPlagged[old_sub->id] = true ;
                }
            }

            if(!isPlagged[submission->id]){
                if(submission->student != nullptr){
                    submission->student->flag_student(submission);
                }
                if(submission->professor != nullptr){
                    submission->professor->flag_professor(submission);
                }
                isPlagged[submission->id] = true ;
            }

        }
        if(t2.joinable()){
            t2.join();
        }
    } 
    if(!isPlagged[submission->id] && pat_mt_glb >= 20){
        if(submission->student != nullptr){
            submission->student->flag_student(submission);
        }
        if(submission->professor != nullptr){
            submission->professor->flag_professor(submission);
        }
        isPlagged[submission->id] = true ;

    }

    // Ensure the suffix tree thread finishes before returning
    if (t1.joinable()) {
        t1.join();
    }

    return ; 
}
// End TODO