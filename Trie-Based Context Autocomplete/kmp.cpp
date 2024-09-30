#include <iostream>
#include <cstring>


// Fills h[] for given pattern pat[0..M-1]
void computeKMPTable(std::string pat, int* h)
{
  int len_pat = (pat).size() ;
  int i = 1 , j = 0 ;
  h[0] = -1 ;
  while(i<len_pat){
    if (pat[j] != pat[i]){
      h[i] = j ;  
      while(j>=0 && pat[j]!=pat[i]){
        j = h[j] ;
      }
    }
    else {
      h[i] = h[j] ; 
    }
    i++ ; j++ ;
  }
  h[len_pat] = j ; 
} 

// Prints occurrences of txt[] in pat[]
int KMPSearch(std::string pat, std::string txt)
{
  int result = 0; 
  int len_pat = (pat).size() , len_txt = (txt).size() ;
  if(len_pat <= 0) {
    std::cout << "This pattern matches trivially at all index !!\n" ;
    return -1 ;
  }
  if(len_txt<=0){
    std::cout << "This textis invalid\n";
    return -1 ;
  }

  int h[len_pat+1] ;
  
  computeKMPTable(pat,h) ;

  // Now finding the index of matches 
  
  int i = 0  , j = 0 ; 

  // for(int i = 0 ; i<len_pat+1 ; i++) std::cout << h[i] << " " ;
  // std::cout <<std::endl; 

  while(i<len_txt){
    if(pat[j]==txt[i]){
      i++ ; 
      j++ ;
      if(j == len_pat){
	      // printf("Found pattern at index %d \n", i-j);
        result ++ ;
        j = h[j] ; 
      }
    }
    else {
      if(h[j]<0){
        i = i+1 ;
        j = 0 ; 
      }
      else {
        j = h[j] ; 
      }
    }
  }
  return result ;
}
