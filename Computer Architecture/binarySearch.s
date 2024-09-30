.data
arr:    .word 2, 3, 4, 10, 14, 19, 25, 30   
len:    .word 8                             
Val: .word 14                            # Val element to search
result: .word 0                             # result
Found:   .asciiz "Element found at index "
notFound: .asciiz "Element is not present in the array."

.text


binarySearch:
    addi $sp, $sp, -20                  
    sw $ra, 16($sp)                    
    sw $a0, 0($sp)          
    sw $a1, 4($sp)             
    sw $a2, 8($sp)           
    sw $a3, 12($sp)           

    bgt $a2, $a3, base_case             # If left > right

    add $t0, $a2, $a3                   # mid = left + right
    srl $t0, $t0, 1                     # mid = (left + right) / 2

    
    sll $t1, $t0, 2                     
    move $t3, $a0                       
    add $t1, $t3, $t1                   # address of arr[mid]
    lw $t2, 0($t1)                      # Load arr[mid]

  
    beq $t2, $a1, found          

    # Check if arr[mid] > Val
    bgt $t2, $a1, first_half           # If arr[mid] > Val, search in left half

    # Else search in right half
    addi $a2, $t0, 1                    # left = mid + 1
    jal binarySearch                    # Recursive call for the right half
    j search_end                        # Proceed to end

first_half:
    addi $a3, $t0, -1                   # right = mid - 1
    jal binarySearch                    # Recursive call for the left half
    j search_end                        # Proceed to end

found:
    move $v0, $t0                       # Return mid
    j search_end

base_case:
    li $v0, -1                          # Element not found, return -1

search_end:
    lw $ra, 16($sp)                   
    addi $sp, $sp, 20                 
    jr $ra                      
main:
    lw $t0, len                        
    li $t1, 0                          
    addi $t2, $t0, -1             
    la $a0, arr                        
    lw $a1, Val               
    move $a2, $t1    
    move $a3, $t2           
    jal binarySearch                   

    sw $v0, result                     

    # Check if the element is found
    li $t5, -1
    beq $v0, $t5, not_found             # If result is -1, element not found

    li $v0, 4                           
    la $a0, Found
    syscall

    li $v0, 1                           
    lw $a0, result
    syscall
    j end                               # Jump to end

not_found:
    li $v0, 4                         
    la $a0, notFound
    syscall
    j end

end:
    li $v0, 10                          # syscall: exit
    syscall
