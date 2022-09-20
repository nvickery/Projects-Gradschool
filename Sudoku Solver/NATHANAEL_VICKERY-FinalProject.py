#   Final Project - March 5, 2021
#   Author: <NATHANAEL> <VICKERY> 

#   **********************************************

#   ----------------------------------------------
#   Sudoku Solver
#   ----------------------------------------------

# Method of Approach: The first step is to import the desired puzzle.
# The puzzle will then be solved using a backtracking algorithm
# (described in more detail in the fucntion description of "bctrck()").
# Depending on the user's desire, the results can be printed or hints
# can be given. Finally, the solution to the puzzle can be exported
# if desired. 

import numpy as np                      # Used to make lists readable
import copy                             # Used to make copy of lists
import random                           # Used to generate hints

# The function "initial()" will open a '.csv' file and transfer the
# necessary data into a list for the creation of the Puzzle.
def initial(filename):
    origin = []
    sol = []
    for line in f:                  # Opens .csv file and imports Puzzle
        res = line.split(';')
        for i in range(0,9):
            res[i] = res[i].rstrip('\n')
            res[i] = list(map(int,res[i]))
        origin.append(res)
    f.close()                       
    for i in range(0,9):            # Cleans up the look of the Puzzle
        for j in range(0,9):
            if origin[i][j] == []:
                origin[i][j] = 0
            else:
                origin[i][j] = origin[i][j][0]
    return origin

# The function "check()" verifies whether a number can be placed
# in a row, column, or box as per Sudoku rules.
def check(row,col,num):
    for i in range(0,9):            # row check
        if solution[i][col] == num:
            return False
    for i in range(0,9):            # column check
        if solution[row][i] == num:
            return False
    Rrem = (row//3)*3               # box check
    Crem = (col//3)*3
    for i in range(0,3):            
        for j in range(0,3):
            if solution[i+Rrem][j+Crem] == num:
                return False
    return True

# The function "bctrck()" uses recursion to fill out the Sudoku
# with possibilites. If it runs into a dead end, it will set the
# previously placed value with zero and run again. The process is
# repeated until the solution is found and saved globally. 
def bcktrck(): 
    for i in range(0,9):
        for j in range(0,9):
            if solution[i][j] == 0:         # Checks if any elements are blank
                for k in range(1,10):
                    if check(i,j,k) == True:
                        solution[i][j] = k  # Places possible number in element
                        bcktrck()           # Calls itself to place next element
                        solution[i][j] = 0  # Sets element to zero if incorrect
                return                      # returns to prev. recursion and tries new number
    global Solved                   # Creates 'Solved' outside of the bctrck()
    Solved = copy.deepcopy(solution) # Save a copy of the Solution to global program
    return

# The function "Solute()" prints the solution in a readable manner.
def getSol(sol): 
    print('The solution is:')
    print(np.array(sol,dtype=object)) 

# The function "Origin()" prints the original puzzle in a readable
# manner.    
def getOrg(original): 
    print('The original puzzle is:')
    print(np.array(original,dtype=object))

# The function "export()" will export the solution into a '.csv' file
def export(fname,sol):
    StrSol = str(sol[0][0:9])+'\n'          # Converts solution into string
    for i in range(1,9):
        StrSol += str(sol[i][0:9])+'\n'
    name = str(fname)+'_'+'Solution'+'.csv'
    s = open(name,'w')                      # Writes and saves solution in '.csv' file
    s.write(StrSol)
    s.close()
    print('Solution has been exported as ',name)
    
#The function "hint()" will return a list of hints to the puzzle using seed.
def hint(SolvPuzzle,original):
    random.seed(1234)
    hint = 'The hints are indexed from the upper right hand side of the puzzle:\n'
    for n in range(0,5):                    # Collects at most 5 hints. 
        i = random.randrange(0,9)
        j = random.randrange(0,9)
        if SolvPuzzle[i][j] != original[i][j]:
            hint += 'Element ['+str(i+1)+']['+str(j+1)+'] contains a '+str(SolvPuzzle[i][j])+'\n'
    print(hint)
    

# Main Code of Sudoku Puzzle Solver Program
Pass = False
while Pass == False:                    # While loop to control user input.
    try:
        filename = input('Please enter the name of your Puzzle: \n')
        f = open(str(filename) + '.csv','r')
        original = initial(f)               # Initiate Puzzle List
        solution = copy.deepcopy(original)  # Saved a copy for use in bctrck() function
        Pass = True
    except FileNotFoundError:
        print('Could not find file. When entering filename, do not include ".csv"')
getOrg(original)                        # Print Puzzle for convenience
Pass = False
while Pass == False:                    # While loop to control user input.
    decision = input('Type "Solution" to view solution or "Hints" for a list of hint \n')
    if decision == 'Solution':
        bcktrck()                       # Solves Puzzle
        getSol(Solved)                  # Prints Solution
        Pass = True
    elif decision == 'Hints':
        bcktrck()                       # Solves Puzzle
        hint(Solved,original)           # Prints hints
        Pass = True
    else:
        print(decision, 'is not an option. ')
Pass = False
while Pass == False:                    # While loop to control user input.
    decision = input('Would you like to export the Solution? "Yes" or "No" \n')
    if decision == 'Yes':
        export(filename,Solved)         # Exports Solution to '.csv' file
        Pass = True
    elif decision == 'No':
        print('Alrighty then! Goodbye!')
        Pass = True
    else:
        print(decision, 'is not an option. ')



# END
