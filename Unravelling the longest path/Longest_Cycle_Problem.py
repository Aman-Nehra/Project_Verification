import numpy as np 
import matplotlib.pyplot as plt
from pyomo.environ import *

# Creating a model
model = ConcreteModel()

model.n = 44  # Number of vertices

# Define vertices
model.V = Set(initialize=range(1,model.n+1))  # Vertices set

# Edges set
model.E = Set(initialize=[(2,4) , (4,2) , (4,5) , (5,4) ,(5,6) , (6,5) ,(2,8), (8,2) , (6,7) , (7,6)  , (7,8) , (8,7) , (4,16) , (16,4) , (5,17), (17,5) , (6,17) , (17,6) , (2,9) , (9,2) , (16,9) , (9,16) , (17,11) , (11,17) , (9,10) , (10,9) , (10,11) , (11,10) , (10,15) , (15,10) , (11,12) , (12,11) , (12,13) , (13,12) , (13,14) , (14,13) , (14,15) , (15,14) , (15,43) , (43,15) , (14,43) , (43,14), (36,12) , (12,36) , (18,36) , (36,18) , (36,38) , (38,36) , (18,35) , (35,18) , (35,34) , (34,35) , (34,38) , (38,34) , (37,35) , (35,37) , (37,33) , (33,37) , (34,33) , (33,34) , (37,25) , (25,37), (33,32) ,(32,33) ,  (25,32) , (32,25) , (32,39) , (39,32) , (39,40) , (40,39) , (40,41) , (41,40) , (41,42) ,(42,41) ,(40,42) , (42,40) , (42,43), (43,42) , (43,22) , (22,43) , (44,22) , (22,44) , (41,44) , (44,41) , (44,3), (3,44) , (3,1) , (1,3) , (1,39) , (39,1) , (1,30) , (30,1) , (3,29) , (29,3) , (29,30) , (30,29) , (30,31) , (31,30) , (31,32) , (32,31) , (31,26) , (26,31) , (29,28) , (28,29) , (27,28) , (28,27) , (26,27) , (27,26) , (26,24) , (24,26) , (28,21) , (21,28) , (21,20) , (21,27) , (27,21) , (20,24) , (24,20) , (24,23) , (23,24) , (20,19) , (19,20) , (19,23) , (23,19) , (19,8) , (8,19) , (23,25) , (25,23),(7,18) , (18,7) , (20,21)])
                                                 

# Edge weights as a dictionary
model.w = Param(model.E, initialize={(2,4):78 , (4,2):78 , (4,5):120 , (5,4):120 ,(5,6):49 , (6,5):49 , (2,8):728 , (8,2):728 , (6,7):50 , (7,6):50  , (7,8):70 , (8,7):70 , (4,16):150 , (16,4):150 , (5,17):130, (17,5):130 , (6,17):247 , (17,6):247 , (2,9):650 , (9,2):650 , (16,9):250 , (9,16):250 , (17,11):300 , (11,17):300 , (9,10):140 , (10,9):140 , (10,11):84 , (11,10):84 , (10,15):270 , (15,10):270 , (11,12):84 , (12,11):84 , (12,13):130 , (13,12):130 , (13,14):300 , (14,13):300 , (14,15):150 , (15,14):150 , (15,43):400 , (43,15):400 , (14,43):250 , (43,14):250, (36,12):260 , (12,36):260 , (18,36):210 , (36,18):210 , (36,38):240, (38,36): 240, (18,35):130 , (35,18):130 , (35,34):36 , (34,35):36 , (34,38):75 , (38,34):75 , (37,35):195 , (35,37):195 , (37,33):55 , (33,37):55 , (34,33):114 , (33,34):114 , (37,25):60 , (25,37):60, (33,32):65 ,(32,33):65 ,  (25,32):55 , (32,25):55 , (32,39):97 , (39,32):97 , (39,40):53 , (40,39):53 , (40,41):40 , (41,40):40 , (41,42):51 ,(42,41):51 ,(40,42): 150, (42,40):150 , (42,43):150, (43,42):130 , (43,22):130 , (22,43):130 , (44,22):52 , (22,44):52 , (41,44):240 , (44,41):240 , (44,3):260, (3,44):260 , (3,1):160 , (1,3):160 , (1,39):160 , (39,1):160 , (1,30):110 , (30,1):110 , (3,29):140 , (29,3):140 , (29,30):270 , (30,29):270 , (30,31):17 , (31,30):17 , (31,32):150 , (32,31):150 , (31,26):140 , (26,31):140 , (29,28):75 , (28,29):75 , (27,28):165 , (28,27):165 , (26,27):160 , (27,26):160 , (26,24):56 , (24,26):56 , (28,21):620 , (21,28):620 , (21,20):160 , (20,21) :160 , (21,27):77 , (27,21):77 , (20,24):15 , (24,20):15 , (24,23):60 , (23,24):60 , (20,19):160 , (19,20):160 , (19,23):50 , (23,19):50 , (19,8):700 , (8,19):700 , (23,25):140 , (25,23):140,(7,18):100 , (18,7):100})  
# (8,19) , (36,38)
# Decision variables
model.x = Var(model.E, domain=Binary)  # Whether an edge is selected xij
model.y = Var(model.V, domain=Binary)  # Whether a vertex is selected yi , yj

# Objective function: maximize the sum of selected edge weights
model.obj = Objective(expr=sum(model.w[e] * model.x[e] for e in model.E), sense=maximize)

# Constraints: Each edge can only be selected in one direction for undirected graph
model.cover = ConstraintList()
for i,j in model.E:
    if (i,j) in model.E and (j,i) in model.E :
        model.cover.add(expr=model.x[(i,j)] + model.x[(j,i)] <= 1.1)

# Degree constraints
for i in model.V:
    model.cover.add(expr=sum(model.x[(i, j)] + model.x[(j, i)] for j in model.V if (i, j) in model.E or (j, i) in model.E) == 2*model.y[i])

for i, j in model.E:
    model.cover.add(expr=model.y[i] >= model.x[(i, j)])  # If edge (i, j) is selected, then y_i must be 1
    model.cover.add(expr=model.y[j] >= model.x[(i, j)])  # If edge (i, j) is selected, then y_i must be 1

# Subtour elimination constraints (MTZ)
model.u = Var(model.V, domain=NonNegativeIntegers, bounds=(1, model.n))
for e in model.E:
    model.cover.add(expr=model.u[e[0]] - model.u[e[1]] + model.n * model.x[e] <= model.n - 1)

# Invoking the solver
solver = SolverFactory('cbc')  # Assuming CBC is installed and accessible
results = solver.solve(model, tee=True)


selected_edges = []
for edge in model.E:
    if model.x[edge].value >= 0.9:  # Using 0.9 to avoid floating-point errors
        selected_edges.append(edge)
        print(f"Selected edge: {edge}")



