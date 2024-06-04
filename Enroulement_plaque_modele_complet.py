# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

file = "L1000.inp"

R_ext_ = 203.1
#choose btw : "circonf" "longi" "thickness"
axe_X_ = "longi"
axe_Y_ = "circonf"
axe_Z_ = "thickness"


#coeff_correctif_longueur_ = longueur voulue / longueur initiale
coeff_correctif_longueur_ = 1


def transform(x, y, z):
    return 0, 0, 0

import csv
from math import *

Nodes_list_ = {}
Start_print = 0
with open(file, newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    
    for row in spamreader:
        #print(row[0])
        if (row[0][0:5] == '*Node' ) and (Start_print == 0) :
            print ("Node find on")
            Start_print = 1
            
        elif (row[0][0:1] == '*') and (Start_print == 1) :
            print ("Node find off")
            Start_print = 2
        
        elif Start_print == 1 :
            for num, val in enumerate(row) :
                
                if num == 0 : 
                    key_ = int(val)
                elif num == 1 :
                    if axe_X_ == "longi" : X_ = float(val)
                    elif axe_X_ == "circonf" : Y_ = float(val)
                    elif axe_X_ == "thickness" : Z_ = float(val)
                elif num == 2 :
                    #Y_ = float(val)
                    if axe_Y_ == "longi" : X_ = -float(val)
                    elif axe_Y_ == "circonf" : Y_ = float(val)
                    elif axe_Y_ == "thickness" : Z_ = float(val)
                elif num == 3 :
                    #Z_ = float(val)
                    if axe_Z_ == "longi" : X_ = float(val)
                    elif axe_Z_ == "circonf" : Y_ = float(val)
                    elif axe_Z_ == "thickness" : Z_ = float(val)
                
            Nodes_list_[key_] = [X_, Y_, Z_]


X_max_ = 0
X_min_ = 0

Y_max_ = 0
Y_min_ = 0

Z_max_ = 0
Z_min_ = 0


for key in Nodes_list_ : 
    if Nodes_list_[key][0] > X_max_ : X_max_ = Nodes_list_[key][0]
    if Nodes_list_[key][0] < X_min_ : X_min_ = Nodes_list_[key][0]
    
    if abs(Nodes_list_[key][1]) > Y_max_ : Y_max_ = abs(Nodes_list_[key][1])
    if abs(Nodes_list_[key][1]) < Y_min_ : Y_min_ = abs(Nodes_list_[key][1])
    
    if Nodes_list_[key][2] > Z_max_ : Z_max_ = Nodes_list_[key][2]
    if Nodes_list_[key][2] < Z_min_ : Z_min_ = Nodes_list_[key][2]


Nodes_list_modifie_ = {}
for key in Nodes_list_ :
    #X = longi ; Y = circonf ; Z = R
    X_ = Nodes_list_[key][0] - X_min_
    Y_ = Nodes_list_[key][1] - Y_min_
    Z_ = Nodes_list_[key][2] - Z_min_
    
    
    R_ = R_ext_ - Z_max_ + Z_
    
    Z_modif_ = R_ * cos((1 - (abs(Y_) / (Y_max_-Y_min_))) * (pi/2))
    Y_modif_ = R_ * sin((1 - (abs(Y_) / (Y_max_-Y_min_))) * (pi/2))
    X_modif_ = X_ * coeff_correctif_longueur_
    
    Nodes_list_modifie_[key] = [X_modif_, Y_modif_, Z_modif_]


new_file = open(file[:-4] + '_modif' + file[-4:], "w")
old_file = open(file)

Start_print = 0
Print_line_ = "on"
for line in old_file :
    if (line[0:5] == '*Node') and (Start_print==0) :
        Start_print = 1
        Print_line_ = "off"
        new_file.write(line)
    elif (line[0:8] == '*Element') and (Start_print==0) :
        Start_print = 2
        Print_line_ = "on"
    elif Start_print == 1 :
        for i in Nodes_list_modifie_ :
            X_ = Nodes_list_modifie_[i][0]
            Y_ = Nodes_list_modifie_[i][1]
            Z_ = Nodes_list_modifie_[i][2]
            new_file.write("  " + str(i) + ",  " + str(X_) + ",  " + str(Y_) + ",  " + str(Z_) + "\n")
            Start_print = 0
         
    if Print_line_ == "on" :
        new_file.write(line)

new_file.close()
old_file.close()


print(X_max_)


transform(1,1,1)