#-*- coding: utf-8 -*-

from abaqus import *
from abaqusConstants import *
from odbAccess import*
from math import*
from caeModules import *
from driverUtils import executeOnCaeStartup

import csv 

def fonction_generale(seedsize,k): 
        Epaisseur = 4
        Diametre = 80
	theta = 2*pi/9.
	epss = pi/4. 
	Longueur_Tube = 150
	my_E = 210E9
	my_nu = 0.3
	my_x = (Diametre/2)-(Epaisseur/2)
	my_y = 0.0
	my_z = Longueur_Tube/2.
	mymin_inc = 0.001
	mymax_inc = 0.1
	#declaration de mon model
	Mdb()
	mdb.Model(name="Buckling_analysis")

	def create_part_3D_cylinder(radius1,radius2,lenght,part,model):
	    s = mdb.models[model].ConstrainedSketch(name='__profile__', 
	    sheetSize=200.0)
	    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
	    s.setPrimaryObject(option=STANDALONE)
	    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(0.0, radius1), point2=(0.0, -radius1), 
		direction=CLOCKWISE)
	    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(0.0, radius2), point2=(0.0, -radius2), 
		direction=CLOCKWISE)
	    s.Line(point1=(0.0, radius1), point2=(0.0, radius2))
	    s.Line(point1=(0.0, -radius1), point2=(0.0, -radius2))
	    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, 
	    type=DEFORMABLE_BODY)
	    p.BaseSolidExtrude(sketch=s, depth=lenght)
	    
	def create_part_3D_sphere(model,part_name,radius):
	    s = mdb.models[model].ConstrainedSketch(name='__profile__', 
	    sheetSize=200.0)
	    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
	    s.setPrimaryObject(option=STANDALONE)
	    s.ConstructionLine(point1=(0.0, -100.0), point2=(0.0, 100.0))
	    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius, 0.0), point2=(0.0, -radius), 
		direction=CLOCKWISE)
	    p = mdb.models[model].Part(name=part_name, dimensionality=THREE_D, 
	    type=ANALYTIC_RIGID_SURFACE)
	 
	    p.AnalyticRigidSurfRevolve(sketch=s)
	    s.unsetPrimaryObject()

	def create_material_data(model,material_name,E,nu,p11,p12,p21,p22):
	     mdb.models[model].Material(name=material_name)
	     mdb.models[model].materials[material_name].Elastic(table=((E, nu), ))
	     mdb.models[model].materials[material_name].Plastic(table=((p11, p12), 
	       (p21,p22))) 


	def Create_section(model,part_name,section_name,material_name):
	    p = mdb.models[model].parts[part_name]
	    session.viewports['Viewport: 1'].setValues(displayedObject=p)
	    mdb.models[model].HomogeneousSolidSection(name=section_name, 
	    material=material_name, thickness=None)

	def Create_Set_all_cells(model,part_name,set_name):
	    p = mdb.models[model].parts[part_name]
	    c = p.cells[:]
	    p.Set(cells= c, name=set_name)


	def assign_section(model,part_name,set_name,section_name):
	   p = mdb.models[model].parts[part_name]
	   region = p.sets[set_name]
	   p = mdb.models[model].parts[part_name]
	   p.SectionAssignment(region=region, sectionName=section_name, offset=0.0, 
	   offsetType=MIDDLE_SURFACE, offsetField='', 
	   thicknessAssignment=FROM_SECTION)

	def create_assembly(model,part_name1,part_name2,instance_name1,instance_name2):
	    a = mdb.models[model].rootAssembly
	    a.DatumCsysByDefault(CARTESIAN)
	    p = mdb.models[model].parts[part_name1]
	    a.Instance(name=instance_name1, part=p, dependent=ON)
	    p = mdb.models[model].parts[part_name2]
	    a.Instance(name=instance_name2, part=p, dependent=ON)
	    a.translate(instanceList=(instance_name2, ), vector=(0.0,(Diametre/2)+15., 0.0))

	  
	def create_analysis_step(model,step_name,min_inc,max_inc,pre_step_name):
	    mdb.models[model].StaticStep(name=step_name , previous=pre_step_name, 
	    initialInc=min_inc, maxInc=max_inc, nlgeom=ON)
	    mdb.models[model].steps[step_name].setValues(maxNumInc=10000)

	def create_interaction_property(model,interaction_property):
	    mdb.models[model].ContactProperty(interaction_property)

	    mdb.models[model].interactionProperties[interaction_property].NormalBehavior(
	    pressureOverclosure=HARD, allowSeparation=ON, 
	    constraintEnforcementMethod=DEFAULT)

	def create_interaction(model,step_name,interaction_name,interaction_property):
	   session.viewports['Viewport: 1'].assemblyDisplay.setValues(step=step_name)

	   mdb.models[model].ContactStd(name=interaction_name, createStepName=step_name)

	   mdb.models[model].interactions[interaction_name].includedPairs.setValuesInStep( stepName=step_name, useAllstar=ON)

	   mdb.models[model].interactions[interaction_name].contactPropertyAssignments.appendInStep(
	   stepName=step_name, assignments=((GLOBAL, SELF, interaction_property), ))
	      
	def Create_Set_Face(x,y,z,model,part,set_name):
	    face = ()
	    p = mdb.models[model].parts[part]
	    f = p.faces
	    myFace = f.findAt((x,y,z),)
	    face = face + (f[myFace.index:myFace.index+1], )
	    p.Set(faces=face, name=set_name)
	     
	def Create_Set_surf(x,y,z,model,part,surface_name):
	    face = ()
	    p = mdb.models[model].parts[part]
	    s = p.faces
	    myFace = s.findAt((x,y,z),)
	    face = face + (s[myFace.index:myFace.index+1], )
	    p.Surface(side1Faces=face, name=surface_name)

	def Create_Set_edge(x,y,z,model,part,edge_name):
	    edge = ()
	    p = mdb.models[model].parts[part]
	    e = p.edges
	    myedge = e.findAt((x,y,z),)
	    edge = edge + (e[myedge.index:myedge.index+1], )
	    p.Set(edges=edge, name=edge_name)
	    ID_edge = edge.index
	    return ID_edge


	def create_reference_point(model,x,y,z,RF_name):
	    a = mdb.models[model].rootAssembly
	    RP = a.ReferencePoint(point=(x,y,z))
	    r = a.referencePoints
	    myRP = r.findAt((x,y,z),)
	    myRF1 = (myRP,)
	    a.Set(referencePoints=myRF1, name=RF_name)

	def create_rigidbody(model,instance_name,surface_name,RF_name):
	   a = mdb.models[model].rootAssembly
	   region5=a.instances[instance_name].surfaces[surface_name]
	   region1=a.sets[RF_name]
	   mdb.models[model].RigidBody(name='Constraint-1', 
	   refPointRegion=region1, surfaceRegion=region5) 

	def Create_CL(model,instance_name,CL_name,set_name,myu1,myu2,myu3,myur1,myur2,myur3,myamplitude):
	    a = mdb.models[model].rootAssembly
	    region = a.instances[instance_name].sets[set_name]
	    mdb.models[model].DisplacementBC(name=CL_name, 
	    createStepName="p_mise_en_pression", region=region, u1=myu1, u2=myu2, u3=myu3, 
	    ur1=myur1, ur2=myur2, ur3=myur3, amplitude=myamplitude, fixed=OFF, 
	    distributionType=UNIFORM, fieldName='', localCsys=None)

	def Create_CL_sphere(model,step_name,CL_name,set_name,myu1,myu2,myu3,myur1,myur2,myur3,myamplitude):
	    a = mdb.models[model].rootAssembly
	    region = a.sets[set_name]
	    mdb.models[model].DisplacementBC(name=CL_name, 
	    createStepName=step_name, region=region, u1=myu1, u2=myu2, u3=myu3, 
	    ur1=myur1, ur2=myur2, ur3=myur3, amplitude=myamplitude, fixed=OFF, 
	    distributionType=UNIFORM, fieldName='', localCsys=None)
	   

	def create_datum_plane(model,part,plane_name,transition):
	   p = mdb.models[model].parts[part]
	   myplane = p.DatumPlaneByPrincipalPlane(principalPlane=plane_name, offset=transition)
	   myID = myplane.id
	   return myID

	def create_axis(model,part):
	   p = mdb.models[model].parts[part]
	   y = p.DatumAxisByPrincipalAxis(principalAxis=ZAXIS)       
	   myID_axis = y.id
	   return myID_axis


	def Datume_plane_rotation(model,part,A,B):
	   p = mdb.models[model].parts[part]
	   d2 = p.datums
	   y = p.DatumPlaneByRotation(plane=d2[A], axis=d2[B], angle=40.0)
	   myID_rotation = y.id
	   return myID_rotation

	def create_partition(model,part,ID):
	   p = mdb.models[model].parts[part]
	   c = p.cells[:]
	   d1 = p.datums
	   p.PartitionCellByDatumPlane(datumPlane=d1[ID], cells=c) 


	def Create_seedEdges(x,y,z,seed_size):
	    p = mdb.models['Buckling_analysis'].parts['cylindre']
	    e = p.edges
	    myedge = e.findAt((x,y,z),)
	    p.seedEdgeBySize(edges=(myedge,), size=seed_size, deviationFactor=0.1, 
	    constraint=FINER)


	def Create_seedEdges_bias(x,y,z,element_number,Flip='non'):
	    p = mdb.models['Buckling_analysis'].parts['cylindre']
	    e = p.edges
	    myedge = e.findAt((x,y,z),)
	    if Flip == 'non' : p.seedEdgeByBias(biasMethod=SINGLE, end1Edges=(myedge,), ratio=5.0, number=element_number, constraint=FINER)
	    else : p.seedEdgeByBias(biasMethod=SINGLE, end2Edges=(myedge,), ratio=5.0, number=element_number, constraint=FINER)

	def Create_Mesh(model,part,x,y,z):
	    p = mdb.models[model].parts[part]
	    c = p.cells
	    pickedRegions = c.findAt((x,y,z),)
	    p.generateMesh(regions=(pickedRegions,))  
	   
	def generate_mesh(model,part):
	    p = mdb.models[model].parts[part]
	    p.generateMesh() 

	def Create_Job(job_name): 
	    mdb.Job(name=job_name, model='Buckling_analysis', description='', type=ANALYSIS, 
	    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
	    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
	    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
	    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
	    scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=1, 
	    numGPUs=0)

	def submit_job(job_name):
	   mdb.jobs[job_name].submit(consistencyChecking=OFF)
	   mdb.jobs[job_name].waitForCompletion()

	#fonctions

	create_part_3D_cylinder((Diametre/2)-Epaisseur,(Diametre/2),Longueur_Tube,"cylindre",'Buckling_analysis')
	create_part_3D_sphere('Buckling_analysis','sphere',15.0)
	create_material_data('Buckling_analysis',"steel",210000.0,0.3,300.0,0.0,550.0,0.1)
	Create_section('Buckling_analysis','cylindre','Section-1','steel')
	Create_Set_all_cells('Buckling_analysis','cylindre','Set-all_cells')
	assign_section('Buckling_analysis','cylindre','Set-all_cells','Section-1')
	create_assembly('Buckling_analysis','cylindre','sphere','cylindre-1','sphere-1')

	create_analysis_step('Buckling_analysis',"p_mise_en_pression",mymin_inc,mymax_inc,"Initial")
	create_analysis_step('Buckling_analysis',"p_enfoncement",mymin_inc,mymax_inc,"p_mise_en_pression")
	create_analysis_step('Buckling_analysis',"p_desenfoncement",mymin_inc,mymax_inc,"p_enfoncement")

	create_interaction_property('Buckling_analysis','IntProp-1')
	create_interaction('Buckling_analysis','Initial','Int-1','IntProp-1')

	Create_Set_Face(0.0,-my_x,my_z,'Buckling_analysis',"cylindre",'face1')
	Create_Set_Face(0.0,my_x,my_z,'Buckling_analysis',"cylindre",'face2')
	Create_Set_Face(my_x,0.0,0.0,'Buckling_analysis',"cylindre",'face3')
	Create_Set_Face(my_x,0.0,Longueur_Tube,'Buckling_analysis',"cylindre",'face4')
	Create_Set_edge(0.0,-((Diametre/2)-(Epaisseur)),my_z,'Buckling_analysis',"cylindre",'set_5')
	Create_Set_surf(((Diametre/2)-(Epaisseur))*cos(theta),((Diametre/2)-(Epaisseur))*sin(theta),Longueur_Tube/2.,'Buckling_analysis',"cylindre",'Surf-1')            
	Create_Set_surf(0.0,-15.0,0.0,'Buckling_analysis','sphere','Surf-2')
	create_reference_point('Buckling_analysis',0.0, 35.0, 0.0,'RF_1')
	create_rigidbody('Buckling_analysis','sphere-1','Surf-2','RF_1')

	Create_CL('Buckling_analysis','cylindre-1',"BC_1","face1",0.0,UNSET,UNSET,UNSET,UNSET,UNSET,UNSET)
	Create_CL('Buckling_analysis','cylindre-1',"BC_2","face2",0.0,UNSET,UNSET,UNSET,UNSET,UNSET,UNSET)
	Create_CL('Buckling_analysis','cylindre-1',"BC_3",'face3',UNSET,UNSET,0.0,UNSET,UNSET,UNSET,UNSET)
	Create_CL('Buckling_analysis','cylindre-1',"BC_4",'face4',UNSET,UNSET,0.0,UNSET,UNSET,UNSET,UNSET)
	Create_CL('Buckling_analysis','cylindre-1',"BC_5",'set_5',UNSET,0.0,UNSET,UNSET,UNSET,UNSET,UNSET)
	Create_CL_sphere('Buckling_analysis',"p_enfoncement","BC_6",'RF_1',0.0,-5.0,0.0,0.0,0.0,0.0,UNSET)
	mdb.models['Buckling_analysis'].boundaryConditions['BC_6'].setValuesInStep(stepName="p_desenfoncement", u2=30.0)    #desenfoncement

	myID_axis_1 = create_axis('Buckling_analysis',"cylindre")
	myID_1 = create_datum_plane('Buckling_analysis',"cylindre",XZPLANE,0.0)
	myID_2 = create_datum_plane('Buckling_analysis',"cylindre",XYPLANE,10.)
	myID_rotation_1 = Datume_plane_rotation('Buckling_analysis',"cylindre",myID_1,myID_axis_1)
	create_partition('Buckling_analysis',"cylindre",myID_rotation_1)
	create_partition('Buckling_analysis',"cylindre",myID_2)

	#Create seedEdges
	List_seedEdges = [[((Diametre/2)-(Epaisseur))*cos(epss),((Diametre/2)-(Epaisseur))*sin(epss),0.0,seedsize],     [0.0,(Diametre/2.)-(Epaisseur/2.),0.0,seedsize],     [(Diametre/2)*cos(epss),(Diametre/2)*sin(epss),0.0,seedsize],        
	[(Diametre/2)*cos(epss),(Diametre/2)*sin(epss),10.0,seedsize],
		          [0.0,(Diametre/2.),5.0,seedsize],        [(Diametre/2)*cos(theta),(Diametre/2.)*sin(theta),5.0,seedsize],     [0.0,(Diametre/2.)-(Epaisseur),5.0,seedsize],         [0.0,(Diametre/2.)-(Epaisseur/2.),10.0,seedsize],  
		          [((Diametre/2)-(Epaisseur/2))*cos(theta),((Diametre/2)-(Epaisseur/2))*sin(theta),0.0,seedsize],      [((Diametre/2)-(Epaisseur))*cos(theta),((Diametre/2)-(Epaisseur))*sin(theta),5.0,seedsize],            [((Diametre/2)-(Epaisseur))*cos(epss),((Diametre/2)-(Epaisseur))*sin(epss),10.0,seedsize]]
	for Edge in List_seedEdges: Create_seedEdges(Edge[0], Edge[1], Edge[2], Edge[3])


	List_seedEdgesbias = [[(Diametre/2),0.0,0.0,25,'non'], [(Diametre/2)-(Epaisseur),0.0,0.0,25,'non'], [(Diametre/2),0.0,10.0,25,'oui'], [(Diametre/2)-(Epaisseur),0.0,10.0,25,'non'],
		              [(Diametre/2),0.0,Longueur_Tube,25,'non'], [(Diametre/2)-(Epaisseur),0.0,Longueur_Tube,25,'oui'], [(Diametre/2)*cos(theta),(Diametre/2.)*sin(theta),Longueur_Tube/2.,70,'oui'],         [0.0,(Diametre/2.),Longueur_Tube/2.,70,'non'], [0.0,-(Diametre/2.),Longueur_Tube/2.,70,'non'], [0.0,-((Diametre/2)-(Epaisseur)),Longueur_Tube/2.,70,'non'], [0.0,((Diametre/2)-(Epaisseur)),Longueur_Tube/2.,70,'non'], [0.0,((Diametre/2)-(Epaisseur)),Longueur_Tube/2.,70,'non'], [((Diametre/2)-(Epaisseur))*cos(theta),((Diametre/2)-(Epaisseur))*sin(theta),Longueur_Tube/2.,70,'non']]
	for Edge in List_seedEdgesbias:
	    if len(Edge) == 5 : Create_seedEdges_bias(Edge[0], Edge[1], Edge[2], Edge[3], Edge[4])
	    else : Create_seedEdges_bias(Edge[0], Edge[1], Edge[2], Edge[3])






	Create_Mesh('Buckling_analysis',"cylindre",0.0,(Diametre/2.)-(Epaisseur/2.),0.0)
	generate_mesh('Buckling_analysis',"cylindre")

	Create_Job(str(k))
#element type changement 
 
#lineeaire 

        '''p = mdb.models['Buckling_analysis'].parts['cylindre']
        elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD, secondOrderAccuracy=OFF, distortionControl=DEFAULT)
        elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
        elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
        c = p.cells
        pickedRegions = c.findAt((0.0,(Diametre/2.)-(Epaisseur/2.),0.0),)
        p.setElementType(regions=(pickedRegions,), elemTypes=(elemType1, elemType2, elemType3))

        p = mdb.models['Buckling_analysis'].parts['cylindre']
	session.viewports['Viewport: 1'].setValues(displayedObject=p)
	a = mdb.models['Model-1'].rootAssembly
	session.viewports['Viewport: 1'].setValues(displayedObject=a)
	session.viewports['Viewport: 1'].assemblyDisplay.setValues(
	adaptiveMeshConstraints=ON, optimizationTasks=OFF, 
	geometricRestrictions=OFF, stopConditions=OFF)
	mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial', nlgeom=ON)
	session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-1')
	mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
	    'S', 'E'))'''
#quadratique 
	
        p = mdb.models['Buckling_analysis'].parts['cylindre']
        elemType1 = mesh.ElemType(elemCode=C3D20, elemLibrary=STANDARD)
	elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
	elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
        c = p.cells
        pickedRegions = c.findAt((0.0,(Diametre/2.)-(Epaisseur/2.),0.0),)
        p.setElementType(regions=(pickedRegions,), elemTypes=(elemType1, elemType2, elemType3))


	#submit_job(str(k))
n=40.0
for i in range(20,26):
     n=n/2.0
     fonction_generale(n,i)

fonction_generale(0.4,400)


