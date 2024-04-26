# Author: @Kevin_Wang
# Date: 03/20/2023
# Update: 04/23/2024
# Based on Abaqus/CAE 2019
# Encoding "UTF-8"
import os 
import numpy as np
from Table import Table_D, Table_s
import csv
from collections import OrderedDict
'''---------------------------------------------------------'''
from abaqus import *
from abaqusConstants import *
from caeModules import *
from driverUtils import *
'''---------------------------------------------------------'''
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
'''---------------------------------------------------------'''
session.journalOptions.setValues(replayGeometry=COORDINATE,recoverGeometry=COORDINATE)
executeOnCaeStartup()
# Parameters Unit:millimeter
Length_CFRP_bond = 170
Length_steel = 380
Width_CFRP = 50
Width_steel = 138
Thickness_CFRP = 1.4
Thickness_steel = 20
Displacement_MAX = 4
D_defect = 0
D_position = 0
D_shape = 'RD'
Step_Mode = AUTOMATIC
Table = tuple(zip(Table_D,Table_s))
# Path
Dir = os.path.dirname(os.path.abspath("__file__")).replace('\\','/')
with open("./ModelInfo.csv",'r') as csvfile:
    reader = csv.DictReader(csvfile)
    ModelInfo = list(reader)
for Info in ModelInfo:
    Length_CFRP_bond = int(Info["Length_CFRP_bond"])
    D_defect = int(Info["D_defect"])
    D_position = int(Info["D_position"])
    if D_defect == 0:
        D_position = 0
    Name = 'A'+str(Length_CFRP_bond)+'-D'+str(D_defect).replace('.','_')+'-P'+str(D_position)+'-'+D_shape+'-'+str(Step_Mode)
    if not os.path.exists(Dir+'/'+Name):
        os.makedirs(Dir+'/'+Name)
    os.chdir(Dir+'/'+Name)
    Length_CFRP = Length_CFRP_bond + 100
    '''---------------------------------------------------------'''
    # Model
    model = mdb.Model(name=Name)
    # Part_steel
    Sketch_steel = model.ConstrainedSketch(name='sketch_steel', 
                                        sheetSize=200.0)
    Sketch_steel.rectangle(point1=(0.0, 0.0), 
                        point2=(Length_steel, Width_steel))
    Part_steel = model.Part(dimensionality=THREE_D, 
                        name='steel', 
                        type=DEFORMABLE_BODY)
    Part_steel.BaseSolidExtrude(depth=Thickness_steel, sketch=Sketch_steel)
    # Part_CFRP
    Sketch_CFRP = model.ConstrainedSketch(name='sketch_CFRP', 
                                        sheetSize=200.0)
    Sketch_CFRP.rectangle(point1=(0.0, 0.0), 
                        point2=(Length_CFRP, Width_CFRP))
    Part_CFRP = model.Part(dimensionality=THREE_D, 
                        name='CFRP', 
                        type=DEFORMABLE_BODY)
    Part_CFRP.BaseSolidExtrude(depth=Thickness_CFRP, sketch=Sketch_CFRP)
    # Material
    Material_steel = model.Material(name='steel')
    Material_steel.Elastic(table=((198000, 0.3), ))
    # Material_steel.Plastic(table=((334.0, 0.0), (531.0, 0.127)))
    Material_CFRP = model.Material(name='CFRP')
    Material_CFRP.Elastic(table=((165000, 0.28), )) # High modulus: 478730
    # Section
    model.HomogeneousSolidSection(material='steel', 
                                name='steel', 
                                thickness=None)
    model.HomogeneousSolidSection(material='CFRP', 
                                name='CFRP', 
                                thickness=None)
    Part_steel.SectionAssignment(offset=0.0, 
                                offsetField='', 
                                offsetType=MIDDLE_SURFACE, 
                                region=Region(cells=Part_steel.cells.findAt(((Length_steel, Width_steel/2, Thickness_steel/2),),)), 
                                sectionName='steel', 
                                thicknessAssignment=FROM_SECTION)
    Part_CFRP.SectionAssignment(offset=0.0, 
                            offsetField='', 
                            offsetType=MIDDLE_SURFACE, 
                            region=Region(cells=Part_CFRP.cells.findAt(((Length_CFRP, Width_CFRP/2, Thickness_CFRP/2),),)), 
                            sectionName='CFRP', 
                            thicknessAssignment=FROM_SECTION)
    # Assemble
    model.rootAssembly.DatumCsysByDefault(CARTESIAN)
    Instance_steel = model.rootAssembly.Instance(dependent=OFF, 
                                                name='steel-1', 
                                                part=Part_steel)
    Instance_CFRP = model.rootAssembly.Instance(dependent=OFF, 
                                            name='CFRP-1', 
                                            part=Part_CFRP)
    Instance_CFRP.translate(vector=(Length_steel-Length_CFRP_bond, (Width_steel-Width_CFRP)/2, Thickness_steel))
    BC_load = Instance_CFRP.faces.findAt(((Length_steel-Length_CFRP_bond+Length_CFRP, (Width_steel-Width_CFRP)/2+Width_CFRP/2, Thickness_steel+Thickness_CFRP/2),),)
    BC_fixed = Instance_steel.faces.findAt(((Length_steel, Width_steel/2, Thickness_steel/2), ), )
    BC_bottom = Instance_steel.faces.findAt(((Length_steel, Width_steel/2, 0.0), ), )
    model.rootAssembly.Set(faces=BC_load, name='Set_BC_load')
    model.rootAssembly.Set(faces=BC_fixed, name='Set_BC_fixed')
    model.rootAssembly.Set(faces=BC_bottom, name='Set_BC_bottom')
    model.rootAssembly.Set(faces=BC_load, name='FACE_LOAD')
    # Parition
    model.rootAssembly.PartitionFaceByProjectingEdges(edges=Instance_steel.edges.findAt(((Length_steel, Width_steel/2, Thickness_steel), ), ), 
                                                    extendEdges=False, 
                                                    faces=Instance_CFRP.faces.findAt(((Length_steel, Width_steel/2, Thickness_steel), ), ))
    if D_defect != 0:
        Sketch_Defect_Partition = model.ConstrainedSketch(name='Sketch_Defect_Partition', 
                                                sheetSize=200, 
                                                transform=model.rootAssembly.MakeSketchTransform(
                                                        sketchPlane=Instance_steel.faces[4], 
                                                        sketchPlaneSide=SIDE1, 
                                                        sketchUpEdge=Instance_steel.edges[7], 
                                                        sketchOrientation=RIGHT, origin=(0, 0, 0)))
        
        if D_shape == 'RD':
            Sketch_Defect_Partition.rectangle(point1=(D_position-D_defect/2, (Width_steel-Width_CFRP)/2), point2=(D_position+D_defect/2, (Width_steel-Width_CFRP)/2+Width_CFRP))
            model.rootAssembly.PartitionFaceBySketch(faces=Instance_steel.faces[4], 
                                                sketch=Sketch_Defect_Partition, 
                                                sketchUpEdge=Instance_steel.edges[7])
            # cell partition
            PickedCells = Instance_steel.cells.findAt(((Length_steel-Length_CFRP_bond,Width_steel/2,Thickness_steel),))
            PickedVertices = Instance_steel.vertices.findAt(coordinates=(D_position+D_defect/2,Width_steel/2+Width_CFRP/2,Thickness_steel))
            PickedEdges = Instance_steel.edges.findAt(coordinates=(D_position+D_defect/2,Width_steel/2,Thickness_steel))
            model.rootAssembly.PartitionCellByPlanePointNormal(point=PickedVertices,normal=PickedEdges,cells=PickedCells)
            PickedCells = Instance_steel.cells.findAt(((Length_steel-Length_CFRP_bond,Width_steel/2,Thickness_steel),))
            PickedVertices = Instance_steel.vertices.findAt(coordinates=(D_position+D_defect/2,Width_steel/2-Width_CFRP/2,Thickness_steel))
            PickedEdges = Instance_steel.edges.findAt(coordinates=(D_position+D_defect/2,Width_steel/2,Thickness_steel))
            model.rootAssembly.PartitionCellByPlanePointNormal(point=PickedVertices,normal=PickedEdges,cells=PickedCells)
            PickedCells = Instance_steel.cells.findAt(((Length_steel-Length_CFRP_bond,Width_steel-1,Thickness_steel),))
            PickedVertices = Instance_steel.vertices.findAt(coordinates=(D_position+D_defect/2,Width_steel/2+Width_CFRP/2,Thickness_steel))
            PickedEdges = Instance_steel.edges.findAt(coordinates=(D_position,Width_steel/2+Width_CFRP/2,Thickness_steel))
            model.rootAssembly.PartitionCellByPlanePointNormal(point=PickedVertices,normal=PickedEdges,cells=PickedCells)
            PickedCells = Instance_steel.cells.findAt(((Length_steel-Length_CFRP_bond,Width_steel-1,Thickness_steel),))
            PickedVertices = Instance_steel.vertices.findAt(coordinates=(D_position-D_defect/2,Width_steel/2+Width_CFRP/2,Thickness_steel))
            PickedEdges = Instance_steel.edges.findAt(coordinates=(D_position,Width_steel/2+Width_CFRP/2,Thickness_steel))
            model.rootAssembly.PartitionCellByPlanePointNormal(point=PickedVertices,normal=PickedEdges,cells=PickedCells)                   
            PickedCells = Instance_steel.cells.findAt(((Length_steel-Length_CFRP_bond,1,Thickness_steel),))
            PickedVertices = Instance_steel.vertices.findAt(coordinates=(D_position+D_defect/2,Width_steel/2-Width_CFRP/2,Thickness_steel))
            PickedEdges = Instance_steel.edges.findAt(coordinates=(D_position,Width_steel/2-Width_CFRP/2,Thickness_steel))
            model.rootAssembly.PartitionCellByPlanePointNormal(point=PickedVertices,normal=PickedEdges,cells=PickedCells)
            PickedCells = Instance_steel.cells.findAt(((Length_steel-Length_CFRP_bond,1,Thickness_steel),))
            PickedVertices = Instance_steel.vertices.findAt(coordinates=(D_position-D_defect/2,Width_steel/2-Width_CFRP/2,Thickness_steel))
            PickedEdges = Instance_steel.edges.findAt(coordinates=(D_position,Width_steel/2-Width_CFRP/2,Thickness_steel))
            model.rootAssembly.PartitionCellByPlanePointNormal(point=PickedVertices,normal=PickedEdges,cells=PickedCells)
            PickedCells = Instance_steel.cells.findAt(((Length_steel-Length_CFRP_bond,Width_steel/2,Thickness_steel),))
            PickedVertices = Instance_steel.vertices.findAt(coordinates=(D_position+D_defect/2,Width_steel/2+Width_CFRP/2,Thickness_steel))
            PickedEdges = Instance_steel.edges.findAt(coordinates=(D_position,Width_steel/2+Width_CFRP/2,Thickness_steel))
            model.rootAssembly.PartitionCellByPlanePointNormal(point=PickedVertices,normal=PickedEdges,cells=PickedCells)
            PickedCells = Instance_steel.cells.findAt(((Length_steel-Length_CFRP_bond,Width_steel/2,Thickness_steel),))
            PickedVertices = Instance_steel.vertices.findAt(coordinates=(D_position-D_defect/2,Width_steel/2+Width_CFRP/2,Thickness_steel))
            PickedEdges = Instance_steel.edges.findAt(coordinates=(D_position,Width_steel/2+Width_CFRP/2,Thickness_steel))
            model.rootAssembly.PartitionCellByPlanePointNormal(point=PickedVertices,normal=PickedEdges,cells=PickedCells)
        if D_shape == 'CD':
            Sketch_Defect_Partition.CircleByCenterPerimeter(center=(D_position, (Width_steel-Width_CFRP)/2+Width_CFRP/2), point1=(D_position+D_defect/2, (Width_steel-Width_CFRP)/2+Width_CFRP/2))
            model.rootAssembly.PartitionFaceBySketch(faces=Instance_steel.faces[4], 
                                                sketch=Sketch_Defect_Partition, 
                                                sketchUpEdge=Instance_steel.edges[7])
    # Edge
    Edge_Displacement = Instance_CFRP.edges.findAt(((Length_steel, Width_steel/2, Thickness_steel), ), )
    # Faces
    Face_steel_U1 = Instance_steel.faces.findAt(((Length_steel-Length_CFRP_bond+1, Width_steel/2, Thickness_steel),),)
    Face_steel_U2 = Instance_steel.faces.findAt(((Length_steel-1, Width_steel/2, Thickness_steel),),)
    Face_CFRP_L = Instance_CFRP.faces.findAt(((Length_steel-Length_CFRP_bond+0.1, Width_steel/2, Thickness_steel),),)
    # Sets
    model.rootAssembly.Set(edges=Edge_Displacement, name='EDGE_DISPLACEMENT')
    # Step
    step = model.StaticStep(continueDampingFactors=False, 
                        initialInc=0.01, 
                        maxInc=0.05, 
                        maxNumInc=100000000, 
                        minInc=1e-20, 
                        name='Tension', 
                        nlgeom=ON, 
                        previous='Initial', 
                        stabilizationMagnitude=0.0002, 
                        stabilizationMethod=DAMPING_FACTOR,
                        timePeriod=1.0, 
                        noStop=ON,
                        timeIncrementationMethod=Step_Mode, 
                        description='')
    # Interaction
    model.ContactProperty('CZM')
    model.interactionProperties['CZM'].CohesiveBehavior(defaultPenalties=OFF, 
                                                    table=((1750, 258.491, 258.491), ))
    model.interactionProperties['CZM'].Damage(evolTable=Table, 
                                            useEvolution=ON,
                                            initTable=((15.1, 13.7, 13.7), ), 
                                            softening=TABULAR, 
                                            criterion=QUAD_TRACTION)
    model.SurfaceToSurfaceContactStd(adjustMethod=NONE, 
                                    clearanceRegion=None, 
                                    createStepName='Initial', 
                                    datumAxis=None, 
                                    initialClearance=OMIT, 
                                    interactionProperty='CZM', 
                                    slave=Region(side1Faces=Face_CFRP_L), 
                                    name='Cohesive_U1', 
                                    master=Region(side1Faces=Face_steel_U1),  
                                    sliding=SMALL, 
                                    thickness=ON)
    if D_defect != 0:
        model.SurfaceToSurfaceContactStd(adjustMethod=NONE, 
                                    clearanceRegion=None, 
                                    createStepName='Initial', 
                                    datumAxis=None, 
                                    initialClearance=OMIT, 
                                    interactionProperty='CZM', 
                                    slave=Region(side1Faces=Face_CFRP_L), 
                                    name='Cohesive_U2', 
                                    master=Region(side1Faces=Face_steel_U2),  
                                    sliding=SMALL, 
                                    thickness=ON)
    # DisplacementBC
    model.DisplacementBC(amplitude=UNSET, 
                        createStepName='Initial', 
                        distributionType=UNIFORM, 
                        fieldName='', 
                        localCsys=None, 
                        name='BC_STEEL_FIXED', 
                        region=model.rootAssembly.sets['Set_BC_fixed'], 
                        u1=SET, 
                        u2=UNSET, 
                        u3=UNSET, 
                        ur1=UNSET, 
                        ur2=UNSET, 
                        ur3=UNSET)
    model.DisplacementBC(amplitude=UNSET, 
                        createStepName='Tension', 
                        distributionType=UNIFORM, 
                        fieldName='', 
                        localCsys=None, 
                        name='LOAD', 
                        region=model.rootAssembly.sets['Set_BC_load'], 
                        u1=Displacement_MAX, 
                        u2=UNSET, 
                        u3=UNSET, 
                        ur1=UNSET, 
                        ur2=UNSET, 
                        ur3=UNSET)
    model.DisplacementBC(amplitude=UNSET, 
                        createStepName='Initial', 
                        distributionType=UNIFORM, 
                        fieldName='', 
                        localCsys=None, 
                        name='BC_STEEL_BOTTOM', 
                        region=model.rootAssembly.sets['Set_BC_bottom'], 
                        u1=UNSET, 
                        u2=UNSET, 
                        u3=SET, 
                        ur1=UNSET, 
                        ur2=UNSET, 
                        ur3=UNSET)
    # FieldOutput
    model.fieldOutputRequests['F-Output-1'].setValues(
        variables=('S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CSTRESS', 'CDISP', 'SDEG','CSDMG'),
        timeInterval=0.01
        )
    # Mesh
    pickedEdges = Instance_steel.edges.findAt(((D_position,Width_steel/2+Width_CFRP/2,Thickness_steel), ), 
                                            ((D_position,Width_steel/2-Width_CFRP/2,Thickness_steel), ))
    model.rootAssembly.seedEdgeBySize(edges=pickedEdges, 
                        size=5.0, 
                        deviationFactor=0.1, 
                        minSizeFactor=0.1, constraint=FINER)
    model.rootAssembly.seedPartInstance(deviationFactor=0.1, 
                                    minSizeFactor=0.1, 
                                    regions=(Instance_steel,), 
                                    size=10)
    model.rootAssembly.seedPartInstance(deviationFactor=0.1, 
                                    minSizeFactor=0.1, 
                                    regions=(Instance_CFRP,), 
                                    size=2)
    model.rootAssembly.generateMesh(regions=(Instance_steel, Instance_CFRP))
    # JOB
    MyJob = mdb.Job(atTime=None, 
                contactPrint=OFF, 
                description='', 
                echoPrint=OFF, 
                explicitPrecision=SINGLE, 
                getMemoryFromAnalysis=True, 
                historyPrint=OFF, 
                memory=90, 
                memoryUnits=PERCENTAGE, 
                model=Name, 
                modelPrint=OFF, 
                multiprocessingMode=DEFAULT, 
                name=Name, 
                nodalOutputPrecision=SINGLE, 
                numCpus=8, 
                numDomains=8, 
                numGPUs=1, 
                queue=None, 
                resultsFormat=ODB, 
                scratch='', 
                type=ANALYSIS, 
                userSubroutine='', 
                waitHours=0, 
                waitMinutes=0)
    mdb.saveAs(Name)
    MyJob.writeInput()
    MyJob.submit(consistencyChecking=OFF)
    MyJob.waitForCompletion()
    # Post-process
    MyOdb = session.openOdb(Name+'.odb',readOnly=False)
    session.viewports['Viewport: 1'].setValues(displayedObject=MyOdb)
    DataList_Load = session.xyDataListFromField(odb=MyOdb, 
                                        outputPosition=NODAL,
                                        variable=(('RF', NODAL, ((COMPONENT, 'RF1'),)),),
                                        nodeSets=("FACE_LOAD", ))
    DataList_Displacement = session.xyDataListFromField(odb=MyOdb, 
                                        outputPosition=NODAL,
                                        variable=(('U', NODAL, ((COMPONENT, 'U1'),)),),
                                        nodeSets=("EDGE_DISPLACEMENT", ))
    DataArray_Displacement = np.array(DataList_Displacement)
    Num_Node = DataArray_Displacement.shape[0]
    DataArray_Time = DataArray_Displacement[0][:,0]
    DataArray_Displacement = np.sum(DataArray_Displacement, axis=0)[:,1]
    DataArray_Displacement = DataArray_Displacement/Num_Node
    DataArray_Load = np.array(DataList_Load)
    DataArray_Load = np.sum(DataArray_Load, axis=0)[:,1]
    DataArray_Load = DataArray_Load/1000
    MyData  = np.zeros((DataArray_Load.shape[0],3), dtype ='float')
    MyData[:,0] = DataArray_Time
    MyData[:,1] = DataArray_Displacement
    MyData[:,2] = DataArray_Load
    Force_max = np.max(DataArray_Load)
    Info["Force"] = Force_max
    with open('../Results.csv','wb') as csvfile:
        fieldnames = ['No.','L1','L2','LD','Length_CFRP_bond','D_defect','D_position','Force']
        writer = csv.DictWriter(csvfile,fieldnames=fieldnames)
        writer.writeheader()
        for row in ModelInfo:
            ordered_row = OrderedDict((key, row[key]) for key in fieldnames)
            writer.writerow(ordered_row)
    # DataSaving
    if os.path.exists("./"+Name+".csv"):
        os.remove("./"+Name+".csv")
    np.savetxt(Name+'.csv',MyData,delimiter=',')
    del mdb.models[Name], model
    del MyOdb, DataList_Load, DataList_Displacement, MyData
    del DataArray_Displacement, DataArray_Load, DataArray_Time, Num_Node
