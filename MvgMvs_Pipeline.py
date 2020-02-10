#!/usr/bin/python
#! -*- encoding: utf-8 -*-
#
# Created by @FlachyJoe
#
# this script is for easy use of OpenMVG and OpenMVS
#
#usage: SfM_MyPipeline.py [-h] [-s FIRST_STEP] [--0 0 [0 ...]]
#                         [--1 1 [1 ...]] [--2 2 [2 ...]] [--3 3 [3 ...]]
#                         [--4 4 [4 ...]] [--5 5 [5 ...]] [--6 6 [6 ...]]
#                         [--7 7 [7 ...]] [--8 8 [8 ...]] [--9 9 [9 ...]]
#                         [--10 10 [10 ...]] [--11 11 [11 ...]][--12 12 [12 ...]]
#                         [--13 13 [13 ...]]
#                         input_dir output_dir
#
#Photogrammetry reconstruction with these steps :
#   0. Intrinsics analysis          openMVG_main_SfMInit_ImageListing
#   1. Compute features             openMVG_main_ComputeFeatures
#   2. Compute matches              openMVG_main_ComputeMatches
#   3. Incremental reconstruction   openMVG_main_IncrementalSfM
#   4. Global reconstruction        openMVG_main_GlobalSfM
#   5. Colorize Structure           openMVG_main_ComputeSfM_DataColor
#   6. Structure from Known Poses   openMVG_main_ComputeStructureFromKnownPoses
#   7. Colorized robust triangulation    openMVG_main_ComputeSfM_DataColor
#   8. Control points registration  ui_openMVG_control_points_registration

#   9. Export to openMVS            openMVG_main_openMVG2openMVS
#   10. Densify point cloud         OpenMVS/DensifyPointCloud
#   11. Reconstruct the mesh        OpenMVS/ReconstructMesh
#   12. Refine the mesh             OpenMVS/RefineMesh
#   13. Texture the mesh            OpenMVS/TextureMesh

######## Todo : MVE support
#   14. Export to MVE               openMVG_main_openMVG2MVE
#   15. Creating depth map          dmrecon
#   16. Convert Scene to Pointset   scene2pset
#   17.

#~ # Convert the openMVG SfM scene to the MVE format
#~ $ openMVG_main_openMVG2MVE -i Dataset/outReconstruction/sfm_data.json -o Dataset/outReconstruction

#~ #--
#~ # shell script example
#~ #--

#~ directory=Dataset/outReconstruction/MVE
#~ resolution=2

#~ # MVE
#~ dmrecon -s$resolution $directory
#~ scene2pset -ddepth-L$resolution -iundist-L$resolution -n -s -c $directory $directory/OUTPUT.ply
#~ fssrecon $directory/OUTPUT.ply $directory/OUTPUT_MESH.ply
#~ meshclean $directory/OUTPUT_MESH.ply $directory/OUTPUT_MESH_CLEAN.ply
#############################

#
#positional arguments:
#  input_dir             the directory wich contains the pictures set.
#  output_dir            the directory wich will contain the resulting files.
#
#optional arguments:
#  -h, --help            show this help message and exit
#  --steps STEPS
#                        Space separated list of steps to process
#                        Default: 0 1 2 3 8 9 10 11 12 13
#                        ie openMVG sequential reconstruction and openMVS
#                        full mesh reconstruction and texturing
#  --preset PRESET       To be coded
#
#Passthrough:
#  Option to be pass to command lines (remove - in front of option names)
#  e.g. --1 p ULTRA to use the ULTRA preset in openMVG_main_ComputeFeatures


import commands
import os
import subprocess
import sys

# Indicate the openMVG and openMVS binary directories
OPENMVG_SFM_BIN = "/usr/local/bin/"
OPENMVS_BIN = "/usr/local/bin/"

# Indicate the openMVG camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "/usr/local/share/openMVG/"

DEBUG=False

## HELPERS for terminal colors
BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)
NO_EFFECT, BOLD, UNDERLINE, BLINK, INVERSE, HIDDEN = (0,1,4,5,7,8)

SEQUENTIAL_STEPS = [0,1,2,3,8,9,10,11,12,13]
GLOBAL_STEPS = [0,1,2,4,8,9,10,11,12,13]
MVG_SEQ_STEPS=[0,1,2,3,5,6,7]
MVG_GLOBAL_STEPS=[0,1,2,4,5,6,7]

#from Python cookbook, #475186
def has_colours(stream):
    if not hasattr(stream, "isatty"):
        return False
    if not stream.isatty():
        return False # auto color only on TTYs
    try:
        import curses
        curses.setupterm()
        return curses.tigetnum("colors") > 2
    except:
        # guess false in case of error
        return False
has_colours = has_colours(sys.stdout)

def printout(text, colour=WHITE, background=BLACK, effect=NO_EFFECT):
        if has_colours:
                seq = "\x1b[%d;%d;%dm" % (effect, 30+colour, 40+background) + text + "\x1b[0m"
                sys.stdout.write(seq+'\r\n')
        else:
                sys.stdout.write(text+'\r\n')

## OBJECTS to store config and data in

class ConfContainer(object):
    """Container for all the config variables"""
    pass

conf=ConfContainer()

class aStep:
    def __init__(self, info, cmd, opt):
        self.info = info
        self.cmd = cmd
        self.opt = opt

class stepsStore :
    def __init__(self):
        self.steps_data=[
            [   "Intrinsics analysis",          #0
                os.path.join(OPENMVG_SFM_BIN,"openMVG_main_SfMInit_ImageListing"),
                ["-i", "%input_dir%", "-o", "%matches_dir%", "-d", "%camera_file_params%"] ],
            [   "Compute features",             #1
                os.path.join(OPENMVG_SFM_BIN,"openMVG_main_ComputeFeatures"),
                ["-i", "%matches_dir%/sfm_data.json", "-o", "%matches_dir%", "-m", "SIFT", "-n", "4"] ],
            [   "Compute matches",              #2
                os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeMatches"),
                ["-i", "%matches_dir%/sfm_data.json", "-o", "%matches_dir%", "-r", ".8"] ],
            [   "Incremental reconstruction",   #3
                os.path.join(OPENMVG_SFM_BIN, "openMVG_main_IncrementalSfM"),
                ["-i", "%matches_dir%/sfm_data.json", "-m", "%matches_dir%", "-o", "%reconstruction_dir%"] ],
            [   "Global reconstruction",        #4
                os.path.join(OPENMVG_SFM_BIN, "openMVG_main_GlobalSfM"),
                ["-i", "%matches_dir%/sfm_data.json", "-m", "%matches_dir%", "-o", "%reconstruction_dir%"] ],
            [   "Colorize Structure",           #5
                os.path.join(OPENMVG_SFM_BIN,"openMVG_main_ComputeSfM_DataColor"),
                ["-i", "%reconstruction_dir%/sfm_data.bin", "-o", "%reconstruction_dir%/colorized.ply"]],
            [   "Structure from Known Poses",   #6
                os.path.join(OPENMVG_SFM_BIN,"openMVG_main_ComputeStructureFromKnownPoses"),
                ["-i", "%reconstruction_dir%/sfm_data.bin", "-m", "%matches_dir%", "-f", "%matches_dir%/matches.f.bin", "-o", "%reconstruction_dir%/robust.bin"]],
            [   "Colorized robust triangulation",#7
                os.path.join(OPENMVG_SFM_BIN,"openMVG_main_ComputeSfM_DataColor"),
                ["-i", "%reconstruction_dir%/robust.bin", "-o", "%reconstruction_dir%/robust_colorized.ply"]],
            [   "Control Points Registration",  #8
                os.path.join(OPENMVG_SFM_BIN,"ui_openMVG_control_points_registration"),
                ["-i", "%reconstruction_dir%/sfm_data.bin"]],
            [   "Export to openMVS",            #9
                os.path.join(OPENMVG_SFM_BIN,"openMVG_main_openMVG2openMVS"),
                ["-i", "%reconstruction_dir%/sfm_data.bin", "-o", "%mvs_dir%/scene.mvs","-d","%mvs_dir%"]],
            [   "Densify point cloud",          #10
                os.path.join(OPENMVS_BIN,"DensifyPointCloud"),
                ["--input-file", "%mvs_dir%/scene.mvs", "--resolution-level","0", "-w", "%mvs_dir%"]],
            [   "Reconstruct the mesh",         #11
                os.path.join(OPENMVS_BIN,"ReconstructMesh"),
                ["%mvs_dir%/scene_dense.mvs","-w", "%mvs_dir%"]],
            [   "Refine the mesh",              #12
                os.path.join(OPENMVS_BIN,"RefineMesh"),
                ["%mvs_dir%/scene_dense_mesh.mvs","-w", "%mvs_dir%"]],
            [   "Texture the mesh",             #13
                os.path.join(OPENMVS_BIN,"TextureMesh"),
                ["scene_dense_mesh_refine.mvs", "-w","%mvs_dir%"]]
            ]

    def __getitem__(self, indice):
        return aStep(*self.steps_data[indice])

    def length(self):
        return len(self.steps_data)

    def apply_conf(self, conf):
        """ replace each %var% per conf.var value in steps data """
        for s in self.steps_data :
            o2=[]
            for o in s[2]:
                co=o.replace("%input_dir%",conf.input_dir)
                co=co.replace("%output_dir%",conf.output_dir)
                co=co.replace("%matches_dir%",conf.matches_dir)
                co=co.replace("%reconstruction_dir%",conf.reconstruction_dir)
                co=co.replace("%mvs_dir%",conf.mvs_dir)
                co=co.replace("%camera_file_params%",conf.camera_file_params)
                o2.append(co)
            s[2]=o2

steps=stepsStore()

## ARGS
import argparse
parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="Photogrammetry reconstruction with these steps : \r\n"+
        "\r\n".join(("\t%i. %s\t %s" % (t, steps[t].info, steps[t].cmd) for t in range(steps.length())))
    )
parser.add_argument('input_dir', help="the directory wich contains the pictures set.")
parser.add_argument('output_dir', help="the directory wich will contain the resulting files.")
parser.add_argument('--steps', type=int, nargs="+", default=SEQUENTIAL_STEPS, help="steps to process")

group = parser.add_argument_group('Passthrough',description="Option to be passed to command lines (remove - in front of option names)\r\ne.g. --1 p ULTRA to use the ULTRA preset in openMVG_main_ComputeFeatures")
for n in range(steps.length()) :
    group.add_argument('--'+str(n), nargs='+')

parser.parse_args(namespace=conf) #store args in the ConfContainer

## FOLDERS

def mkdir_ine(dirname):
    """Create the folder if not presents"""
    if not os.path.exists(dirname):
        os.mkdir(dirname)

#Absolute path for input and ouput dirs
conf.input_dir=os.path.abspath(conf.input_dir)
conf.output_dir=os.path.abspath(conf.output_dir)

if not os.path.exists(conf.input_dir):
    sys.exit("%s : path not found" % conf.input_dir)


conf.matches_dir = os.path.join(conf.output_dir, "matches")
conf.reconstruction_dir = os.path.join(conf.output_dir, "reconstruction")
conf.mvs_dir = os.path.join(conf.output_dir, "mvs")
conf.camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

mkdir_ine(conf.output_dir)
mkdir_ine(conf.matches_dir)
mkdir_ine(conf.reconstruction_dir)
mkdir_ine(conf.mvs_dir)

steps.apply_conf(conf)

## WALK
print "# Using input dir  :  %s" % conf.input_dir
print "#       output_dir :  %s" % conf.output_dir
print "# Steps  :  %s" % str(conf.steps)

if 2 in conf.steps :    #ComputeMatches
    if 4 in conf.steps : #GlobalReconstruction
        #Set the geometric_model of ComputeMatches to Essential
        steps[2].opt.extend(["-g","e"])


for cstep in conf.steps:
    printout("#%i. %s" % (cstep, steps[cstep].info), effect=INVERSE)

    opt=getattr(conf,str(cstep))
    if opt is not None :
        #add - sign to short options and -- to long ones
        for o in range(0,len(opt),2):
            if len(opt[o])>1:
                opt[o]='-'+opt[o]
            opt[o]='-'+opt[o]
    else:
        opt=[]

    #Remove steps[cstep].opt options now defined in opt
    for anOpt in steps[cstep].opt :
        if anOpt in opt :
            idx=steps[cstep].opt.index(anOpt)
            if DEBUG :
                print '#\t'+'Remove '+ str(anOpt) + ' from defaults options at id ' + str(idx)
            del steps[cstep].opt[idx:idx+2]

    cmdline = [steps[cstep].cmd] + steps[cstep].opt + opt

    if not DEBUG :
        pStep = subprocess.Popen(cmdline)
        pStep.wait()
    else:
        print '\t'+' '.join(cmdline)
