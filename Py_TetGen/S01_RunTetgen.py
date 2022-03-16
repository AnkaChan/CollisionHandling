import subprocess
from pathlib import Path
from os.path import join
class TetGenConfig:
    def __init__(s):
        # pq1.5YkV
        s.tetgenBin = r'..\Bin\tetgen.exe'
        s.outputVtk=True
        s.useQualityControl=True
        s.q_radiu_sedge_ratio = 1.5
        s.q_minimum_dihedral_angle_bound = 15
        s.preserveSurfaceMesh = True
        s.verbose = True
        s.outputT = True

def convertTetgenOutput(tetgenInFile):
    fp = Path(inFile)
    path = str(fp.parent)
    inVertsFile = join(path, fp.stem + '.1.node')
    inVertsFp = open(inVertsFile)
    lines = inVertsFp.readlines()[1:]

    vs = []
    vIds = []
    for line in lines:
        if line[0] == '#':
            continue

        l = line.split()
        vs.append([float(l[1]), float(l[2]), float(l[3])])

        assert len(vs) == int(l[0])+1
        vIds.append(int(l[0]))


    tets= []
    tIds = []
    inTetsFile = join(path, fp.stem + '.1.ele')
    inTetsFp = open(inTetsFile)
    lines = inTetsFp.readlines()[1:]
    for line in lines:
        if line[0] == '#':
            continue

        l = line.split()
        tets.append([int(l[1]), int(l[2]), int(l[3]),  int(l[4])])

        assert len(tets) == int(l[0])+1
        tIds.append(int(l[0]))

    outTFile = join(path,  fp.stem + '.t')

    with open(outTFile, 'w') as outTFp:
        for iV, v in zip(vIds, vs):
            outTFp.write("Vertex {:d} {:f} {:f} {:f}\n".format(iV, v[0], v[1], v[2]))

        for iT, t in zip(tIds, tets):
            outTFp.write("Tet {:d} {:d} {:d} {:d} {:d}\n".format(iT, t[0], t[1], t[2], t[3]))

# def convertTetgenOutput(tetgenInFile):
#     fp = Path(inFile)
#     path = str(fp.parent)
#     inVertsFile = join(path, fp.stem + '.1.node')
#     inVertsFp = open(inVertsFile)
#     lines = inVertsFp.readlines()[1:]
#
#     vs = []
#     for line in lines:
#         if line[0] == '#':
#             continue
#
#         l = line.split()
#         vs.append([float(l[1]), float(l[2]), float(l[3])])
#
#         assert len(vs) == int(l[0])+1
#
#     tets= []
#
#     inTetsFile = join(path, fp.stem + '.1.ele')
#     inTetsFp = open(inTetsFile)
#     lines = inTetsFp.readlines()[1:]
#     for line in lines:
#         if line[0] == '#':
#             continue
#
#         l = line.split()
#         tets.append([int(l[1]), int(l[2]), int(l[3]),  int(l[4])])
#
#         assert len(tets) == int(l[0])+1
#
#     outTFile = join(path,  fp.stem + '.t')
#
#     with open(outTFile, 'w') as outTFp:
#         for iV, v in enumerate(vs):
#             outTFp.write("Vertex {:d} {:f} {:f} {:f}\n".format(iV, v[0], v[1], v[2]))
#
#         for iT, t in enumerate(tets):
#             outTFp.write("Tet {:d} {:d} {:d} {:d} {:d}\n".format(iT, t[0], t[1], t[2], t[3]))

def runTetGen(inFile, cfg=TetGenConfig()):
    cmd = '-Fp'
    if cfg.useQualityControl:
        cmd = cmd + 'q' + str(cfg.q_radiu_sedge_ratio) + '/' + str(cfg.q_minimum_dihedral_angle_bound)

    if cfg.outputVtk:
        cmd = cmd + 'k'

    if cfg.preserveSurfaceMesh:
        cmd = cmd + 'Y'

    if cfg.verbose:
        cmd=cmd+'V'

    print([cfg.tetgenBin, cmd, inFile])
    subprocess.call([cfg.tetgenBin, cmd, inFile],  shell=True)

    if cfg.outputT:
        convertTetgenOutput(inFile)


if __name__ == '__main__':
    # inFile = r'C:\Code\02_Graphics\CollisionHandling\Cpp_CollisionDetection\Data\bun_10KV20KF.ply'
    # runTetGen(inFile)

    # inFile = r'C:\Code\02_Graphics\CollisionHandling\Cpp_CollisionDetection\Data\Dragon\dragon_low.ply'
    # runTetGen(inFile)

    # inFile = r'C:\Users\ankac\Downloads\dragon_tetgen\dragon\dragon_low.ply'
    # inFile = r'C:\Users\ankac\Downloads\dragon2048\dragon2048\dragon_low2048.ply'
    # convertTetgenOutput(inFile)

    inFile = r'C:\Code\02_Graphics\FEM-with-Collision-Handling\Data\Cube\Cube.ply'
    runTetGen(inFile)