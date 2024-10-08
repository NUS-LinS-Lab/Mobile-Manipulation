import numpy as np
import sys

def obj_loader(file_name,normalize=False,centered=False,output_obj_name=None):
  """It loads the vertices, vertice normals, the faces of a wavefront obj file.
  """
  vertices = []
  faces = []
  vnormals = []

  with open(file_name,'r') as fin:
    for line in fin:
      if line.startswith('#'):
        continue
      values = line.split()
      if len(values) < 1:
        continue
      if values[0] == 'v':
        v = list(map(float,values[1:4]))
        vertices.append(v)
      elif values[0] == 'vn':
        vn = list(map(float,values[1:4]))
        vnormals.append(vn)
      elif values[0] == 'f':
        face = []
        for v in values[1:]:
          w = v.split('/')
          face.append(int(w[0])) 
        faces.append(face) 
  
  vertices = np.array(vertices)
  faces = np.array(faces)
  print(vertices)
  print("faceces",faces)
  bbox_max = np.max(vertices,axis=0)
  bbox_min = np.min(vertices,axis=0)
  bbox_center = 0.5 * (bbox_max + bbox_min)
  bbox_rad =  np.linalg.norm(bbox_max - bbox_center)

  writelines = []
  with open(file_name,'r') as fin:
    for line in fin:
      if line.startswith('#'):
        writelines.append(line)
        continue
      values = line.split()
      if len(values) < 1:
        writelines.append(line)
        continue
      if values[0] == 'v':
        v = list(map(float,values[1:4])) 
        for vi in range(3):
          v[vi] -= bbox_center[vi]
          v[vi] /= (bbox_rad * 2.0)
        newline = 'v %f %f %f\n' % (v[0],v[1],v[2])
        writelines.append(newline)
      else:
        writelines.append(line)

  print("output_obj_name",output_obj_name)
  if output_obj_name is not None:
    with open(output_obj_name,'w') as fw:
      for line in writelines:
        fw.write(line)

if __name__ == "__main__":
  obj_name = sys.argv[1]
  output_obj_name = sys.argv[1][:-4] + '_normalized.obj'
  obj_loader(obj_name,centered=True,output_obj_name=output_obj_name)
