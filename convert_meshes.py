import trimesh
import os

# Convert all DAE files to STL
mesh_dir = 'meshes/visual'
files = ['base.dae', 'shoulder.dae', 'upperarm.dae', 'forearm.dae', 
         'wrist1.dae', 'wrist2.dae', 'wrist3.dae']

for filename in files:
    dae_path = os.path.join(mesh_dir, filename)
    stl_path = os.path.join(mesh_dir, filename.replace('.dae', '.stl'))
    
    print(f"Converting {filename}...")
    mesh = trimesh.load(dae_path)
    mesh.export(stl_path)
    print(f"  -> {stl_path}")

print("\n✅ All meshes converted to STL!")
